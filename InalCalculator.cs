using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Threading;
using KSP;
using UnityEngine;

namespace ImprovedNonAtmosphericLandings
{
    class InalCalculator : MonoBehaviour
    {
        //Thread stuff
        private Thread calcThread;
        private volatile Boolean shouldStop;
        private double resultUT;
        private String tMinus = string.Empty;
        private String status = string.Empty;
        bool success = false;

        //Calculation stuff
        private CelestialBody body;
        private Vessel vessel;
        private double dryMass;
        private double thrust;
        private double fuelFlow;
        private double t0;
        double mass;
        double burnTime;

        //Calculation parameters
        private float targetAltitudeValue = 50;
        private float altitudeError = 25;
        private float targetSpeedValue = 5F;
        private float speedError = 2F;

        //Return stuff
        private Vector3d initialRetrograde;
        private double etaUT;
        public bool calculating = false;

        #region Public getters

        public Vector3d GetInitialRetrograde()
        {
            return initialRetrograde;
        }

        public string GetETA()
        {
            TimeSpan timeToThrust = TimeSpan.FromSeconds(etaUT - Planetarium.GetUniversalTime());
            return string.Format("{0:D2}:{1:D2}:{2:D2}", timeToThrust.Hours, timeToThrust.Minutes, timeToThrust.Seconds);
        }

        public String GetStatus()
        {
            return status;
        }

        public string GetTMinus()
        {
            TimeSpan timeToThrust = TimeSpan.FromSeconds(resultUT - Planetarium.GetUniversalTime());
            return string.Format("{0:D2}:{1:D2}:{2:D2}", timeToThrust.Hours, timeToThrust.Minutes, timeToThrust.Seconds);
        }

        public double GetResultUT()
        {
            return resultUT;
        }

        public bool IsCalculating()
        {
            return calculating;
        }

        public bool IsComplete()
        {
            return success;
        }

        public double GetMaxSpeed()
        {
            return targetSpeedValue + speedError;
        }

        public double GetMinSpeed()
        {
            return targetSpeedValue - speedError;
        }

        public double GetThrust()
        {
            return thrust;
        }

        public double GetFinalMass()
        {
            return mass;
        }

        #endregion

        public void Stop()
        {

        }

        public void CalculateResult()
        {

            //TODO: Conditioning.
            //Must be nonatmospheric
            //Must be descending
            //Must be on suborbital path
            //Must have TWR > 1 at surface
            //Recommend disabling gimbal
            
            Logger.Info("Beginning calculation.");

            success = false;


            vessel = FlightGlobals.ActiveVessel;
            body = vessel.mainBody;

            CalculateThrustFuelFlowAndFuelMass();

            t0 = Planetarium.GetUniversalTime();
            
            //The time at which we start retrograde thrusting
            double startTimeOfThrust = 0;

            //The number of integration steps for each simulation
            int integrationSteps = 10;

            //What happens if we start burning immediately?
            IterationResult initialTest = simulateThrust(startTimeOfThrust, integrationSteps);
            
            //If immediate thrust does not bring the ship to a stop in time, we are in trouble.
            if (initialTest == IterationResult.TOO_LATE)
            {
                Logger.Info("Cannot bring ship to a stop in time.");
                return;
            }

            //These are the initial upper and lower bounds for initiating thrust in the loop below
            double shortTime = 0;
            double longTime = ComputeImpactTime();

            Logger.Info("Impact will occur in " + longTime.ToString("E2") + " seconds.");

            //Counter
            int i = 0;

            //The result of each iteration
            IterationResult iterationResult;
            
            //Main loop that computes 
            do
            {
                i++;

                startTimeOfThrust = (shortTime + longTime) / 2;

                Logger.Info("Computing approximation " + i + " with " + integrationSteps + " integration steps. Start time of thrust is " + startTimeOfThrust);
                
                //Iterate for this start time
                iterationResult = simulateThrust(startTimeOfThrust, integrationSteps);

                if (iterationResult == IterationResult.MORE_ACCURACY)
                {
                    Logger.Info("Increasing accuracy from " + integrationSteps + "integration steps.");
                    integrationSteps = integrationSteps * 2;
                }
                else if (iterationResult == IterationResult.TOO_EARLY)
                {
                    Logger.Info("Starting thrust at " + startTimeOfThrust + " was too early. Trying later.");
                    shortTime = startTimeOfThrust;
                }
                else if (iterationResult == IterationResult.TOO_LATE)
                {
                    Logger.Info("Starting thrust at " + startTimeOfThrust + " was too late. Trying earlier.");
                    longTime = startTimeOfThrust;
                }
                

            } while (iterationResult != IterationResult.SUCCESS);

            Logger.Info("Calculation successful. Thrust should start at UT " + (t0 + startTimeOfThrust));

            
            resultUT = (t0 + startTimeOfThrust);
            etaUT = resultUT + burnTime;
            success = true;
        }
        
        private IterationResult simulateThrust(double startTimeOfThrust, int integrationSteps)
        {
            //The vector arrays that will store upper- and lower- estimates for the initial position and velocity in each timestep
            EstimatePair<Vector3d> initialPosition = new EstimatePair<Vector3d>();
            EstimatePair<Vector3d> initialVelocity = new EstimatePair<Vector3d>();

            //The orbital status of the vessel immediately before thrust
            initialPosition.lower = (vessel.GetOrbit().getPositionAtUT(t0 + startTimeOfThrust));
            initialVelocity.lower = OrbitToWorld(vessel.GetOrbit().getOrbitalVelocityAtUT(t0 + startTimeOfThrust));
            initialPosition.upper = initialPosition.lower;
            initialVelocity.upper = initialVelocity.lower;
            initialRetrograde = -initialVelocity.lower;
            mass = vessel.totalMass; //Known exactly between time steps

            //Resulting quantities
            EstimatePair<Vector3d> finalPosition = new EstimatePair<Vector3d>();
            EstimatePair<Vector3d> finalVelocity = new EstimatePair<Vector3d>();
            EstimatePair<double> finalSrfSpeed = new EstimatePair<double>();
            EstimatePair<double> finalSrfAltitude = new EstimatePair<double>();

            //Other integration step values
            Vector3d gravAcc;
            Vector3d thrustAcc;

            //Based on the initial velocity, compute an approximate stopping duration and use this to estimate a suitable time step
            double initialSrfSpeed = Vector3d.Magnitude(GetSrfVelocity(initialPosition.lower, initialVelocity.lower));
            double expectedTimeToStop = initialSrfSpeed * mass / (thrust - Vector3d.Magnitude(FlightGlobals.getGeeForceAtPosition(initialPosition.lower)));
            double timeStep = (double)expectedTimeToStop / integrationSteps;

            Logger.Info("Thrust begins " + startTimeOfThrust.ToString("E2") + " from now. Timestep is " + timeStep);
            Logger.Info("At start of burn, surface altitude is " + GetSrfAltitude(initialPosition.lower, startTimeOfThrust).ToString("N1") + "m and surface speed is " + initialSrfSpeed.ToString("N1"));
            Logger.Info("Angle between orbital velocity and surface velocity is " + Vector3d.Angle(initialVelocity.lower, GetSrfVelocity(initialPosition.lower, initialVelocity.lower)));

            //The time from start of burn
            burnTime = 0;

            //The target surface heights and velocities
            EstimatePair<double> targetAltitude = new EstimatePair<double>(targetAltitudeValue - altitudeError, targetAltitudeValue + altitudeError);
            EstimatePair<double> targetSpeed = new EstimatePair<double>(targetSpeedValue - speedError, targetSpeedValue + speedError);

            IterationResult iterationResult = IterationResult.INCOMPLETE;
            
            IterationResult[,,,] decisionArray = new IterationResult[3, 3, 3, 3];
            /* OLD VERSION:
            decisionArray[0, 2, 0, 2] = IterationResult.TOO_LATE;
            decisionArray[0, 2, 0, 1] = IterationResult.STEP_BACK;
            decisionArray[0, 2, 1, 1] = IterationResult.MORE_ACCURACY;
            decisionArray[0, 2, 2, 1] = IterationResult.MORE_ACCURACY;
            decisionArray[0, 2, 0, 0] = IterationResult.STEP_BACK;
            decisionArray[0, 2, 1, 0] = IterationResult.MORE_ACCURACY;
            decisionArray[0, 2, 2, 0] = IterationResult.MORE_ACCURACY;
            decisionArray[1, 2, 1, 0] = IterationResult.MORE_ACCURACY;
            decisionArray[1, 2, 2, 0] = IterationResult.MORE_ACCURACY;
            decisionArray[0, 1, 0, 1] = IterationResult.STEP_BACK;
            decisionArray[0, 1, 1, 0] = IterationResult.STEP_BACK;
            decisionArray[0, 1, 1, 1] = IterationResult.STEP_BACK;
            decisionArray[0, 1, 2, 1] = IterationResult.MORE_ACCURACY;
            decisionArray[0, 1, 0, 0] = IterationResult.STEP_BACK;
            decisionArray[0, 1, 1, 0] = IterationResult.STEP_BACK;
            decisionArray[0, 1, 2, 0] = IterationResult.MORE_ACCURACY;
            decisionArray[1, 1, 1, 1] = IterationResult.SUCCESS;
            decisionArray[1, 1, 1, 0] = IterationResult.STEP_BACK;
            decisionArray[1, 1, 2, 0] = IterationResult.MORE_ACCURACY;
            decisionArray[0, 0, 0, 0] = IterationResult.STEP_BACK;
            decisionArray[0, 0, 1, 0] = IterationResult.STEP_BACK;
            decisionArray[0, 0, 2, 0] = IterationResult.STEP_BACK;
            decisionArray[1, 0, 1, 0] = IterationResult.STEP_BACK;
            decisionArray[1, 0, 2, 0] = IterationResult.STEP_BACK;
            decisionArray[2, 0, 2, 0] = IterationResult.TOO_EARLY;
            */

            decisionArray[0, 2, 0, 2] = IterationResult.TOO_LATE;
            decisionArray[0, 2, 0, 1] = IterationResult.TOO_LATE;
            decisionArray[0, 2, 1, 1] = IterationResult.TOO_LATE;
            decisionArray[0, 2, 2, 1] = IterationResult.MORE_ACCURACY;
            decisionArray[0, 2, 0, 0] = IterationResult.STEP_BACK;
            decisionArray[0, 2, 1, 0] = IterationResult.MORE_ACCURACY;
            decisionArray[0, 2, 2, 0] = IterationResult.MORE_ACCURACY;
            decisionArray[1, 2, 1, 0] = IterationResult.MORE_ACCURACY;
            decisionArray[1, 2, 2, 0] = IterationResult.MORE_ACCURACY;
            decisionArray[0, 1, 0, 1] = IterationResult.STEP_BACK;
            decisionArray[0, 1, 1, 0] = IterationResult.STEP_BACK;
            decisionArray[0, 1, 1, 1] = IterationResult.STEP_BACK;
            decisionArray[0, 1, 2, 1] = IterationResult.MORE_ACCURACY;
            decisionArray[0, 1, 0, 0] = IterationResult.STEP_BACK;
            decisionArray[0, 1, 1, 0] = IterationResult.STEP_BACK;
            decisionArray[0, 1, 2, 0] = IterationResult.MORE_ACCURACY;
            decisionArray[1, 1, 1, 1] = IterationResult.SUCCESS;
            decisionArray[1, 1, 1, 0] = IterationResult.STEP_BACK;
            decisionArray[1, 1, 2, 0] = IterationResult.TOO_EARLY;
            decisionArray[0, 0, 0, 0] = IterationResult.STEP_BACK;
            decisionArray[0, 0, 1, 0] = IterationResult.STEP_BACK;
            decisionArray[0, 0, 2, 0] = IterationResult.STEP_BACK;
            decisionArray[1, 0, 1, 0] = IterationResult.STEP_BACK;
            decisionArray[1, 0, 2, 0] = IterationResult.TOO_EARLY;
            decisionArray[2, 0, 2, 0] = IterationResult.TOO_EARLY;
            
            //Counts the number of step-backs to avoid oscillation
            int stepBackCounter = 0;

            //This vector contains flags to detect whether the vessel is rising according to each estimate
            EstimatePair<bool> rising = new EstimatePair<bool>(false, false);

            //The following loop describes a single time step, testing an over- and under- estimate
            do
            {
                Logger.Info("Calculating step. Timestep is " + timeStep + ". Total burn time before this step is " + burnTime);

                //First generate an estimate using the initial values of the integration step
                finalPosition.lower = (Vector3d) initialPosition.lower + (Vector3d) initialVelocity.lower * timeStep; //This line is altering initialPosition somehow MUST GET TO THE BOTTOM OF THIS
                gravAcc = FlightGlobals.getGeeForceAtPosition(initialPosition.lower);
                thrustAcc = thrust / mass * -GetSrfVelocity(initialPosition.lower, initialVelocity.lower).normalized;
                finalVelocity.lower = (Vector3d) initialVelocity.lower + (gravAcc + thrustAcc) * timeStep;

                //Then generate an estimate using the final values of the integration step
                gravAcc = FlightGlobals.getGeeForceAtPosition(finalPosition.lower); //Assume this is close enough
                thrustAcc = thrust / (mass - fuelFlow * timeStep) * -GetSrfVelocity(finalPosition.lower, finalVelocity.lower).normalized;
                finalVelocity.upper = initialVelocity.upper + (gravAcc + thrustAcc) * timeStep;
                finalPosition.upper = initialPosition.upper + finalVelocity.upper * timeStep;
                
                Logger.Info("Final position is " + finalPosition.lower.ToString());

                //TODO: Complete this properly.
                if (mass < 0)
                {
                    Logger.Error("Couldn't complete maneuver without running out of fuel.");
                    return IterationResult.INCOMPLETE;
                }

                //Calculate the things we care about: surface height and speed
                for (int j = 0; j < 2; j++)
                { 
                    finalSrfSpeed.SetByIndex(j, Vector3d.Magnitude(GetSrfVelocity(finalPosition.GetByIndex(j), finalVelocity.GetByIndex(j))));
                    finalSrfAltitude.SetByIndex(j, GetSrfAltitude(finalPosition.GetByIndex(j), startTimeOfThrust + burnTime));
                }


                //TODO: Test that each estimate is lower/higher as expected
                if (finalSrfSpeed.lower > finalSrfSpeed.upper)
                {
                    finalSrfSpeed.Switch();
                }
                if (finalSrfAltitude.lower > finalSrfAltitude.upper)
                {
                    finalSrfAltitude.Switch();
                }

                Logger.Info("Final surface speed estimates are [" + finalSrfSpeed.lower.ToString("E2") + ", " + finalSrfSpeed.upper.ToString("E2") +
                    "]. Final surface altitude estimates are [" + finalSrfAltitude.lower.ToString("E2") + ", " + finalSrfAltitude.upper.ToString("E2") + "].");

                //Determine if we have overshot and started rising again
                for (int j = 0; j < 2; j++)
                {
                    if (Vector3d.Dot(finalVelocity.GetByIndex(j), body.position - finalPosition.GetByIndex(j)) < 0)
                    {
                        rising.SetByIndex(j, true);
                    }
                }
                
                //Figure out what's going on
                if (finalSrfAltitude.lower < targetAltitude.upper || finalSrfSpeed.lower < targetSpeed.upper || rising.lower || rising.upper) //A broad test to reduce the processing for ordinary iteration cases
                {
                    double[] upperPoint = new double[] { finalSrfAltitude.lower, finalSrfSpeed.upper };
                    double[] lowerPoint = new double[] { finalSrfAltitude.upper, finalSrfSpeed.lower };
                    
                    EstimatePair<double[]> points = new EstimatePair<double[]>(upperPoint, lowerPoint);
                    int[] regions = new int[4]; //This contains the location of the two points in the 3-by-3 space in the form [upper.x, upper.y, lower.x, lower.y]
                    for (int j = 0; j < 2; j++)
                    {
                        if (points.GetByIndex(j)[0] < targetAltitude.lower) { regions[2 * j] = 0; }
                        else if (points.GetByIndex(j)[0] > targetAltitude.upper) { regions[2 * j] = 2; }
                        else { regions[2 * j] = 1; }

                        if (points.GetByIndex(j)[1] < targetSpeed.lower) { regions[2 * j + 1] = 0; }
                        else if (points.GetByIndex(j)[1] > targetSpeed.upper) { regions[2 * j + 1] = 2; }
                        else { regions[2 * j + 1] = 1; }
                        
                    }
                    
                    if (rising.lower || rising.upper)
                    {
                        Logger.Info("Detected rising vessel.");

                        //If at least one estimate suggests vessel is rising, set the lower point to region 0
                        regions[3] = 0;

                        if (rising.lower && rising.upper)
                        {
                            //If both estimates suggest vessel is rising, set the upper point to region 0 as well
                            regions[1] = 0;
                        }
                    }
                    
                    Logger.Info("Points lie in regions [" + regions[0] + ", " + regions[1] + "] and [" + regions[2] + ", " + regions[3] + "].");
                    
                    try
                    {
                        iterationResult = decisionArray[regions[0], regions[1], regions[2], regions[3]];
                    }
                    catch (Exception e)
                    {
                        Logger.Warn(e.Message);
                        iterationResult = IterationResult.INCOMPLETE;
                    }
                    Logger.Info("Decision was iteration result: " + iterationResult);

                    if (iterationResult == IterationResult.STEP_BACK)
                    {
                        stepBackCounter++;
                        if (stepBackCounter > 10)
                        {
                            Logger.Info("Too many step-backs, detected oscillation. Requesting more accuracy.");
                            return IterationResult.MORE_ACCURACY;
                        }
                        else
                        {
                            Logger.Info("Stepping back.");
                            iterationResult = IterationResult.INCOMPLETE;
                            timeStep = timeStep / 2;
                            
                            continue;
                        }
                    }
                }

                //Progress values:
                Logger.Info("Progressing values");
                mass = mass - fuelFlow * timeStep;
                initialPosition = finalPosition;
                initialVelocity = finalVelocity;
                burnTime += timeStep;

            } while (iterationResult == IterationResult.INCOMPLETE);
            
            return iterationResult;
        }

        #region enums        

        private enum IterationResult
        {
            INCOMPLETE, TOO_EARLY, TOO_LATE, MORE_ACCURACY, STEP_BACK, SUCCESS
        }

        #endregion

        #region Time to impact copied and modified from KER

        public double ComputeImpactTime()
        {
            //do impact site calculations
            double impactTime = 0;
            double impactAltitude = 0;
            double impactLongitude = 0;
            double impactLatitude = 0;
            var e = vessel.orbit.eccentricity;
            //get current position direction vector
            var currentpos = RadiusDirection(vessel.orbit.trueAnomaly);
            //calculate longitude in inertial reference frame from that
            var currentirflong = 180 * Math.Atan2(currentpos.x, currentpos.y) / Math.PI;

            //experimentally determined; even for very flat trajectories, the errors go into the sub-millimeter area after 5 iterations or so
            const int impactiterations = 6;

            //do a few iterations of impact site calculations
            for (var i = 0; i < impactiterations; i++)
            {
                double impacttheta = 0;
                if (e > 0)
                {
                    //in this step, we are using the calculated impact altitude of the last step, to refine the impact site position
                    impacttheta = -180 * Math.Acos((vessel.orbit.PeR * (1 + e) / (vessel.mainBody.Radius + impactAltitude) - 1) / e) / Math.PI;
                }

                //calculate time to impact
                impactTime = vessel.orbit.timeToPe - TimeToPeriapsis(impacttheta);
                //calculate position vector of impact site
                var impactpos = RadiusDirection(impacttheta);
                //calculate longitude of impact site in inertial reference frame
                var impactirflong = 180 * Math.Atan2(impactpos.x, impactpos.y) / Math.PI;
                var deltairflong = impactirflong - currentirflong;
                //get body rotation until impact
                var bodyrot = 360 * impactTime / vessel.mainBody.rotationPeriod;
                //get current longitude in body coordinates
                var currentlong = vessel.longitude;
                //finally, calculate the impact longitude in body coordinates
                impactLongitude = NormAngle(currentlong - deltairflong - bodyrot);
                //calculate impact latitude from impact position
                impactLatitude = 180 * Math.Asin(impactpos.z / impactpos.magnitude) / Math.PI;
                //calculate the actual altitude of the impact site
                //altitude for long/lat code stolen from some ISA MapSat forum post; who knows why this works, but it seems to.
                var rad = QuaternionD.AngleAxis(impactLongitude, Vector3d.down) * QuaternionD.AngleAxis(impactLatitude, Vector3d.forward) * Vector3d.right;
                if (vessel.mainBody.pqsController != null)
                {
                    impactAltitude = vessel.mainBody.pqsController.GetSurfaceHeight(rad) - vessel.mainBody.pqsController.radius;
                }
                else
                {
                    impactAltitude = 0;
                }
                if ((impactAltitude < 0) && vessel.mainBody.ocean)
                {
                    impactAltitude = 0;
                }
            }

            return impactTime;

        }

        private Vector3d RadiusDirection(double theta)
        {
            theta = Math.PI * theta / 180;
            var omega = Math.PI * vessel.orbit.argumentOfPeriapsis / 180;
            var incl = Math.PI * vessel.orbit.inclination / 180;

            var costheta = Math.Cos(theta);
            var sintheta = Math.Sin(theta);
            var cosomega = Math.Cos(omega);
            var sinomega = Math.Sin(omega);
            var cosincl = Math.Cos(incl);
            var sinincl = Math.Sin(incl);

            Vector3d result;

            result.x = cosomega * costheta - sinomega * sintheta;
            result.y = cosincl * (sinomega * costheta + cosomega * sintheta);
            result.z = sinincl * (sinomega * costheta + cosomega * sintheta);

            return result;
        }

        private double TimeToPeriapsis(double theta)
        {
            var e = vessel.orbit.eccentricity;
            var a = vessel.orbit.semiMajorAxis;
            var rp = vessel.orbit.PeR;
            var mu = vessel.mainBody.gravParameter;

            if (e == 1.0)
            {
                var D = Math.Tan(Math.PI * theta / 360.0);
                var M = D + D * D * D / 3.0;
                return (Math.Sqrt(2.0 * rp * rp * rp / mu) * M);
            }
            if (a > 0)
            {
                var cosTheta = Math.Cos(Math.PI * theta / 180.0);
                var cosE = (e + cosTheta) / (1.0 + e * cosTheta);
                var radE = Math.Acos(cosE);
                var M = radE - e * Math.Sin(radE);
                return (Math.Sqrt(a * a * a / mu) * M);
            }
            if (a < 0)
            {
                var cosTheta = Math.Cos(Math.PI * theta / 180.0);
                var coshF = (e + cosTheta) / (1.0 + e * cosTheta);
                var radF = ACosh(coshF);
                var M = e * Math.Sinh(radF) - radF;
                return (Math.Sqrt(-a * a * a / mu) * M);
            }

            return 0;
        }

        private double NormAngle(double ang)
        {
            if (ang > 180)
            {
                ang -= 360 * Math.Ceiling((ang - 180) / 360);
            }
            if (ang <= -180)
            {
                ang -= 360 * Math.Floor((ang + 180) / 360);
            }

            return ang;
        }

        public static double ACosh(double x)
        {
            return (Math.Log(x + Math.Sqrt((x * x) - 1.0)));
        }

        #endregion

        #region My own private helper methods

        private double GetSrfAltitude(Vector3d worldPos, double deltaT)
        {
            double bodyRot = 360 * deltaT / body.rotationPeriod;

            double lat = body.GetLatitude(worldPos);
            double lon = NormAngle(body.GetLongitude(worldPos) - bodyRot);

            var rad = QuaternionD.AngleAxis(lon, Vector3d.down) * QuaternionD.AngleAxis(lat, Vector3d.forward) * Vector3d.right;
            var terrainHeight = body.pqsController.GetSurfaceHeight(rad);

            return Vector3d.Magnitude(worldPos - body.position) - terrainHeight;
        }

        private Vector3d GetSrfVelocity(Vector3d worldPos, Vector3d worldVelocity)
        {
            return worldVelocity - body.getRFrmVel(worldPos);
        }

        private Vector3d GetWorldVelocity(Vector3d worldPos, Vector3d srfVelocity)
        {
            return srfVelocity + body.getRFrmVel(worldPos);
        }

        private Vector3d OrbitToWorld(Vector3d orbitVector)
        {
            return new Vector3d(orbitVector.x, orbitVector.z, orbitVector.y);
        }

        private void CalculateThrustFuelFlowAndFuelMass()
        {
            List<Part> parts = vessel.GetActiveParts();

            dryMass = vessel.totalMass;
            thrust = 0;
            fuelFlow = 0;

            List<PartResourceDefinition> requiredResources = new List<PartResourceDefinition>();
            List<PartResource> resourcesToBeUsed = new List<PartResource>();

            foreach (Part part in parts)
            {
                List<ModuleEngines> engineModules = part.Modules.GetModules<ModuleEngines>();
                
                foreach (ModuleEngines module in engineModules)
                {
                    BaseFieldList fieldList = module.Fields;
                    bool ignited = (bool) fieldList.GetValue("EngineIgnited");
                    if (ignited)
                    {
                        double thrustLimiter = Convert.ToDouble(fieldList.GetValue("thrustPercentage"));
                        thrust += module.maxThrust * thrustLimiter / 100f;
                        fuelFlow += module.maxFuelFlow * thrustLimiter / 100f;
                        requiredResources.AddRange(module.GetConsumedResources());
                    }
                }

                foreach (PartResourceDefinition resource in requiredResources)
                {
                    if (requiredResources.Contains(resource))
                    {
                        part.GetConnectedResources(resource.id, resource.resourceFlowMode, resourcesToBeUsed);
                    }
                }
                
            }

            foreach(PartResource resourceStore in resourcesToBeUsed)
            {
                Logger.Info("Found " + resourceStore.amount + " units of resource " + resourceStore.resourceName + " in part " + resourceStore.part.name);
                dryMass -= resourceStore.amount;
            }

            Logger.Info("Total thrust is: " + thrust + ", total fuel flow is: " + fuelFlow + ", dry mass is " + dryMass + ".");
            
        }

        #endregion

        class EstimatePair<T> : IEnumerable
        {
            public T lower;
            public T upper;

            public EstimatePair()
            {
                //this.lower = default(T);
                //this.upper = default(T);
            }

            public EstimatePair(T lower, T upper)
            {
                this.lower = lower;
                this.upper = upper;
            }

            public void Switch()
            {
                var temp = lower;
                lower = upper;
                upper = temp;
            }

            public T GetByIndex(int index)
            {
                switch (index)
                {
                    case 0: return lower;
                    case 1: return upper;
                    default: throw new ArgumentOutOfRangeException("Only index of 0 or 1 is valid for this method.");
                }
            }

            public void SetByIndex(int index, T value)
            {
                switch (index)
                {
                    case 0: lower = value; break;
                    case 1: upper = value; break;
                    default: throw new ArgumentOutOfRangeException("Only index of 0 or 1 is valid for this method.");
                }
            }

            public IEnumerator GetEnumerator()
            {
                yield return lower;
                yield return upper;
            }

            IEnumerator IEnumerable.GetEnumerator()
            {
                return GetEnumerator();
            }

        }
    }
}
