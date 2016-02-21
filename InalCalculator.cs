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
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    class InalCalculator : MonoBehaviour
    {
        //Trajectory loop fields
        private int trajectoryCount;
        private double shortTime;
        private double longTime;
        private int integrationSteps;
        private double startTimeOfThrust;

        //Integration loop fields
        EstimatePair<Vector3d> initialPosition = new EstimatePair<Vector3d>();
        EstimatePair<Vector3d> initialVelocity = new EstimatePair<Vector3d>();
        double burnTime;
        int stepBackCounter;
        double timeStep;
        IterationResult[,,,] decisionArray;
        int calculationCount;
        EstimatePair<bool> rising;
        EstimatePair<double> finalSrfAltitude;
        EstimatePair<double> finalSrfSpeed;

        //General calculation fields
        private CelestialBody body;
        private Vessel vessel;
        private double dryMass;
        private double thrust;
        private double fuelFlow;
        private double t0;
        double changingMass;
        EstimatePair<double> targetAltitude;
        EstimatePair<double> targetSpeed;
        private double offerUT;
        private double offerBurnTime;
        private float vesselApproxHeight;

        //Calculation parameters
        private int calcsPerUpdate = 2;
        private float maxAltitude = 50;
        private float minAltitude = 10;
        private float maxSpeed = 5f;
        private float minSpeed = 0f;

        //Return fields
        private Vector3d initialRetrograde;
        private double etaUT;
        private double resultUT;
        private String tMinus = string.Empty;
        private String status = string.Empty;
        private string altitudeOffer = String.Empty;
        private string speedOffer = String.Empty;
        private double maxBurnTime;

        //Readonly constants
        private readonly double minimumTimeStep = 0.02d;

        #region Public getters

        public double GetTimeStep()
        {
            return timeStep;
        }

        public float GetVesselHeight()
        {
            return vesselApproxHeight;
        }

        public string GetAltitudeOffer()
        {
            return altitudeOffer;
        }

        public string GetSpeedOffer()
        {
            return speedOffer;
        }

        public Vector3d GetInitialRetrograde()
        {
            return initialRetrograde;
        }

        public string GetETAString()
        {
            TimeSpan timeToThrust = TimeSpan.FromSeconds(etaUT - Planetarium.GetUniversalTime());
            return string.Format("{0:D2}:{1:D2}:{2:D2}", timeToThrust.Hours, timeToThrust.Minutes, timeToThrust.Seconds);
        }

        public string GetStatus()
        {
            return status;
        }

        public string GetTMinusString()
        {
            TimeSpan timeToThrust = TimeSpan.FromSeconds(resultUT - Planetarium.GetUniversalTime());
            return string.Format("{0:D2}:{1:D2}:{2:D2}", timeToThrust.Hours, timeToThrust.Minutes, timeToThrust.Seconds);
        }

        public double GetResultUT()
        {
            return resultUT;
        }

        public double GetMaxSpeed()
        {
            return maxSpeed;
        }

        public double GetMinSpeed()
        {
            return minSpeed;
        }

        public double GetThrust()
        {
            return thrust;
        }

        public double GetFinalMass()
        {
            return changingMass;
        }

        public double GetTrajectory()
        {
            return trajectoryCount;
        }

        public string GetBurnTimeString()
        {
            return burnTime.ToString("G5") + "s/" + maxBurnTime.ToString("G5") + "s";
        }

        public double GetStartTimeOfThrust()
        {
            return startTimeOfThrust;
        }

        #endregion

        #region Public setters

        public void SetMaxSpeed(float value)
        {
            maxSpeed = value;
        }

        #endregion

        #region MonoBehaviour inherited methods

        /// <summary>
        /// Initialises the decision array
        /// </summary>
        public void Awake()
        {
            //The decision array for the outcome of each integration step
            decisionArray = new IterationResult[3, 3, 3, 3];
            decisionArray.Initialize();
            decisionArray[0, 2, 0, 2] = IterationResult.TOO_LATE;
            decisionArray[0, 2, 0, 1] = IterationResult.TOO_LATE;
            decisionArray[0, 2, 1, 1] = IterationResult.TOO_LATE;
            decisionArray[0, 2, 0, 0] = IterationResult.STEP_BACK;
            decisionArray[0, 2, 1, 0] = IterationResult.STEP_BACK;
            decisionArray[0, 2, 2, 0] = IterationResult.MORE_ACCURACY;
            decisionArray[0, 1, 0, 1] = IterationResult.STEP_BACK;
            decisionArray[0, 1, 1, 0] = IterationResult.STEP_BACK;
            decisionArray[0, 1, 1, 1] = IterationResult.STEP_BACK;
            decisionArray[0, 1, 0, 0] = IterationResult.STEP_BACK;
            decisionArray[0, 1, 1, 0] = IterationResult.STEP_BACK;
            decisionArray[0, 1, 2, 0] = IterationResult.STEP_BACK;
            decisionArray[1, 1, 1, 1] = IterationResult.SUCCESS;
            decisionArray[1, 1, 1, 0] = IterationResult.STEP_BACK;
            decisionArray[1, 1, 2, 0] = IterationResult.TOO_EARLY;
            decisionArray[0, 0, 0, 0] = IterationResult.STEP_BACK;
            decisionArray[0, 0, 1, 0] = IterationResult.STEP_BACK;
            decisionArray[0, 0, 2, 0] = IterationResult.STEP_BACK;
            decisionArray[1, 0, 1, 0] = IterationResult.STEP_BACK;
            decisionArray[1, 0, 2, 0] = IterationResult.TOO_EARLY;
            decisionArray[2, 0, 2, 0] = IterationResult.TOO_EARLY;

            //Disable this monobehaviour
            this.enabled = false;
        }

        /// <summary>
        /// Continues the calculation on the next GUI update
        /// </summary>
        public void OnGUI()
        {
            calculationCount = 0;
            
            while (calculationCount < calcsPerUpdate)
            {
                IterationResult result = DoNextIntegrationSteps();

                Logger.Debug("Iteration result was " + result);

                if (result == IterationResult.SUCCESS)
                {
                    //TODO: check that we have enough fuel - if not, maybe just notify the user since we may be only slightly off
                    
                    //Set result fields
                    resultUT = (t0 + startTimeOfThrust);
                    etaUT = resultUT + burnTime;

                    Logger.Info("Calculation was successful. Thrust should begin at UT " + resultUT);

                    //Notify the GUI of success
                    GameObject.FindObjectOfType<MainGUI>().SetCalculationSuccessful();

                    //Some logging to debug landing inaccuracy
                    Vector3d startPos = vessel.GetOrbit().getPositionAtUT(resultUT);
                    Vector3d startVel = vessel.GetOrbit().getOrbitalVelocityAtUT(resultUT);
                    Logger.Debug("Predicted vessel state at thrust start: srfAltitude = " + GetSrfAltitude(startPos, startTimeOfThrust) + ", srfSpeed = " + Vector3d.Magnitude(GetSrfVelocity(startPos, OrbitToWorld(startVel))));

                    //Kill this monobehaviour
                    Disable();
                    break;
                }
                else if (result == IterationResult.TOO_LATE && startTimeOfThrust == 0)
                {
                    //It is too late to begin thrusting, vessel cannot be stopped in time
                    GameObject.FindObjectOfType<MainGUI>().SetFatalError(Messages.TooLateToStartThrusting);
                    Disable();
                    break;
                }
                else if (result == IterationResult.TOO_LATE || result == IterationResult.TOO_EARLY || result == IterationResult.MORE_ACCURACY)
                {
                    if (result == IterationResult.TOO_EARLY)
                    {
                        //Remember this result in case the user accepts it
                        altitudeOffer = "[" + finalSrfAltitude.lower.ToString("G5") + ", " + finalSrfAltitude.upper.ToString("G5") + "]";
                        speedOffer = "[" + finalSrfSpeed.lower.ToString("G5") + ", " + finalSrfSpeed.upper.ToString("G5") + "]";
                        offerUT = t0 + startTimeOfThrust;
                        maxBurnTime = Mathf.Max((float)burnTime, (float)maxBurnTime);
                        offerBurnTime = burnTime;
                    }

                    DoNextTrajectory(result);
                }
                else if (result == IterationResult.OUT_OF_FUEL)
                {
                    //TODO: Actually implement this checker in integration steps
                    GameObject.FindObjectOfType<MainGUI>().SetFatalError(Messages.InsufficientFuel);
                    Disable();
                    break;
                }
                else
                {
                    if (result != IterationResult.INCOMPLETE)
                    {
                        Logger.Error("Unexpected iteration result");
                    }
                }
            }
        }

        #endregion

        /// <summary>
        /// Initialises the calculation
        /// </summary>
        public void BeginCalculation()
        {
            //TODO: Recommendations
            //Recommend disabling gimbal
            //Recommend disabling RCS

            vessel = FlightGlobals.ActiveVessel;
            body = vessel.mainBody;
            changingMass = vessel.totalMass;

            //Initialises the thrust, fuel flow, fuel mass and vessel height fields
            CalculateVesselParameters();

            //Check that we are descending
            if (vessel.verticalSpeed > 0)
            {
                GameObject.FindObjectOfType<MainGUI>().SetFatalError(Messages.VesselMustBeDescending);
                Disable();
                return;
            }
            //Check that the thrust is non-zero
            if (thrust == 0)
            {
                GameObject.FindObjectOfType<MainGUI>().SetFatalError(Messages.NoActiveEnginesDetected);
                Disable();
                return;
            }
            //Check that the body has no atmosphere
            if (body.atmosphere)
            {
                GameObject.FindObjectOfType<MainGUI>().SetFatalError(Messages.BodyMustHaveNoAtmosphere);
                Disable();
                return;
            }
            //Check that the vessel is on a suborbital trajectory
            if (Double.IsNaN(ComputeImpactTime()))
            {
                GameObject.FindObjectOfType<MainGUI>().SetFatalError(Messages.MustBeOnSubOrbitalTrajectory);
                Disable();
                return;
            }
            //Check that the vessel is not currently landed
            if (vessel.Landed)
            {
                GameObject.FindObjectOfType<MainGUI>().SetFatalError(Messages.VesselIsCurrentlyLanded);
                Disable();
                return;
            }

            //Reset previous fields that are displayed in the GUI to avoid user confusion
            altitudeOffer = String.Empty;
            maxBurnTime = 0;
            offerUT = 0;

            //Enable this monobehaviour
            this.enabled = true;

            //Set the current time t0
            t0 = Planetarium.GetUniversalTime();

            //The time (from now) at which we start retrograde thrusting for the first trajectory
            startTimeOfThrust = 0;

            //The number of integration steps for the initial simulation
            integrationSteps = 10;

            //This vector contains flags to detect whether the vessel is rising according to each estimate
            rising = new EstimatePair<bool>(false, false);

            //Set the target altitude and speed upper and lower limits, accounting for the height of the vessel
            targetAltitude = new EstimatePair<double>(minAltitude + vesselApproxHeight, maxAltitude + vesselApproxHeight);
            targetSpeed = new EstimatePair<double>(minSpeed, maxSpeed);

            //Set the initial position and velocity for thrust starting immediately
            SetInitialPositionAndVelocity(t0);

            //Based on the initial velocity, compute an approximate stopping duration and use this to estimate a suitable time step
            double initialSrfSpeed = Vector3d.Magnitude(GetSrfVelocity(initialPosition.lower, initialVelocity.lower));
            double expectedTimeToStop = initialSrfSpeed * changingMass / (thrust - Vector3d.Magnitude(FlightGlobals.getGeeForceAtPosition(initialPosition.lower)));
            timeStep = (double)expectedTimeToStop / integrationSteps;

            //These are the initial upper and lower bounds for initiating thrust in the trajectory loop
            shortTime = 0;
            longTime = ComputeImpactTime();

            Logger.Debug("With no thrust, impact will occur in " + longTime.ToString("E2") + " seconds.");

            //Trajectory counter
            trajectoryCount = 1;
        }
        
        private void DoNextTrajectory(IterationResult previousResult)
        {
            trajectoryCount++;

            if (previousResult == IterationResult.MORE_ACCURACY)
            {
                Logger.Debug("Increasing accuracy from " + integrationSteps + "integration steps.");

                integrationSteps = integrationSteps * 2;
            }
            else if (previousResult == IterationResult.TOO_EARLY)
            {
                Logger.Debug("Starting thrust at " + startTimeOfThrust + " was too early. Trying later.");
                shortTime = startTimeOfThrust;
                startTimeOfThrust = (shortTime + longTime) / 2;
            }
            else if (previousResult == IterationResult.TOO_LATE)
            {
                Logger.Debug("Starting thrust at " + startTimeOfThrust + " was too late. Trying earlier.");
                longTime = startTimeOfThrust;
                startTimeOfThrust = (shortTime + longTime) / 2;
            }
            
            Logger.Debug("Computing approximation " + trajectoryCount + " with " + integrationSteps + " integration steps. Start time of thrust is " + startTimeOfThrust);
            
            //The vector arrays that will store upper- and lower- estimates for the initial position and velocity in each timestep
            initialPosition = new EstimatePair<Vector3d>();
            initialVelocity = new EstimatePair<Vector3d>();

            //Set the orbital status of the vessel immediately before thrust
            SetInitialPositionAndVelocity(t0 + startTimeOfThrust);

            //The variable that contains the mass of the vessel as it decreases over the course of the trajectory simulation
            changingMass = vessel.totalMass;

            //Reset the rising flags
            rising = new EstimatePair<bool>(false, false);

            //Based on the initial velocity, compute an approximate stopping duration and use this to estimate a suitable time step
            double initialSrfSpeed = Vector3d.Magnitude(GetSrfVelocity(initialPosition.lower, initialVelocity.lower));
            double expectedTimeToStop = initialSrfSpeed * changingMass / (thrust - Vector3d.Magnitude(FlightGlobals.getGeeForceAtPosition(initialPosition.lower)));
            timeStep = (double)expectedTimeToStop / integrationSteps;

            //If time step is less than the physics update time of the game, this calculation doesn't make a lot of sense. As such, we recommend accepting the previous best result
            if (timeStep < minimumTimeStep)
            {
                GameObject.FindObjectOfType<MainGUI>().AddWarning(Messages.TimeStepLessThanMinimum);
            }
            else
            {
                GameObject.FindObjectOfType<MainGUI>().RemoveWarning(Messages.TimeStepLessThanMinimum);
            }

            //Reset the burn time
            burnTime = 0;

            //Counts the number of step-backs to avoid oscillation
            stepBackCounter = 0;
        }
        
        private IterationResult DoNextIntegrationSteps()
        {
            //Resulting quantities
            EstimatePair<Vector3d> finalPosition = new EstimatePair<Vector3d>();
            EstimatePair<Vector3d> finalVelocity = new EstimatePair<Vector3d>();
            finalSrfSpeed = new EstimatePair<double>();
            finalSrfAltitude = new EstimatePair<double>();

            //Other integration step quantities
            Vector3d gravAcc;
            Vector3d thrustAcc;
            
            //The result of a single integration step
            IterationResult iterationResult = IterationResult.INCOMPLETE;

            Logger.Debug("Doing next integration steps.");

            //The following loop describes a single time step, testing an over- and under- estimate
            do
            {
                //First generate an estimate using the initial values of the integration step
                finalPosition.lower = (Vector3d) initialPosition.lower + (Vector3d) initialVelocity.lower * timeStep;
                gravAcc = FlightGlobals.getGeeForceAtPosition(initialPosition.lower);
                thrustAcc = thrust / changingMass * -GetSrfVelocity(initialPosition.lower, initialVelocity.lower).normalized;
                finalVelocity.lower = (Vector3d) initialVelocity.lower + (gravAcc + thrustAcc) * timeStep;

                //Then generate an estimate using the final values of the integration step
                gravAcc = FlightGlobals.getGeeForceAtPosition(finalPosition.lower); //Assume this is close enough
                thrustAcc = thrust / (changingMass - fuelFlow * timeStep) * -GetSrfVelocity(finalPosition.lower, finalVelocity.lower).normalized;
                finalVelocity.upper = initialVelocity.upper + (gravAcc + thrustAcc) * timeStep;
                finalPosition.upper = initialPosition.upper + finalVelocity.upper * timeStep;

                //TODO: Complete this properly.
                if (changingMass - fuelFlow * timeStep < 0)
                {
                    Logger.Error("Couldn't complete maneuver without running out of fuel.");
                    return IterationResult.OUT_OF_FUEL;
                }

                //Calculate the things we care about: surface height and surface speed
                for (int j = 0; j < 2; j++)
                { 
                    finalSrfSpeed.SetByIndex(j, Vector3d.Magnitude(GetSrfVelocity(finalPosition.GetByIndex(j), finalVelocity.GetByIndex(j))));
                    finalSrfAltitude.SetByIndex(j, GetSrfAltitude(finalPosition.GetByIndex(j), startTimeOfThrust + burnTime));
                }

                //Test that each estimate is lower/higher as expected
                if (finalSrfSpeed.lower > finalSrfSpeed.upper)
                {
                    finalSrfSpeed.Switch();
                }
                if (finalSrfAltitude.lower > finalSrfAltitude.upper)
                {
                    finalSrfAltitude.Switch();
                }

                Logger.Debug("Final surface speed estimates are [" + finalSrfSpeed.lower.ToString("E2") + ", " + finalSrfSpeed.upper.ToString("E2") +
                    "]. Final surface altitude estimates are [" + finalSrfAltitude.lower.ToString("E2") + ", " + finalSrfAltitude.upper.ToString("E2") + "].");

                //Determine if we have overshot and started rising again (since checking surface speed will not detect this)
                for (int j = 0; j < 2; j++)
                {
                    if (Vector3d.Dot(finalVelocity.GetByIndex(j), body.position - finalPosition.GetByIndex(j)) < 0)
                    {
                        rising.SetByIndex(j, true);
                    }
                }
                
                /*
                This section will evaluate the resulting surface height and surface speed estimate pairs to determine if:
                a) THe calculation if currently incopmlete and should continue (INCOMPLETE)
                b) The timestep we are using is too large, and we have to step back to teh previous iteration and use a smaller timestep to check what the actual result is (STEP_BACK)
                c) The timestep we are using is too large, and we have to repeat the entire trajectory (MORE_ACCURACY)
                d) The thrust was definitely initiated too early (TOO_EARLY)
                e) The thrust was definitely initiated too late (TOO_LATE)
                f) The trajectory has reached the desired surface speed and surface altitude (SUCCESS)

                This is based on transforming the speed and altitude estimates into a 3-by-3 grid in speed-altitude space according to whether each estimate is below, above or between the target values.
                Based on the positions of each point, we can make a qualified decision how to continue by looking up the value in decisionArray. A diagram of this can be made available upon request
                */
                if (finalSrfAltitude.lower < targetAltitude.upper || finalSrfSpeed.lower < targetSpeed.upper || rising.lower || rising.upper) //A broad test to reduce the processing for ordinary iteration cases
                {
                    double[] upperPoint = new double[] { finalSrfAltitude.lower, finalSrfSpeed.upper };
                    double[] lowerPoint = new double[] { finalSrfAltitude.upper, finalSrfSpeed.lower };
                    
                    EstimatePair<double[]> points = new EstimatePair<double[]>(upperPoint, lowerPoint);
                    int[] regions = new int[4]; //This contains the location of the two points in the 3-by-3 space in the form [upper.x, upper.y, lower.x, lower.y]

                    //Map the surface height and speed onto the 3-by-3 grid
                    for (int j = 0; j < 2; j++)
                    {
                        if (points.GetByIndex(j)[0] < targetAltitude.lower) { regions[2 * j] = 0; }
                        else if (points.GetByIndex(j)[0] > targetAltitude.upper) { regions[2 * j] = 2; }
                        else { regions[2 * j] = 1; }

                        if (points.GetByIndex(j)[1] < targetSpeed.lower) { regions[2 * j + 1] = 0; }
                        else if (points.GetByIndex(j)[1] > targetSpeed.upper) { regions[2 * j + 1] = 2; }
                        else { regions[2 * j + 1] = 1; }
                        
                    }
                    
                    //Test for rising vessel
                    if (rising.lower || rising.upper)
                    {
                        Logger.Debug("Detected rising vessel.");

                        //If at least one estimate suggests vessel is rising, set the lower point to region 0
                        regions[3] = 0;

                        if (rising.lower && rising.upper)
                        {
                            //If both estimates suggest vessel is rising, set the upper point to region 0 as well
                            regions[1] = 0;
                        }
                    }
                    
                    Logger.Debug("Points lie in regions [" + regions[0] + ", " + regions[1] + "] and [" + regions[2] + ", " + regions[3] + "].");
                    
                    //Look up the decision in the decision array
                    iterationResult = decisionArray[regions[0], regions[1], regions[2], regions[3]];
                    
                    Logger.Debug("Decision was iteration result: " + iterationResult);

                    if (iterationResult == IterationResult.STEP_BACK)
                    {
                        stepBackCounter++;
                        if (stepBackCounter > 5)
                        {
                            Logger.Debug("Too many step-backs, detected oscillation. Requesting more accuracy.");
                            return IterationResult.MORE_ACCURACY;
                        }
                        else
                        {
                            Logger.Debug("Stepping back.");
                            iterationResult = IterationResult.INCOMPLETE;
                            timeStep = timeStep / 2;
                            
                            continue;
                        }
                    }
                }

                //Progress values:
                Logger.Debug("Progressing values");
                changingMass = changingMass - fuelFlow * timeStep;
                initialPosition = finalPosition;
                initialVelocity = finalVelocity;
                burnTime += timeStep;
                calculationCount++;

            } while (iterationResult == IterationResult.INCOMPLETE && calculationCount < calcsPerUpdate);
            
            return iterationResult;
        }

        /// <summary>
        /// Accepts the best result so far. Triggered by user through GUI button.
        /// </summary>
        public void AcceptOffer()
        {
            Disable();
            GameObject.FindObjectOfType<MainGUI>().SetCalculationSuccessful();
            resultUT = offerUT;
            etaUT = resultUT + offerBurnTime;

            Logger.Info("Offer was accepted. Thrust should begin at UT " + resultUT);

            //Some logging to debug landing inaccuracy
            Vector3d startPos = vessel.GetOrbit().getPositionAtUT(resultUT);
            Vector3d startVel = vessel.GetOrbit().getOrbitalVelocityAtUT(resultUT);
            Logger.Debug("Predicted vessel state at thrust start: srfAltitude = " + GetSrfAltitude(startPos, startTimeOfThrust) + ", srfSpeed = " + Vector3d.Magnitude(GetSrfVelocity(startPos, OrbitToWorld(startVel))));

        }

        /// <summary>
        /// Disables this monobehaviour and unpauses the game
        /// </summary>
        public void Disable()
        {
            GameObject.FindObjectOfType<MainGUI>().UnPause();
            this.enabled = false;
        }

        #region enums        

        /// <summary>
        /// Represents the outcome of a single intergration step
        /// </summary>
        private enum IterationResult
        {
            INCOMPLETE = 0, TOO_EARLY = 1, TOO_LATE = 2, MORE_ACCURACY = 3, STEP_BACK = 4, SUCCESS = 5, REACHED_MAX_PRECISION = 6, OUT_OF_FUEL = 7
        }

        #endregion

        #region Time-to-impact copied and modified from KER

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

        private void SetInitialPositionAndVelocity(double UT)
        {
            initialPosition.lower = vessel.GetOrbit().getPositionAtUT(UT);
            initialVelocity.lower = OrbitToWorld(vessel.GetOrbit().getOrbitalVelocityAtUT(UT));
            initialPosition.upper = initialPosition.lower;
            initialVelocity.upper = initialVelocity.lower;
            initialRetrograde = - GetSrfVelocity(initialPosition.lower, initialVelocity.lower);
        }

        private Vector3d GetWorldPositionTransformedToCurrent(double UT)
        {
            Vector3d worldPos = vessel.GetOrbit().getPositionAtUT(UT);
            Vector3d bodyPosDelta = body.GetOrbit().getPositionAtUT(UT) - body.GetOrbit().getPositionAtUT(Planetarium.GetUniversalTime());

            return worldPos - bodyPosDelta;
        }

        private Vector3d GetWorldVelocityTransformedToCurrent(double UT)
        {
            Vector3d worldVel = OrbitToWorld(vessel.GetOrbit().getOrbitalVelocityAtUT(UT));
            Vector3d bodyVelDelta = OrbitToWorld(body.GetOrbit().getOrbitalVelocityAtUT(UT) - body.GetOrbit().getOrbitalVelocityAtUT(Planetarium.GetUniversalTime()));

            return worldVel - bodyVelDelta;
        }

        private double GetSrfAltitude(Vector3d orbitPos, double deltaT)
        {
            //deltaT is the elapsed time since t0.
            double bodyRot = 360 * (deltaT) / body.rotationPeriod;

            double lat = body.GetLatitude(orbitPos);
            double lon = NormAngle(body.GetLongitude(orbitPos) - bodyRot);

            Logger.Debug("Calculated lat, long is: " + lat + ", " + lon);
            
            var rad = QuaternionD.AngleAxis(lon, Vector3d.down) * QuaternionD.AngleAxis(lat, Vector3d.forward) * Vector3d.right;
            var terrainHeight = body.pqsController.GetSurfaceHeight(rad);

            Logger.Debug("Calculated terrain height is: " + terrainHeight);

            Logger.Debug("Calculated distance from centre of body is: " + Vector3d.Magnitude(orbitPos - body.position));

            return Vector3d.Magnitude(orbitPos - body.position) - terrainHeight;
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

        /// <summary>
        /// Sets the thrust, fuel flow and height of the active vessel
        /// TODO: Calculate useable fuel mass
        /// </summary>
        private void CalculateVesselParameters()
        {
            List<Part> parts = vessel.GetActiveParts();
            
            //TODO: Make dry mass/fuel mass work
            dryMass = vessel.totalMass;
            thrust = 0;
            fuelFlow = 0;

            vesselApproxHeight = 0;

            List<PartResourceDefinition> requiredResources = new List<PartResourceDefinition>();
            List<PartResource> resourcesToBeUsed = new List<PartResource>();

            foreach (Part part in parts)
            {
                List<ModuleEngines> engineModules = part.Modules.GetModules<ModuleEngines>();
                List<ModuleGimbal> gimbalModules = part.Modules.GetModules<ModuleGimbal>();
                
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

                //Check for active gimbals
                foreach (ModuleGimbal module in gimbalModules)
                {
                    if (!(bool)module.Fields.GetValue("gimbalLock"))
                    {
                        GameObject.FindObjectOfType<MainGUI>().AddWarning(Messages.GimbalsDetected);
                        break;
                    }
                }

                //TODO: Check for RCS active (currently broken)
                /*
                if (vessel.ActionGroups.groups.ElementAt((int)Math.Log((int)KSPActionGroup.RCS, 2)))
                {
                    GameObject.FindObjectOfType<MainGUI>().AddWarning(Messages.RCSDetected);
                }
                */

                foreach (PartResourceDefinition resource in requiredResources)
                {
                    if (requiredResources.Contains(resource))
                    {
                        List<PartResource> temp = new List<PartResource>(); 
                        part.GetConnectedResources(resource.id, resource.resourceFlowMode, temp);
                        resourcesToBeUsed.AddUniqueRange(temp);
                    }
                }

                //Here we calculate the vessel height. This is a bit ugly, but essentially we find the closest point on the vessel to the point in space 1km behind the vessel
                if (part.collider != null)
                {
                    Vector3d partFarthestPos = part.collider.ClosestPointOnBounds(vessel.GetWorldPos3D() - 1e3 * vessel.upAxis.normalized);
                    float partFarthestLength = (float)Vector3d.Magnitude(vessel.GetWorldPos3D() - partFarthestPos);
                    vesselApproxHeight = Mathf.Max(partFarthestLength, vesselApproxHeight);
                }
            }

            foreach(PartResource resourceStore in resourcesToBeUsed)
            {
                Logger.Debug("Found " + resourceStore.amount + " units of resource " + resourceStore.resourceName + " in part " + resourceStore.part.name);
                dryMass -= resourceStore.amount;
            }

            Logger.Debug("Total thrust is: " + thrust + ", total fuel flow is: " + fuelFlow + ", dry mass is " + dryMass + ", approx height is " + vesselApproxHeight + ".");
        }

        #endregion

        /// <summary>
        /// Contains the messages used by the calculator for display in GUI
        /// </summary>
        private class Messages
        {
            //Errors
            public static readonly string TooLateToStartThrusting = "Too late to begin thrusting. Vessel cannot be brought to a stop before contact with the surface.";
            public static readonly string MaybeInsufficientFuel = "Vessel may have insufficient fuel to land safely";
            public static readonly string InsufficientFuel = "Vessel has insufficient fuel to land safely.";
            public static readonly string VesselMustBeDescending = "Vessel must be descending in order to calculate descent path.";
            public static readonly string NoActiveEnginesDetected = "No active engines detected on this vessel";
            public static readonly string BodyMustHaveNoAtmosphere = "Can only compute optimal descent for non-atmospheric bodies.";
            public static readonly string MustBeOnSubOrbitalTrajectory = "Vessel must be on suborbital trajectory.";
            public static readonly string VesselIsCurrentlyLanded = "Vessel is currently landed.";

            //Warnings
            public static readonly string TimeStepLessThanMinimum = "Time step is less than minimum. Consider accepting best stop height.";
            public static readonly string GimbalsDetected = "Free gimbals detected. Consider limiting or disabling them if vessel has enough control authority without them.";
            public static readonly string RCSDetected = "Active RCS thrusters detected. Consider limiting or disabling them to preserve the expected mass of the vessel during descent.";
        }
        
    }
}
