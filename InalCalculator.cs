using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Threading;
using KSP;
using UnityEngine;

namespace ImprovedNonAtmosphericLandings
{
    class InalCalculator
    {
        //Thread stuff
        private Thread calcThread;
        private volatile Boolean shouldStop;
        private volatile float result;
        private String stringResult = string.Empty;
        private volatile String status = string.Empty;
        private readonly object statusLock = new System.Object();

        //Calculation stuff
        private CelestialBody body;
        private Vessel vessel;
        private double thrust;
        private double fuelFlow;
        private double t0;
        private Vector3d initialSrfVelocity;
        private Vector3d initialPosition;
        private double initialMass;

        //Calculation parameters
        private int integrationSteps = 10;
        private float targetStopAltitude = 100;
        private float altitudeError = 100;
        private float targetSpeed = 5;
        private float speedError = 5;

        public String GetStatus()
        {
            return status;
        }

        public String GetStringResult()
        {
            return stringResult;
        }

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

            Logger.Info("Beginning calculation.");

            vessel = FlightGlobals.ActiveVessel;
            body = vessel.mainBody;

            thrust = CalculateThrust();
            fuelFlow = CalculateFuelFlow();

            t0 = Planetarium.GetUniversalTime();
            
            //The time at which we start retrograde thrusting
            double startTimeOfThrust = 0;

            //The calculated terrain altitude and time of stop
            double radarAltitudeOfStop;
            double expectedTimeToStop;

            //What happens if we start burning immediately?
            initialPosition = vessel.GetWorldPos3D();
            initialSrfVelocity = vessel.srf_velocity;
            initialMass = vessel.totalMass;

            //Based on the initial velocity, compute an approximate stopping duration
            expectedTimeToStop = Vector3d.Magnitude(initialSrfVelocity) * initialMass / (thrust - Vector3d.Magnitude(FlightGlobals.getGeeForceAtPosition(initialPosition)));
            double timeStep = (double) expectedTimeToStop / integrationSteps;

            StopObject immediateBurn = GetTimeAndTerrainAltitudeOfStop(startTimeOfThrust, timeStep);
            
            //If immediate thrust does not bring the ship to a stop in time, we are in trouble.
            if (immediateBurn.altitude < 0)
            {
                Logger.Info("Cannot bring ship to a stop in time.");
                return;
            }

            //These are the initial upper and lower bounds for initiating thrust in the loop below
            double shortTime = 0;
            double longTime = ComputeImpactTime();

            //Counter
            int i = 0;

            //Main loop that computes 
            do
            {
                i++;

                startTimeOfThrust = (shortTime + longTime) / 2;

                //Calculate the initial position, velocity and mass (i.e. just before the burn commences)
                initialPosition = vessel.GetOrbit().getPositionAtUT(t0 + startTimeOfThrust);
                initialSrfVelocity = GetSurfaceVelocity(initialPosition, vessel.orbit.getOrbitalVelocityAtUT(t0 + startTimeOfThrust));
                initialMass = vessel.totalMass;

                //Based on the initial velocity, compute an approximate stopping duration
                expectedTimeToStop = Vector3d.Magnitude(initialSrfVelocity) * initialMass / (thrust - Vector3d.Magnitude(FlightGlobals.getGeeForceAtPosition(initialPosition)));

                timeStep = (double) expectedTimeToStop / integrationSteps;

                Logger.Info("Computing approximation " + i + ". Thrust begins " + startTimeOfThrust.ToString("E2") + " from now. Timestep is " + timeStep);

                //Calculate the altitude and time of this iteration
                StopObject thisStop = GetTimeAndTerrainAltitudeOfStop(startTimeOfThrust, timeStep);
                radarAltitudeOfStop = thisStop.altitude;
                expectedTimeToStop = thisStop.time;

                Logger.Info("Thrust at time [" + startTimeOfThrust.ToString("E2") + "s] results in rest terrain altitude of [" + radarAltitudeOfStop.ToString("N1") + "m]");

                if (radarAltitudeOfStop < targetStopAltitude)
                {
                    //Initiated thrust too late, therefore try earlier
                    longTime = startTimeOfThrust;
                    startTimeOfThrust = (startTimeOfThrust + shortTime) / 2;
                }
                else
                {
                    //Initiated thrust too early, therefore try later
                    shortTime = startTimeOfThrust;
                    startTimeOfThrust = (startTimeOfThrust + longTime) / 2;
                }

            } while (!isWithinBounds(radarAltitudeOfStop));

            Logger.Info("Success!");
        }
    
        private StopObject GetTimeAndTerrainAltitudeOfStop(double startTimeOfThrust, double timeStep)
        {
            Logger.Info("At start of burn, altitude is " + GetRadarAltitude(initialPosition, startTimeOfThrust).ToString("N1") + "m and speed is " + Vector3d.Magnitude(initialSrfVelocity).ToString("N1"));

            //The initial velocity vector (not to be confused with the initial surface velocity vector
            Vector3d initialVelocity = vessel.orbit.getOrbitalVelocityAtUT(t0 + startTimeOfThrust);

            //Other integration step variables
            Vector3d initialThrustAcc;
            Vector3d initialGravAcc;

            //Final motion values calculated from initial values
            Vector3d finalPositionL;
            Vector3d finalVelocityU;

            //Final vessel properties
            Vector3d finalGravAcc;
            double finalMass;
            Vector3d finalThrustAcc;

            //Final motion values calculated from final values
            Vector3d finalSrfVelocity;
            Vector3d finalPositionU;
            Vector3d finalVelocityL;

            //Average values
            Vector3d finalPositionMean;
            Vector3d finalVelocityMean;

            //The final speed squared
            double sqrFinalSpeed;

            //The time from start of burn
            double burnTime = 0;

            //The flag to continue the approximation
            Boolean cont = true;

            //The final terrain altitude
            double finalTerrainAltitude;

            //The following loop describes a single time step, averaging an over- and under- estimate
            do
            {
                //Calculate the acceleration on the vessel based on other parameters
                initialGravAcc = FlightGlobals.getGeeForceAtPosition(initialPosition);
                initialThrustAcc = thrust * Vector3d.Normalize(-initialSrfVelocity) / initialMass;

                //Calculate motion deltas based on initial values
                finalPositionL = initialPosition + initialVelocity * timeStep;
                finalVelocityU = initialVelocity + (initialGravAcc + initialThrustAcc) * timeStep;

                //Calculate new forces and masses
                //TODO: Clamp this
                finalMass = initialMass - fuelFlow * timeStep;
                if (finalMass < 0)
                {
                    Logger.Warn("Not enough fuel to reach a stop.");
                }

                finalSrfVelocity = GetSurfaceVelocity(finalPositionL, finalVelocityU);
                finalGravAcc = FlightGlobals.getGeeForceAtPosition(finalPositionL);
                finalThrustAcc = thrust * Vector3d.Normalize(-finalSrfVelocity) / finalMass;

                //Calculate motion deltas based on final values
                finalPositionU = initialPosition + finalVelocityU * timeStep;
                finalVelocityL = initialVelocity + (finalGravAcc + finalThrustAcc) * timeStep;

                finalPositionMean = (finalPositionL + finalPositionU) / 2;
                finalVelocityMean = (finalVelocityL + finalVelocityU) / 2;

                Logger.Info("Final position difference: " + Vector3d.Magnitude(finalPositionU - finalPositionL).ToString("E2") + ". Final velocity difference: " + Vector3d.Magnitude(finalVelocityU - finalVelocityL).ToString("E2"));

                finalTerrainAltitude = GetRadarAltitude(finalPositionMean, t0 + startTimeOfThrust + burnTime);
                var finalSrfSpeed = Vector3d.Magnitude(GetSurfaceVelocity(finalPositionMean, finalVelocityMean));

                burnTime += timeStep;

                Logger.Info("At " + burnTime.ToString("E2") + "s after burn start, altitude is " + finalTerrainAltitude.ToString("N1") + "m and speed is " + finalSrfSpeed.ToString("N1"));

                if (isAscending(finalPositionMean, finalVelocityMean))
                {
                    //Vessel started ascending before it was measured with an acceptable downward velocity.
                    Logger.Info("Detected ascent. Trying with a smaller time increment.");

                    burnTime -= timeStep;

                    timeStep = timeStep / 2;
                }
                else
                {
                    initialPosition = finalPositionMean;
                    initialSrfVelocity = finalVelocityMean;
                    initialMass = finalMass;

                    //Test if the vessel has slow to an acceptable speed
                    if (finalSrfSpeed < targetSpeed + speedError)
                    {
                        Logger.Info("Calculation halted.");
                        cont = false;
                    }
                    
                }
                

            } while (cont);


            return new StopObject(burnTime, finalTerrainAltitude);
        }

        public Boolean isAscending(Vector3d position, Vector3d velocity)
        {
            return Vector3d.Dot(velocity, (body.position - position)) < 0;
        }

        private double GetRadarAltitude(Vector3d worldPos, double deltaT)
        {
            double bodyRot = 360 * deltaT / body.rotationPeriod;

            double lat = body.GetLatitude(worldPos);
            double lon = NormAngle(body.GetLongitude(worldPos) - bodyRot);
            
            var rad = QuaternionD.AngleAxis(lon, Vector3d.down) * QuaternionD.AngleAxis(lat, Vector3d.forward) * Vector3d.right;
            var terrainHeight = body.pqsController.GetSurfaceHeight(rad);

            return Vector3d.Magnitude(worldPos - body.position) - terrainHeight;
        }


        public class StopObject
        {
            public StopObject(double time, double altitude)
            {
                this.time = time;
                this.altitude = altitude;
            }
            public double altitude { get; set; }
            public double time { get; set; }
        }


        private Boolean isWithinBounds(double altitudeOfStop)
        {
            return ((altitudeOfStop < targetStopAltitude + altitudeError) && (altitudeOfStop > targetStopAltitude - altitudeError));
        }

        private Vector3d GetSurfaceVelocity(Vector3d worldPos, Vector3d velocity)
        {
            return velocity - body.getRFrmVel(worldPos);
        }

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

        #region Dummy thrust and fuel flow values

        private double CalculateThrust()
        {
            return 250d;
        }

        private double CalculateFuelFlow()
        {
            return 14.56737 * 5d / 1000;
        }

        #endregion


    }
}
