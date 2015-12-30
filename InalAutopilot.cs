using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;
using KSP;

namespace ImprovedNonAtmosphericLandings
{
    class InalAutopilot : MonoBehaviour
    {
        private double startUT;
        private double maxSpeed;
        private double minSpeed;
        private double thrust;
        private double finalMass;
        private AutopilotState state;
        private Vessel vessel;
        public bool isActive = false;
        private double previousTime = 0;
        private double updateTime;
        private CelestialBody body;
        private WarpWatcher warpWatcher;
        float kp = 1F;
        float ki = 1F;
        float kd = 1.5F;
        Vector3 integrator = Vector3.zero;
        Vector3d error = Vector3d.zero;
        Vector3d retrograde;
        Vector3d upwards;
        double gravAcc;
        int stableCount;

        float vesselHeight = 5;
        float safetyMargin = 2;
        
        public AutopilotState GetState()
        {
            return state;
        }

        public void Activate(InalCalculator inalCalculator)
        {
            //Initialise autopilot parameters
            startUT = inalCalculator.GetResultUT();
            maxSpeed = inalCalculator.GetMaxSpeed();
            minSpeed = inalCalculator.GetMinSpeed();
            thrust = inalCalculator.GetThrust();
            retrograde = inalCalculator.GetInitialRetrograde();
            vessel = FlightGlobals.ActiveVessel;

            //Register autopilot
            vessel.OnFlyByWire += new FlightInputCallback(fly);
            isActive = true;

            state = AutopilotState.ROTATING;
            body = vessel.mainBody;
        }

        public enum AutopilotState
        {
            ROTATING, FREEFALL, MAIN_DESCENT, FINAL_DESCENT_FREEFALL, FINAL_DESCENT_THRUST, LANDED
        }


        public void fly(FlightCtrlState s)
        {
            if (state == AutopilotState.ROTATING)
            {
                if (PIDHeading(retrograde, s))
                {
                    warpWatcher = GameObject.FindObjectOfType<WarpWatcher>();
                    warpWatcher.Activate(startUT);
                    stableCount = 0;
                    state = AutopilotState.FREEFALL;
                }
            }
            else if (state == AutopilotState.FREEFALL)
            {
                double currentTime = Planetarium.GetUniversalTime();
                if (stableCount < 3) //Ensures that 3 frames pass at 1x warp before we check the update time
                {
                    //Set update time to zero, otherwise engines will fire up too early due to large expected updateTime.
                    stableCount++;
                    updateTime = 0;
                }
                else
                {
                    PIDHeading(retrograde, s);
                    updateTime = currentTime - previousTime;
                }
                previousTime = currentTime;
                if (currentTime + 2 * updateTime >= startUT)
                {
                    Logger.Info("Initiating burn at UT " + currentTime);
                    s.mainThrottle = 1.0F;
                    state = AutopilotState.MAIN_DESCENT;
                }
            }
            else if (state == AutopilotState.MAIN_DESCENT)
            {
                PIDHeading(-vessel.srf_velocity, s);
                s.mainThrottle = 1.0F;

                if (vessel.srfSpeed < maxSpeed)
                {
                    Logger.Info("Final descent.");
                    s.mainThrottle = 0.0F;
                    state = AutopilotState.FINAL_DESCENT_FREEFALL;

                    //Final descent assumes constant mass, constant upwards vector and constant gravitational acceleration.
                    finalMass = vessel.totalMass;
                    upwards = vessel.GetWorldPos3D() - vessel.mainBody.position;
                    gravAcc = Vector3d.Magnitude(FlightGlobals.getGeeForceAtPosition(vessel.GetWorldPos3D()));
                }
            }
            else if (state == AutopilotState.FINAL_DESCENT_FREEFALL)
            {
                PIDHeading(upwards, s);
                s.mainThrottle = 0.0F;
                double altitude = FlightGlobals.ship_altitude - FlightGlobals.ActiveVessel.terrainAltitude;
                
                if (altitude - vesselHeight - safetyMargin <  (Math.Pow(vessel.srfSpeed, 2) - Math.Pow(maxSpeed, 2)) / (2 * (thrust / finalMass - gravAcc)))
                {
                    Logger.Info("Final burn. Altitude is " + altitude + ", surface speed is " + vessel.srfSpeed + ", max allowed landing speed is " + maxSpeed + ", gravAcc is " + gravAcc + ", thrust is " + thrust + ", mass is " + finalMass);
                    s.mainThrottle = 1.0F;
                    state = AutopilotState.FINAL_DESCENT_THRUST;
                }
            }
            else if (state == AutopilotState.FINAL_DESCENT_THRUST)
            {
                PIDHeading(-vessel.srf_velocity, s);
                if (vessel.srfSpeed < 0.75 * maxSpeed)
                {
                    s.mainThrottle = (float) (gravAcc * vessel.totalMass / thrust);

                    if (vessel.srfSpeed < maxSpeed / 2)
                    {
                        s.mainThrottle = 0.0F;
                    }
                }
                else if (vessel.srfSpeed > maxSpeed)
                {
                    s.mainThrottle = 1.0F;
                }
                if (vessel.Landed)
                {
                    s.mainThrottle = 0.0F;
                    state = AutopilotState.LANDED;
                    vessel.OnFlyByWire -= new FlightInputCallback(fly);
                    isActive = false;
                }
            }
        }

        public void Deactivate()
        {
            if (isActive)
            {
                vessel.OnFlyByWire -= new FlightInputCallback(fly);
                isActive = false;
                previousTime = 0;
            }
        }

        public bool PIDHeading(Vector3d heading, FlightCtrlState s)
        {
            Vector3d previousError = error;
            //Move to initial retrograde position
            error = (Vector3d)vessel.transform.InverseTransformDirection((Vector3)heading.normalized);
            error.y = 0;
            integrator = 0.9F * integrator + 0.1F * error;
            float yaw_command = Mathf.Clamp(kp * (float)error.x + kd * (float)(error - previousError).x / Time.deltaTime + ki * integrator.x, -1.0F, 1.0F);
            float pitch_command = Mathf.Clamp(-kp * (float)error.z - kd * (float)(error - previousError).z / Time.deltaTime - ki * integrator.z, -1.0F, 1.0F);

            s.yaw = yaw_command;
            s.pitch = pitch_command;

            Logger.Info("Sqr magnitude error is : " + Vector3d.SqrMagnitude(error) + ". SqrMagnitude difference is " + Vector3d.SqrMagnitude(error - previousError) / Time.deltaTime);

            return (Vector3d.SqrMagnitude(error) < 1E-3F && Vector3d.SqrMagnitude(error - previousError) < 1E-6F);
        }

    }
}
