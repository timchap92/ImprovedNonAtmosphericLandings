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
        #region Initialised parameters from calculator and FlightGlobals
        private double startUT;
        private double maxSpeed;
        private double minSpeed;
        private double thrust;
        private Vessel vessel;
        private CelestialBody body;
        Vector3d retrograde;
        #endregion
        
        #region PID parameters
        float kp = 10F;
        float ki = 0.0F;
        float kd = 30F;
        Vector3 integrator = Vector3.zero;
        Vector3d error = Vector3d.zero;
        #endregion

        #region Final descent parameters
        double gravAcc;
        double vesselHeight;
        readonly float safetyMargin = 5;
        private double finalMass;
        #endregion

        #region Other private fields
        private AutopilotState state;
        private bool isActive = false;
        private double previousTime = 0;
        private double updateTime;
        private WarpWatcher warpWatcher;
        #endregion

        #region Public getters & setters

        public AutopilotState GetState()
        {
            return state;
        }

        public float GetKp()
        {
            return kp;
        }

        public float GetKd()
        {
            return kd;
        }

        public void SetKd(float value)
        {
            kd = value;
        }

        public void SetKp(float value)
        {
            kp = value;
        }

        public void SetMaxSpeed(float value)
        {
            maxSpeed = value;
            minSpeed = maxSpeed / 2;
        }

        #endregion

        /// <summary>
        /// Activates the autopilot and initialises it based on the passed calculator
        /// </summary>
        /// <param name="inalCalculator"></param>
        public void Activate(InalCalculator inalCalculator)
        {
            //Initialise autopilot parameters
            startUT = inalCalculator.GetResultUT();
            maxSpeed = inalCalculator.GetMaxSpeed();
            minSpeed = maxSpeed / 2;
            thrust = inalCalculator.GetThrust();
            retrograde = inalCalculator.GetInitialRetrograde();
            vessel = FlightGlobals.ActiveVessel;
            state = AutopilotState.ROTATING;
            body = vessel.mainBody;
            vesselHeight = inalCalculator.GetVesselHeight();
            
            //Register autopilot
            vessel.OnFlyByWire += new FlightInputCallback(fly);
            isActive = true;

            //Set time warp to 1 to allow rotation
            TimeWarp.SetRate(0, true);
        }

        /// <summary>
        /// Enum of autopilot states
        /// </summary>
        public enum AutopilotState
        {
            ROTATING, FREEFALL, MAIN_DESCENT, FINAL_DESCENT_FREEFALL, FINAL_DESCENT_THRUST, LANDED
        }

        /// <summary>
        /// Modifies the passed FlightCtrlState to execute calculated descent
        /// </summary>
        /// <param name="s"></param>
        public void fly(FlightCtrlState s)
        {
            if (state == AutopilotState.ROTATING)
            {
                if (PIDHeading(retrograde, s))
                {
                    warpWatcher = GameObject.FindObjectOfType<WarpWatcher>();
                    warpWatcher.Activate(startUT);
                    state = AutopilotState.FREEFALL;
                }
                else if (Planetarium.GetUniversalTime() > startUT)
                {
                    GameObject.FindObjectOfType<MainGUI>().SetFatalError(Messages.CouldNotReachRetrograde);
                    vessel.OnFlyByWire -= new FlightInputCallback(fly);
                    isActive = false;
                }
            }
            else if (state == AutopilotState.FREEFALL)
            {
                double currentTime = Planetarium.GetUniversalTime();

                PIDHeading(-vessel.srf_velocity, s);
                updateTime = currentTime - previousTime;

                previousTime = currentTime;
                if (warpWatcher.IsTimeWarpStable() && currentTime + 2 * updateTime >= startUT)
                {
                    Logger.Info("Initiating burn at UT " + currentTime);
                    s.mainThrottle = 1.0F;
                    state = AutopilotState.MAIN_DESCENT;

                    //Some logging to debug landing innacuracy
                    Vector3d worldPos = vessel.GetWorldPos3D();
                    Logger.Debug("Actual lat, long is: " + body.GetLatitude(worldPos) + ", " + body.GetLongitude(worldPos));
                    Logger.Debug("Actual terrain height is: " + (vessel.terrainAltitude + body.Radius));
                    Logger.Debug("Actual distance to body centre is: " + Vector3d.Magnitude(worldPos - body.position));
                    Logger.Debug("Actual vessel state at thrust start: srfAltitude = " + (FlightGlobals.ship_altitude - vessel.terrainAltitude) + ", srfSpeed = " + vessel.srfSpeed);
                }
            }
            else if (state == AutopilotState.MAIN_DESCENT)
            {
                PIDHeading(-vessel.srf_velocity, s);
                s.mainThrottle = 1.0F;
                
                if (vessel.srfSpeed < maxSpeed || vessel.verticalSpeed > 0)
                {
                    Logger.Debug("Final descent.");
                    s.mainThrottle = 0.0F;
                    state = AutopilotState.FINAL_DESCENT_FREEFALL;

                    //Need to calculate the difference between the ship altitude and the altitude of the bottom part - from stupid_chris on the KSP forums
                    double altitude = FlightGlobals.ship_altitude - vessel.terrainAltitude;
                    float bottomAlt = (float) altitude;
                    foreach (Part part in vessel.parts)
                    {
                        if (part.collider != null) //Makes sure the part actually has a collider to touch ground
                        {
                            Vector3 bottom = part.collider.ClosestPointOnBounds(vessel.mainBody.position); //Gets the bottom point
                            float partAlt = FlightGlobals.getAltitudeAtPos(bottom) - (float) vessel.terrainAltitude;  //Gets the looped part alt
                            bottomAlt = Mathf.Max(0, Mathf.Min(bottomAlt, partAlt));  //Stores the smallest value in all the parts
                        }
                    }

                    vesselHeight = altitude - bottomAlt;

                    Logger.Debug("Vessel height is " + vesselHeight);

                    //Final descent assumes constant mass and constant gravitational acceleration.
                    finalMass = vessel.totalMass;
                    gravAcc = Vector3d.Magnitude(FlightGlobals.getGeeForceAtPosition(vessel.GetWorldPos3D()));
                }
            }
            else if (state == AutopilotState.FINAL_DESCENT_FREEFALL)
            {
                PIDHeading(-vessel.srf_velocity, s);
                s.mainThrottle = 0.0F;
                double altitude = FlightGlobals.ship_altitude - vessel.terrainAltitude;
                
                if (altitude - vesselHeight - safetyMargin <  (Math.Pow(vessel.srfSpeed, 2) - Math.Pow(maxSpeed, 2)) / (2 * (thrust / finalMass - gravAcc)))
                {
                    Logger.Debug("Final burn. Altitude is " + altitude + ", vessel height is " + vesselHeight + ", surface speed is " + vessel.srfSpeed + ", max allowed landing speed is " + maxSpeed + ", gravAcc is " + gravAcc + ", thrust is " + thrust + ", mass is " + finalMass);
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
                    double retrogradeAngle = Vector3d.Angle(-vessel.srf_velocity, vessel.upAxis);
                    Logger.Debug("Retrograde angle is: " + retrogradeAngle);
                    if (retrogradeAngle < 45)
                    {
                        s.mainThrottle = 1.0F;
                    }
                    else
                    {
                        Logger.Debug("Vessel is not oriented close enough to retrograde!");
                        s.mainThrottle = 0.0F;
                    }
                }
                if (vessel.Landed)
                {
                    s.mainThrottle = 0.0F;
                    state = AutopilotState.LANDED;
                    vessel.OnFlyByWire -= new FlightInputCallback(fly);
                    isActive = false;

                    GameObject.FindObjectOfType<MainGUI>().SetIdle();

                }
            }
        }

        /// <summary>
        /// Removes this FlightInputCallback and disables warp watcher
        /// </summary>
        public void Deactivate()
        {
            if (isActive)
            {
                vessel.OnFlyByWire -= new FlightInputCallback(fly);
                isActive = false;
                previousTime = 0;
            }
            if (warpWatcher != null)
            {
                if (warpWatcher.enabled)
                {
                    warpWatcher.Disable();
                }
            }
        }

        /// <summary>
        /// Returns true if the autopilot is active
        /// </summary>
        /// <returns></returns>
        public bool IsActive()
        {
            return isActive;
        }

        /// <summary>
        /// Modifies the passed FlightCtrlState to aim towards the passed heading vector
        /// </summary>
        /// <param name="heading"></param>
        /// <param name="s"></param>
        /// <returns>True if the heading is sufficiently close to the target heading</returns>
        public bool PIDHeading(Vector3d heading, FlightCtrlState s)
        {
            //Remove SAS
            vessel.ActionGroups.SetGroup(KSPActionGroup.SAS, false);

            Vector3d previousError = error;

            //Move to initial retrograde position
            error = (Vector3d)vessel.transform.InverseTransformDirection((Vector3)heading.normalized);
            
            integrator = 0.9F * integrator + 0.1F * error;
            float yaw_command = Mathf.Clamp(kp * (float)error.x + kd * (float)(error - previousError).x / Time.deltaTime + ki * integrator.x, -1.0F, 1.0F);
            float pitch_command = Mathf.Clamp(-kp * (float)error.z - kd * (float)(error - previousError).z / Time.deltaTime - ki * integrator.z, -1.0F, 1.0F);

            s.yaw += yaw_command;
            s.pitch += pitch_command;
            
            error.y = 0;

            return (Vector3d.SqrMagnitude(error) < 1E-3F && Vector3d.SqrMagnitude(error - previousError) < 1E-7F);
        }

        /// <summary>
        /// The messages used by this class to display in the GUI
        /// </summary>
        class Messages
        {
            public static readonly string CouldNotReachRetrograde = "Could not reach retrograde in the required amount of time. Disabling autopilot.";
        }
    }
}
