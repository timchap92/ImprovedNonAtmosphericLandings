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
        private double previousTime;
        private double updateTime;
        private CelestialBody body;
        private WarpWatcher warpWatcher;
        
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
            finalMass = inalCalculator.GetFinalMass();
            vessel = FlightGlobals.ActiveVessel;

            //Register autopilot
            vessel.OnFlyByWire += new FlightInputCallback(fly);
            isActive = true;

            state = AutopilotState.FREEFALL;
            previousTime = Planetarium.GetUniversalTime();
            body = vessel.mainBody;

            warpWatcher = GameObject.FindObjectOfType<WarpWatcher>();
            warpWatcher.Activate(startUT);
        }

        public enum AutopilotState
        {
            FREEFALL, MAIN_DESCENT, FINAL_DESCENT_FREEFALL, FINAL_DESCENT_THRUST, LANDED
        }


        public void fly(FlightCtrlState s)
        {
            if (state == AutopilotState.FREEFALL)
            {
                double currentTime = Planetarium.GetUniversalTime();
                updateTime = currentTime - previousTime;
                previousTime = currentTime;
                if (currentTime + updateTime >= startUT)
                {
                    Logger.Info("Initiating burn.");
                    s.mainThrottle = 1.0F;
                    state = AutopilotState.MAIN_DESCENT;
                }
            }
            else if (state == AutopilotState.MAIN_DESCENT)
            {
                s.mainThrottle = 1.0F;
                if (vessel.srfSpeed < maxSpeed)
                {
                    Logger.Info("Final descent.");
                    s.mainThrottle = 0.0F;
                    state = AutopilotState.FINAL_DESCENT_FREEFALL;
                }
            }
            else if (state == AutopilotState.FINAL_DESCENT_FREEFALL)
            {
                s.mainThrottle = 0.0F;
                if (vessel.pqsAltitude < (Math.Pow(maxSpeed, 2) - Math.Pow(vessel.srfSpeed, 2) / (2 * (thrust / finalMass - vessel.geeForce))))
                {
                    s.mainThrottle = 1.0F;
                    state = AutopilotState.FINAL_DESCENT_THRUST;
                }
            }
            else if (state == AutopilotState.FINAL_DESCENT_THRUST)
            {
                if (vessel.srfSpeed < maxSpeed)
                {
                    s.mainThrottle = (float) (finalMass * vessel.geeForce / thrust);

                    if (vessel.srfSpeed < maxSpeed / 2)
                    {
                        s.mainThrottle = 0.0F;
                    }
                }
                else
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
            }
        }

    }
}
