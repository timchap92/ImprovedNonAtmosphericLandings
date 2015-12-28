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
        private AutopilotState state = AutopilotState.FREEFALL;
        private Vessel vessel;

        public void Activate(InalCalculator inalCalculator)
        {
            //Initialise autopilot parameters
            startUT = inalCalculator.GetResultUT();
            maxSpeed = inalCalculator.GetMaxSpeed();
            minSpeed = inalCalculator.GetMinSpeed();
            vessel = FlightGlobals.ActiveVessel;

            //Register autopilot
            vessel.OnFlyByWire += new FlightInputCallback(fly);
        }

        private enum AutopilotState
        {
            FREEFALL, MAIN_DESCENT, FINAL_DESCENT
        }

        public void fly(FlightCtrlState s)
        {
            if (state == AutopilotState.FREEFALL)
            {
                if (Planetarium.GetUniversalTime() >= startUT)
                {
                    Logger.Info("Initiating burn.");
                    state = AutopilotState.MAIN_DESCENT;
                }
            }
            if (state == AutopilotState.MAIN_DESCENT)
            {
                s.mainThrottle = 1.0F;
                if (vessel.srfSpeed < maxSpeed)
                {
                    Logger.Info("Final descent.");
                    state = AutopilotState.FINAL_DESCENT;
                }
            }
            if (state == AutopilotState.FINAL_DESCENT)
            {
                if (vessel.srfSpeed > maxSpeed)
                {
                    s.mainThrottle = 1.0F;
                }
                else if (vessel.srfSpeed < maxSpeed / 2)
                {
                    s.mainThrottle = 0.0F;
                }
                if (vessel.Landed)
                {
                    Logger.Info("Vessel landed.");
                    vessel.OnFlyByWire -= new FlightInputCallback(fly);
                }
            }
        }

    }
}
