using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace ImprovedNonAtmosphericLandings
{
    class OptionsGUI : MonoBehaviour
    {
        private Rect windowPosition = new Rect(20, 500, 300, 0);
        private GUIStyle windowStyle = null;
        private bool drawWindow = false;
        private String kpAsString;
        private String kdAsString;
        private String maxSpeedAsString;
        private String gimbalPercentAsString ="0";
        private InalAutopilot autopilot;
        private InalCalculator calculator;


        public void Start()
        {
            windowStyle = new GUIStyle(HighLogic.Skin.window);

            try
            {
                autopilot = GameObject.FindObjectOfType<InalAutopilot>();
                calculator = GameObject.FindObjectOfType<InalCalculator>();
            }
            catch (Exception e)
            {
                Logger.Error(e.Message);
            }
        }
        
        public void toggle()
        {
            if (drawWindow)
            {
                close();
            }
            else
            {
                open();
            }
        }

        public void open()
        {
            maxSpeedAsString = calculator.GetMaxSpeed().ToString();
            kpAsString = autopilot.GetKp().ToString();
            kdAsString = autopilot.GetKd().ToString();
            drawWindow = true;
        }

        public void close()
        {
            drawWindow = false;
        }

        private void OnGUI()
        {
            GUI.skin = null;
            if (drawWindow)
            {
                //Render GUI
                windowPosition = GUILayout.Window(this.GetInstanceID(), windowPosition, OptionsWindow, Resources.optionsTitle);
            }
        }

        private void OptionsWindow(int windowID)
        {
            GUILayout.BeginVertical();
            GUILayout.Space(5.0f);
            GUILayout.Label("Landing settings: ");
            GUILayout.BeginHorizontal();
            GUILayout.Label("Max. landing speed: ");
            maxSpeedAsString = GUILayout.TextField(maxSpeedAsString);
            GUILayout.EndHorizontal();
            GUILayout.Space(5.0f);
            GUILayout.Label("Autopilot PID: ");
            GUILayout.BeginHorizontal();
            GUILayout.BeginVertical();
            GUILayout.Label("Autopilot Kp: ");
            GUILayout.Label("Autopilot Kd: ");
            GUILayout.EndVertical();
            GUILayout.BeginVertical();
            kpAsString = GUILayout.TextField(kpAsString);
            kdAsString = GUILayout.TextField(kdAsString);
            GUILayout.EndVertical();
            GUILayout.EndHorizontal();

            GUILayout.Label("Vessel Adjustments:");
            GUILayout.BeginHorizontal();
            GUILayout.Label("Gimbal %: ");
            gimbalPercentAsString = GUILayout.TextField(gimbalPercentAsString);
            GUILayout.EndHorizontal();

            GUILayout.Space(5.0f);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Submit"))
            {
                float kp;
                float kd;
                float maxSpeed;
                float gimbalPercent;

                if (!float.TryParse(kpAsString, out kp))
                {
                    Logger.Info("Invalid input in Kp field");
                    kpAsString = autopilot.GetKp().ToString();
                }
                else
                {
                    autopilot.SetKp(kp);
                }

                if (!float.TryParse(kdAsString, out kd))
                {
                    Logger.Info("Invalid input in Kd field");
                    kdAsString = autopilot.GetKd().ToString();
                }
                else
                {
                    autopilot.SetKd(kd);
                }

                if (!float.TryParse(maxSpeedAsString, out maxSpeed))
                {
                    Logger.Info("Invalid input in max speed field");
                    kdAsString = autopilot.GetKd().ToString();
                }
                else
                {
                    calculator.SetMaxSpeed(maxSpeed);
                    if (autopilot.IsActive())
                    {
                        autopilot.SetMaxSpeed(maxSpeed);
                    }
                }

                if (!float.TryParse(gimbalPercentAsString, out gimbalPercent))
                {
                    Logger.Info("Invalid input in gimbal percent field");
                    gimbalPercentAsString = "Invalid input";
                }
                else
                {
                    if (gimbalPercent > 100)
                    {
                        gimbalPercent = 100;
                        gimbalPercentAsString = "100";
                    }
                    else if (gimbalPercent < 0)
                    {
                        gimbalPercent = 0;
                        gimbalPercentAsString = "0";
                    }

                    SetGimbals(gimbalPercent);
                }

                toggle();
            }
            if (GUILayout.Button("Cancel"))
            {
                toggle();
            }
            GUILayout.EndHorizontal();
            GUILayout.EndVertical();
            GUI.DragWindow();
        }

        private void SetGimbals(float percent)
        {
            List<Part> parts = FlightGlobals.ActiveVessel.GetActiveParts();

            foreach (Part part in parts)
            {
                List<ModuleGimbal> gimbalModules = part.Modules.GetModules<ModuleGimbal>();

                //Set each gimbal
                foreach (ModuleGimbal module in gimbalModules)
                {
                    if (percent == 0)
                    {
                        Logger.Debug("Locking gimbals");
                        module.Fields.SetValue("gimbalLock", true);
                    }
                    else
                    {
                        Logger.Debug("Limiting gimbals");
                        module.Fields.SetValue("gimbalLimiter", percent);
                    }
                }
            }
        }
    }
}
