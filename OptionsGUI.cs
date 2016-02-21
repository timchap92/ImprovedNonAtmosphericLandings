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
            
            GUILayout.Space(5.0f);
            if (GUILayout.Button("Submit"))
            {
                float kp;
                float kd;
                float maxSpeed;

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
            }
            GUILayout.EndVertical();
            GUI.DragWindow();
        }
    }
}
