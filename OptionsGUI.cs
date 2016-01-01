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
        private InalAutopilot autopilot;

        public void Start()
        {
            windowStyle = new GUIStyle(HighLogic.Skin.window);

            try
            {
                autopilot = GameObject.FindObjectOfType<InalAutopilot>();
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

                if (!float.TryParse(kpAsString, out kp))
                {
                    Logger.Info("Invalid input");
                    kpAsString = autopilot.GetKp().ToString();
                }
                else
                {
                    autopilot.SetKp(kp);
                }
                if (!float.TryParse(kdAsString, out kd))
                {
                    Logger.Info("Invalid input");
                    kdAsString = autopilot.GetKd().ToString();
                }
                else
                {
                    autopilot.SetKd(kd);
                }
            }
            GUILayout.EndVertical();
            GUI.DragWindow();
        }
    }
}
