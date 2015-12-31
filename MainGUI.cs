using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace ImprovedNonAtmosphericLandings
{
    class MainGUI : MonoBehaviour
    {
        private bool drawWindow = false;
        private Rect mainWindowPosition = new Rect(20, 200, 300, 0);
        private GUIStyle windowStyle = null;
        private OptionsGUI optionsWindow;

        private InalCalculator inalCalculator;
        private InalAutopilot autopilot;

        private void Awake()
        {

        }

        public void Start()
        {
            windowStyle = new GUIStyle(HighLogic.Skin.window);
        }

        public void open()
        {
            drawWindow = true;

            //Should possibly move these to more sensible places
            if (inalCalculator == null)
            {
                try
                {
                    inalCalculator = GameObject.FindObjectOfType<InalCalculator>();
                }
                catch (Exception e)
                {
                    Logger.Error(e.Message);
                }
            }
            if (autopilot == null)
            {
                try
                {
                    autopilot = GameObject.FindObjectOfType<InalAutopilot>();
                }
                catch (Exception e)
                {
                    Logger.Error(e.Message);
                }
            }
            if (optionsWindow == null)
            {
                try
                {
                    optionsWindow = GameObject.FindObjectOfType<OptionsGUI>();
                }
                catch (Exception e)
                {
                    Logger.Error(e.Message);
                }
            }
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
                mainWindowPosition = GUILayout.Window(this.GetInstanceID(), mainWindowPosition, MainWindow, Resources.modName);
            }
        }

        private void MainWindow(int windowID)
        {
            GUILayout.BeginVertical();
            GUILayout.Space(5.0f);

            if (!autopilot.IsActive())
            {
                if (GUILayout.Button("Calculate descent"))
                {
                    inalCalculator.CalculateResult();

                }
                
                if (!inalCalculator.IsCalculating())
                { 
                    if (GUILayout.Button("Settings"))
                    {
                        optionsWindow.toggle();
                    }
                }

                if (inalCalculator.IsComplete())
                {
                    if (GUILayout.Button("Activate Autopilot"))
                    {
                        Logger.Info("Activating autopilot.");
                        optionsWindow.close();
                        autopilot.Activate(inalCalculator);
                    }
                    
                    GUILayout.TextArea(inalCalculator.GetTMinus());
                }
            }
            else
            {
                GUILayout.BeginHorizontal();
                GUILayout.BeginVertical();
                GUILayout.Label("State: ");
                GUILayout.Label("T Minus: ");
                GUILayout.Label("ETA (approx): ");
                GUILayout.EndVertical();

                GUILayout.BeginVertical();
                GUILayout.TextArea(autopilot.GetState().ToString());
                GUILayout.TextArea(inalCalculator.GetTMinus());
                GUILayout.TextArea(inalCalculator.GetETA());
                GUILayout.EndHorizontal();
                GUILayout.EndHorizontal();
                if (GUILayout.Button("Deactivate Autopilot"))
                {
                    autopilot.Deactivate();
                }
                
            }

            GUILayout.Space(5.0f);
            GUILayout.EndVertical();
            GUI.DragWindow();
        }

        
    }
}
