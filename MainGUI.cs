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
        private ScreenMessage screenMessage;
        private bool paused = false;

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
                if (!inalCalculator.IsCalculating())
                {
                    if (GUILayout.Button("Calculate descent"))
                    {
                        Pause();
                        screenMessage = ScreenMessages.PostScreenMessage("Pausing for calculation");


                        //Logger.Info("OLD CALCULATOR START");
                        //OldCalculator oldCalculator = new OldCalculator();
                        //oldCalculator.CalculateResult();
                        //Logger.Info("NEW CALCULATOR START");

                        inalCalculator.BeginCalculation();
                        mainWindowPosition.height = 0;
                    }

                    if(GUILayout.Button("Settings"))
                    {
                        optionsWindow.toggle();
                    }
                }
                else
                {
                    optionsWindow.close();
                    if (GUILayout.Button("Abort calculation"))
                    {
                        inalCalculator.Disable();
                        UnPause();
                        mainWindowPosition.height = 0;
                    }
                    GUILayout.BeginHorizontal();
                    GUILayout.BeginVertical();
                    GUILayout.Label("Trajectory number: ");
                    GUILayout.Label("Time step:");
                    GUILayout.Label("Burn time: ");
                    GUILayout.Space(10.0F);
                    GUILayout.Label("Altitude offer (m): ");
                    GUILayout.EndVertical();
                    GUILayout.BeginVertical();
                    GUILayout.TextArea(inalCalculator.GetTrajectory().ToString());
                    GUILayout.TextArea(inalCalculator.GetTimeStep().ToString("N4"));
                    GUILayout.TextArea(inalCalculator.GetBurnTime());
                    GUILayout.Space(10.0F);
                    GUILayout.TextArea(inalCalculator.GetAltitudeOffer());
                    GUILayout.EndVertical();
                    GUILayout.EndHorizontal();
                    if (GUILayout.Button("Accept"))
                    {
                        inalCalculator.AcceptOffer();
                        mainWindowPosition.height = 0;
                    }
                }

                if (inalCalculator.IsComplete())
                {
                    UnPause();

                    if (GUILayout.Button("Activate Autopilot"))
                    {
                        Logger.Info("Activating autopilot.");
                        optionsWindow.close();
                        
                        autopilot.Activate(inalCalculator);
                        mainWindowPosition.height = 0;
                    }

                    GUILayout.BeginHorizontal();
                    GUILayout.Label("T Minus: ");
                    GUILayout.TextArea(inalCalculator.GetTMinus());
                    GUILayout.EndHorizontal();
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
                    mainWindowPosition.height = 0;
                }
                
            }

            GUILayout.Space(5.0f);
            GUILayout.EndVertical();
            GUI.DragWindow();
        }

        private void Pause()
        {
            if (paused == false)
            {
                try
                {
                    FlightDriver.SetPause(true);
                    paused = true;
                }
                catch (Exception ex)
                {
                    Logger.Info(ex.Message);
                }
            }
        }

        private void UnPause()
        {
            if (paused == true)
            {
                try
                {
                    FlightDriver.SetPause(false);
                    paused = false;
                }
                catch (Exception ex)
                {
                    Logger.Info(ex.Message);
                }
            }
        }
        
    }
}
