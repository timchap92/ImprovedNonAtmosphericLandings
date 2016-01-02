using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;
using System.Reflection;

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
        private string failReason;

        private InalCalculator inalCalculator;
        private InalAutopilot autopilot;

        private GUI.WindowFunction currentWindow;

        /// <summary>
        /// Privately settable window property. Ensures that the height of the window is reset when the window changes
        /// </summary>
        public GUI.WindowFunction CurrentWindow
        {
            set
            {
                mainWindowPosition.height = 0;
                currentWindow = value;
            }
        }

        public void SetCalculationSuccessful()
        {
            CurrentWindow = CalculationSuccessful;
        }

        public void SetFatalError(string s)
        {
            failReason = s;
            CurrentWindow = FatalError;
        }

        public void SetIdle()
        {
            CurrentWindow = Idle;
        }

        public void Start()
        {
            windowStyle = new GUIStyle(HighLogic.Skin.window);

            currentWindow = Idle;
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
                
                mainWindowPosition = GUILayout.Window(this.GetInstanceID(), mainWindowPosition, GetWindow(), Resources.modName);
            }
        }

        private GUI.WindowFunction GetWindow()
        {
            return currentWindow;
        }

        private void Idle(int windowID)
        {
            BeginWindow();

            DrawSettingsButton();
            DrawCalculateDescentButton();

            EndWindow();
        }

        private void Calculating(int windowID)
        {
            BeginWindow();

            DrawAbortCalculationButton();

            GUILayout.BeginHorizontal();
            GUILayout.BeginVertical();
            GUILayout.Label("Trajectory number: ");
            GUILayout.Label("Time step:");
            GUILayout.Label("Burn time: ");
            GUILayout.Space(10.0F);
            GUILayout.Label("Final stop height (m): ");
            GUILayout.EndVertical();
            GUILayout.BeginVertical();
            GUILayout.TextArea(inalCalculator.GetTrajectory().ToString());
            GUILayout.TextArea(inalCalculator.GetTimeStep().ToString("N4"));
            GUILayout.TextArea(inalCalculator.GetBurnTime());
            GUILayout.Space(10.0F);
            GUILayout.TextArea(inalCalculator.GetAltitudeOffer());
            GUILayout.EndVertical();
            GUILayout.EndHorizontal();

            DrawAcceptButton();

            EndWindow();
        }

        private void CalculationSuccessful(int windowID)
        {
            if(Planetarium.GetUniversalTime() > inalCalculator.GetResultUT())
            {
                CurrentWindow = Idle;
                return;
            }

            BeginWindow();

            DrawSettingsButton();
            DrawCalculateDescentButton();

            GUILayout.Space(10.0f);
            DrawTMinus();
            DrawActivateAutopilotButton();

            EndWindow();
        }

        private void FatalError(int windowID)
        {
            BeginWindow();

            GUILayout.Label("Error:");
            GUILayout.TextArea(failReason);

            if (GUILayout.Button("OK"))
            {
                CurrentWindow = Idle;
            }

            EndWindow();
        }

        private void Autopilot(int windowID)
        {
            BeginWindow();

            DrawSettingsButton();

            GUILayout.Space(10.0f);

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
                CurrentWindow = CalculationSuccessful;
            }

            EndWindow();
        }

        #region Window, button and field helpers


        private void BeginWindow()
        {
            GUILayout.BeginVertical();
            GUILayout.Space(5.0f);
        }

        private void EndWindow()
        {
            GUILayout.EndVertical();
            GUILayout.Space(5.0f);
            GUI.DragWindow();
        }

        private void DrawSettingsButton()
        {
            if (GUILayout.Button("Settings"))
            {
                optionsWindow.toggle();
            }
        }

        private void DrawCalculateDescentButton()
        {
            if (GUILayout.Button("Calculate descent"))
            {
                Pause();
                screenMessage = ScreenMessages.PostScreenMessage("Pausing for calculation");

                CurrentWindow = Calculating;
                inalCalculator.BeginCalculation();

                optionsWindow.close();
            }
        }

        private void DrawActivateAutopilotButton()
        {
            if (GUILayout.Button("Activate Autopilot"))
            {
                Logger.Info("Activating autopilot.");

                autopilot.Activate(inalCalculator);

                CurrentWindow = Autopilot;
            }
        }

        private void DrawAbortCalculationButton()
        {
            if (GUILayout.Button("Abort calculation"))
            {
                inalCalculator.Disable();
                CurrentWindow = Idle;
            }
        }

        private void DrawAcceptButton()
        {
            if (GUILayout.Button("Accept"))
            {
                inalCalculator.AcceptOffer();
            }
        }

        private void DrawTMinus()
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label("T Minus: ");
            GUILayout.TextArea(inalCalculator.GetTMinus());
            GUILayout.EndHorizontal();
        }

        #endregion

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

        public void UnPause()
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
