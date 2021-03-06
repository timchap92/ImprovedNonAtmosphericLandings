﻿using System;
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
        private List<string> warnings = new List<string>();
        private string warningString;
        private InalCalculator inalCalculator;
        private InalAutopilot autopilot;
        private GUI.WindowFunction currentWindow;
        
        /// <summary>
        /// Privately settable window property. Ensures that the height of the window is reset when the window changes
        /// </summary>
        private GUI.WindowFunction CurrentWindow
        {
            set
            {
                mainWindowPosition.width = 300;
                mainWindowPosition.height = 0;
                currentWindow = value;
            }
        }

        public void AddWarning(string s)
        {
            if (!warnings.Contains(s))
            {
                warnings.AddUnique(s);
            }
            RenderWarningString();
        }

        public void RemoveWarning(string s)
        {
            warnings.Remove(s);

            RenderWarningString();
        }

        private void RenderWarningString()
        {
            warningString = string.Empty;
            foreach (string warning in warnings)
            {
                warningString += "- " + warning + "\n";
            }
            if (warningString != string.Empty)
            {
                warningString = warningString.Substring(0, warningString.Length - 1);
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
            GUILayout.Label("Thrust start T minus: ");
            GUILayout.Label("Time step:");
            GUILayout.Label("Burn time: ");
            GUILayout.Space(10.0F);
            GUILayout.Label("Best stop height (m): ");
            GUILayout.EndVertical();
            GUILayout.BeginVertical();
            GUILayout.TextArea(inalCalculator.GetTrajectory().ToString());
            GUILayout.TextArea(inalCalculator.GetStartTimeOfThrust().ToString("N2"));
            GUILayout.TextArea(inalCalculator.GetTimeStep().ToString("N4"));
            GUILayout.TextArea(inalCalculator.GetBurnTimeString());
            GUILayout.Space(10.0F);
            GUILayout.TextArea(inalCalculator.GetAltitudeOffer());
            GUILayout.EndVertical();
            GUILayout.EndHorizontal();

            DrawAcceptButton();

            GUILayout.Label("Warnings:");
            GUILayout.TextArea(warningString);

            EndWindow();
        }

        private void CalculationSuccessful(int windowID)
        {
            if(Planetarium.GetUniversalTime() > inalCalculator.GetResultUT())
            {
                CurrentWindow = Idle;
                Idle(windowID);
                return;
            }

            BeginWindow();

            DrawSettingsButton();
            DrawCalculateDescentButton();

            GUILayout.Space(10.0f);
            DrawTMinus();
            DrawDeltaV();
            DrawLandingLat();
            DrawLandingLon();
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
            GUILayout.Label("Total delta-V required: ");
            GUILayout.Label("Landing Latitude: ");
            GUILayout.Label("Landing Longitude: ");
            GUILayout.EndVertical();

            GUILayout.BeginVertical();
            GUILayout.TextArea(autopilot.GetState().ToString());
            GUILayout.TextArea(inalCalculator.GetTMinusString());
            GUILayout.TextArea(inalCalculator.GetETAString());
            GUILayout.TextArea(inalCalculator.GetDeltaV());
            GUILayout.TextArea(inalCalculator.GetLandingLat());
            GUILayout.TextArea(inalCalculator.GetLandingLon());
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

                warnings = new List<string>();
                warningString = String.Empty;

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
            GUILayout.TextArea(inalCalculator.GetTMinusString());
            GUILayout.EndHorizontal();
        }

        private void DrawDeltaV()
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label("Delta-V: ");
            GUILayout.TextArea(inalCalculator.GetDeltaV());
            GUILayout.EndHorizontal();
        }

        private void DrawLandingLat()
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label("Landing Latitude: ");
            GUILayout.TextArea(inalCalculator.GetLandingLat());
            GUILayout.EndHorizontal();
        }

        private void DrawLandingLon()
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label("Landing Longitude: ");
            GUILayout.TextArea(inalCalculator.GetLandingLon());
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
