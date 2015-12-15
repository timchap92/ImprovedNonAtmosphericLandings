﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace ImprovedNonAtmosphericLandings
{

    class InalGui : MonoBehaviour
    {
        private Boolean drawWindow = false;
        private Rect windowPosition = new Rect(20, 200, 300, 0);
        private GUIStyle windowStyle = null;
        private InalCalculator inalCalculator;

        private void Awake()
        {
            Logger.Info("GUI was created.");
        }

        public void Start()
        {
            windowStyle = new GUIStyle(HighLogic.Skin.window);
        }

        public void openWindow()
        {
            Logger.Info("Opening window");
            drawWindow = true;
            inalCalculator = new InalCalculator();
        }

        public void closeWindow()
        {
            Logger.Info("Closing window");
            drawWindow = false;
            inalCalculator.Stop();
            inalCalculator = null;
            Logger.Info("Calculator set to null"); //Maybe this will be an issue?
        }

        private void OnGUI()
        {
            //Render GUI
            if (drawWindow)
            {
                GUI.skin = null;
                windowPosition = GUILayout.Window(this.GetInstanceID(), windowPosition, DoWindow, Resources.modName);
            }
        }

        private void DoWindow(int windowID)
        {
            GUILayout.BeginVertical();
            GUILayout.Space(5.0f);
            if (GUILayout.Button("Calculate descent"))
            {
                inalCalculator.BackgroundCalculateResult();

            }
            GUILayout.Space(5.0f);
            GUILayout.TextArea(inalCalculator.GetStatus());
            GUILayout.TextArea(inalCalculator.GetStringResult());

            if (GUILayout.Button("Cancel calculation"))
            {
                inalCalculator.Stop();
            }

            GUILayout.EndVertical();
            GUI.DragWindow();
        }
    }
}
