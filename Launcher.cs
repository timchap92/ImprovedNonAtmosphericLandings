using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;
using KSP;

namespace ImprovedNonAtmosphericLandings
{
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class Launcher : MonoBehaviour
    {
        private ApplicationLauncherButton button = null;

        //Called when FLIGHT scene begins
        protected void Awake()
        {
            //Register for the Ready event
            try
            {
                GameEvents.onGUIApplicationLauncherReady.Add(OnAppLauncherReady);
                Logger.Info("Added listener.");
            }
            catch (Exception ex)
            {
                Logger.Info(ex.Message);
            }
        }

        protected void Start()
        {
            if (button == null)
            {
                OnAppLauncherReady();
            }
        }

        //Remove listener
        public void OnDestroy()
        {
            GameEvents.onGUIApplicationLauncherReady.Remove(OnAppLauncherReady);
            if (button != null)
            {
                ApplicationLauncher.Instance.RemoveModApplication(button);
            }
            Logger.Info("Application was removed.");
        }

        //Adds the application to the toolbar and registers behaviours
        void OnAppLauncherReady()
        {
            //Load icon texture
            Texture iconTexture = GameDatabase.Instance.GetTexture(Resources.iconPath, false);
            //Add application
            button = ApplicationLauncher.Instance.AddModApplication(OnButtonClick, OnButtonUnclick, null, null, null, null, ApplicationLauncher.AppScenes.FLIGHT, iconTexture);
            Logger.Info("Added application.");
        }

        private Rect windowPosition = new Rect();

        //Launch the app window
        private void OnButtonClick()
        {
            Logger.Info("Launching window.");
            windowPosition = GUILayout.Window(10, windowPosition, OnWindow, "Improved Non-Atmospheric Landings");

        }

        private void OnWindow(int windowId)
        {
            Logger.Info("Adding window content.");
            GUILayout.BeginHorizontal(GUILayout.Width(250f));
            GUILayout.Label("Label");
            GUILayout.EndHorizontal();

            GUI.DragWindow();
        }



        //Remove the app window
        private void OnButtonUnclick()
        {
            Logger.Info("Removing window");
        }
    }
    
}

