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
    public class InalLauncher : MonoBehaviour
    {
        private InalAutopilot autopilot;
        private InalCalculator calculator;
        private OptionsGUI optionsWindow;
        private ApplicationLauncherButton button = null;
        private MainGUI gui;

        //Called when FLIGHT scene begins
        protected void Awake()
        {
            //Register for the Ready event
            try
            {
                GameEvents.onGUIApplicationLauncherReady.Add(OnGuiAppLauncherReady);
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
                OnGuiAppLauncherReady();
            }

        }

        //Remove listener
        public void OnDestroy()
        {
            GameEvents.onGUIApplicationLauncherReady.Remove(OnGuiAppLauncherReady);
            if (button != null)
            {
                ApplicationLauncher.Instance.RemoveModApplication(button);
            }
        }

        //Adds the application to the toolbar and registers behaviours
        void OnGuiAppLauncherReady()
        {
            //Load icon texture
            Texture iconTexture = GameDatabase.Instance.GetTexture(Resources.iconPath, false);
            //Add application
            button = ApplicationLauncher.Instance.AddModApplication(OnButtonClick, OnButtonUnclick, null, null, null, null, ApplicationLauncher.AppScenes.FLIGHT | ApplicationLauncher.AppScenes.MAPVIEW, iconTexture);
            gui = button.gameObject.AddComponent<MainGUI>();
        }
        
        private void OnButtonClick()
        {
            if (calculator == null)
            {
                calculator = gameObject.AddComponent<InalCalculator>();
            }
            if (autopilot == null)
            {
                autopilot = gameObject.AddComponent<InalAutopilot>();
            }
            if (optionsWindow == null)
            {
                optionsWindow = gameObject.AddComponent<OptionsGUI>();
            }

            //Open gui window
            gui.open();
        }

        private void OnButtonUnclick()
        {
            //Close gui window
            gui.close();
        }
    }
    
}

