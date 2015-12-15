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
        private ApplicationLauncherButton button = null;
        private InalGui gui;

        //Called when FLIGHT scene begins
        protected void Awake()
        {
            //Register for the Ready event
            try
            {
                GameEvents.onGUIApplicationLauncherReady.Add(OnGuiAppLauncherReady);
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
            Logger.Info("Application was removed.");
        }

        //Adds the application to the toolbar and registers behaviours
        void OnGuiAppLauncherReady()
        {
            //Load icon texture
            Texture iconTexture = GameDatabase.Instance.GetTexture(Resources.iconPath, false);
            //Add application
            button = ApplicationLauncher.Instance.AddModApplication(OnButtonClick, OnButtonUnclick, null, null, null, null, ApplicationLauncher.AppScenes.FLIGHT, iconTexture);
            gui = button.gameObject.AddComponent<InalGui>();
            Logger.Info("Added application.");
        }
        
        private void OnButtonClick()
        {
            //Open gui window
            gui.openWindow();
        }

        private void OnButtonUnclick()
        {
            //Close gui window
            gui.closeWindow();
        }

        private void OnGUI()
        {
            //Logger.Info("Gui triggered (Launcher)");
        }

    }
    
}

