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

        public void Start()
        {
            windowStyle = new GUIStyle(HighLogic.Skin.window);
        }

        public void toggle()
        {
            drawWindow = !drawWindow;
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
            GUILayout.BeginVertical();
            GUILayout.Label("Max. allowed height: ");
            GUILayout.Label("Min. allowed height: ");
            GUILayout.Label("Max. allowed speed: ");
            GUILayout.Label("Min. allowed speed: ");
            GUILayout.EndVertical();
            GUILayout.BeginVertical();
            GUILayout.TextField("");
            GUILayout.TextField("");
            GUILayout.TextField("");
            GUILayout.TextField("");
            GUILayout.EndVertical();
            GUILayout.EndHorizontal();
            GUILayout.Label("Autpilot PID: ");
            GUILayout.Space(5.0f);
            GUILayout.EndVertical();
            GUI.DragWindow();
        }
    }
}
