using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using KSP;
using UnityEngine;

namespace ImprovedNonAtmosphericLandings
{
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    class WarpWatcher : MonoBehaviour
    {
        private double previousTime;
        private double updateTime;
        private double stopUT;

        private void Awake()
        {
            this.enabled = false;
            Logger.Info("WarpWatcher disabled.");
        }

        public void Activate(double stopUT)
        {
            Logger.Info("WarpWatcher activated");
            StartWarp();
            this.stopUT = stopUT;
            this.previousTime = Planetarium.GetUniversalTime();
            this.enabled = true;
        }

        public void Update()
        {
            Logger.Info("Update triggered!");
            double currentTime = Planetarium.GetUniversalTime();
            updateTime = currentTime - previousTime;
            previousTime = currentTime;

            Logger.Info("Current update time is: " + updateTime);
            if (currentTime + updateTime > stopUT - 60)
            {
                Logger.Info("Stop time for is less than 6s away from next frame: " + (stopUT - currentTime));
                if (currentTime + updateTime > stopUT - 10)
                {
                    Logger.Info("Stop time is less than 10s away from next frame.");
                    if (TimeWarp.CurrentRate == 1)
                    {
                        this.enabled = false;
                    }
                    else
                    {
                        TimeWarp.SetRate(0, true);
                    }
                }
                else
                {
                    TimeWarp.SetRate(1, true);
                }
            }
        }

        //<summary>
        //Sets initial warping speed according to current altitude
        //</summmary>
        private void StartWarp()
        {
            float[] rates = TimeWarp.fetch.warpRates;
            double altitude = FlightGlobals.ActiveVessel.altitude;
            CelestialBody body = FlightGlobals.ActiveVessel.mainBody;
            int i = 0;
            do
            {
                double altitudeLimit = TimeWarp.fetch.GetAltitudeLimit(i, body);
                if (altitude < altitudeLimit || rates[i] > 1000F) //If this warp rate is NOT allowed
                {
                    //Use the previous rate
                    TimeWarp.SetRate(i - 1, false);
                    return;
                }

                i++;

            } while (i < rates.Length);
        }
    }
}
