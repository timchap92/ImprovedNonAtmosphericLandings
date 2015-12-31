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
        private int stableCount;
        private bool stable;

        private void Awake()
        {
            this.enabled = false;
        }

        public void Activate(double stopUT)
        {
            Logger.Info("WarpWatcher activated");
            this.stopUT = stopUT;
            this.previousTime = Planetarium.GetUniversalTime();
            this.enabled = true;
            this.stable = false;
            this.stableCount = 0;

            StartWarp();
        }

        public void Update()
        {
            double currentTime = Planetarium.GetUniversalTime();
            updateTime = currentTime - previousTime;
            previousTime = currentTime;
            
            if (currentTime + updateTime > stopUT - 60)
            {
                if (currentTime + updateTime > stopUT - 10)
                {
                    if (TimeWarp.CurrentRate == 1)
                    {
                        stableCount++;
                        if (stableCount > 3)
                        {
                            Logger.Info("Time warp is stable at 1x.");
                            this.stable = true;
                        }
                    }
                    else
                    {
                        stableCount = 0;
                        TimeWarp.SetRate(0, true);
                    }
                    if (currentTime > stopUT)
                    {
                        this.enabled = false;
                        Logger.Info("Time has passed the target UT. Disabling WarpWatcher.");
                    }
                }
                else
                {
                    if (FlightGlobals.ActiveVessel.altitude > TimeWarp.fetch.GetAltitudeLimit(1, FlightGlobals.ActiveVessel.mainBody))
                    {
                        TimeWarp.SetRate(1, true);
                    }
                    else
                    {
                        TimeWarp.SetRate(0, true);
                        stableCount++;
                        if (stableCount > 3)
                        {
                            Logger.Info("Time warp is stable at 1x.");
                            this.stable = true;
                        }
                    }
                }
            }
        }

        public bool IsTimeWarpStable()
        {
            return (TimeWarp.CurrentRate == 1 && stable);
        }

        //<summary>
        //Sets initial warping speed according to current altitude
        //</summmary>
        private void StartWarp()
        {
            double currentTime = Planetarium.GetUniversalTime();
            Logger.Info("Time until target UT is " + (stopUT - currentTime));
            if (currentTime < stopUT - 60)
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
                        Logger.Info("Setting warp to rate index " + (i - 1));
                        TimeWarp.SetRate(i - 1, false);
                        return;
                    }

                    i++;

                } while (i < rates.Length);
            }
        }
    }
}
