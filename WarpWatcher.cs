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

        //Disable this monobehaviour on awake
        private void Awake()
        {
            this.enabled = false;
        }

        public void Activate(double stopUT)
        {
            Logger.Debug("WarpWatcher activated");
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
            
            //Set warp to 5x when within one minute of thrust, and 1x when within 10sec.
            if (currentTime + updateTime > stopUT - 60 && TimeWarp.WarpMode == TimeWarp.Modes.HIGH) //Only checks warp if it is not in physics mode
            {
                if (currentTime + updateTime > stopUT - 10)
                {
                    if (TimeWarp.CurrentRate == 1)
                    {
                        stableCount++;
                        if (stableCount > 3)
                        {
                            Logger.Debug("Time warp is stable at 1x.");
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
                        Logger.Debug("Time has passed the target UT. Disabling WarpWatcher.");
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
                            Logger.Debug("Time warp is stable at 1x.");
                            this.stable = true;
                        }
                    }
                }
            }
            else if (TimeWarp.WarpMode == TimeWarp.Modes.LOW)
            {
                this.stable = true;
                if (currentTime > stopUT)
                {
                    this.enabled = false;
                }
                //TODO: Send warning to user to disable physics warp
            }
        }

        /// <summary>
        /// Disables this monobehaviour
        /// </summary>
        public void Disable()
        {
            this.enabled = false;
            TimeWarp.SetRate(0, true);
        }

        /// <summary>
        /// Returns true if:
        /// a) Timewarp has been 1x for at least 3 updates
        /// b) Timewarp is in physics mode
        /// </summary>
        /// <returns></returns>
        public bool IsTimeWarpStable()
        {
            return ((TimeWarp.CurrentRate == 1 || TimeWarp.WarpMode == TimeWarp.Modes.LOW) && stable);
        }

        //<summary>
        //Sets initial warping speed according to current altitude
        //</summmary>
        private void StartWarp()
        {
            double currentTime = Planetarium.GetUniversalTime();
            Logger.Debug("Time until target UT is " + (stopUT - currentTime));
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
                        Logger.Debug("Setting warp to rate index " + (i - 1));
                        TimeWarp.SetRate(i - 1, false);
                        return;
                    }

                    i++;

                } while (i < rates.Length);
            }
        }
    }
}
