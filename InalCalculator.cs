using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Threading;
using KSP;

namespace ImprovedNonAtmosphericLandings
{
    class InalCalculator
    {
        private Boolean calculating;
        private Vessel vessel;
        private Thread calcThread;
        private volatile Boolean shouldStop;
        private volatile float result;
        private volatile String stringResult = string.Empty;
        private volatile String status = string.Empty;
        private readonly object statusLock = new Object();

        /*
        public double Calculate()
        {
            if (calculating)
            {
                //Abort calculation and begin new
            }
            //TODO: Check conditions (i.e. descending, no atmosphere etc.)
            //NOTE: may have to go off rails for the approach

            Logger.Info("Calculating path.");

            vessel = FlightGlobals.ActiveVessel;
            double initialMass = vessel.GetTotalMass();
            Vector3d initialVelocity = vessel.GetSrfVelocity();
            Vector3d initialPosition = vessel.GetWorldPos3D();

            Logger.Info("Vessel mass is: [" + initialMass.ToString() + "], velocity is [" + initialVelocity.ToString() + "], position is [" + initialPosition.ToString() + "]}");
            
            return result;
        }
        */

        public InalCalculator()
        {
            //Create thread that will perform background calculation work
            calcThread = new Thread(new ThreadStart(SlowMethod)) { IsBackground = true };
        }

        public String GetStringResult()
        {
            return stringResult;
        }

        public String GetStatus()
        {
            lock (statusLock)
            {
                return status;
            }
        }

        //Starts the thread that will background calculate the result
        public void BackgroundCalculateResult()
        {
            Logger.Info("Calculation initiated.");

            //Empty result so that it clears from GUI
            stringResult = string.Empty;

            //Set status so that it appears in the GUI
            lock (statusLock)
            {
                status = "Calculating...";
            }

            //Stop previous thread (if it exists)
            Stop();

            //Start thread
            shouldStop = false;
            calcThread.Start();
        }

        public void SlowMethod()
        {
            int i = 0;
            while (i < 10 && !shouldStop)
            {
                //Do all the work here
                Logger.Info("Sleeping... " + i);
                //Set status so that it appears in the GUI
                lock (statusLock)
                {
                    status = "Iteration " + i;
                }
                Thread.Sleep(1000);
                i++;
            }

            if (!shouldStop)
            {
                //Thread completed successfully
                result = 123.456f;
                stringResult = result.ToString("N1") + " seconds";
                Logger.Info("Thread completed.");
            }
            else
            {
                //Thread stop requested
                Logger.Info("Stop was requested.");
            }
        }

        public void Stop()
        {
            Logger.Info("Stopping thread...");
            
            //Set flag to stop thread
            shouldStop = true;

            //Set status so that it appears in the GUI
            lock (statusLock)
            {
                status = "Calculating...";
            }

            while (calcThread.IsAlive)
            {
                try
                {
                    calcThread.Join();
                }
                catch (Exception ex)
                {
                    Logger.Info(ex.Message);
                }

            }

            Logger.Info("Thread was stopped.");
            status = "Cancelled";

            //Set status so it appears in GUI
            status = string.Empty;

            Logger.Info("No running thread detected.");
            
        }
    }
}
