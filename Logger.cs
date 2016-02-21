using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace ImprovedNonAtmosphericLandings
{
    class Logger
    {
        private static readonly String prefix = "[ImprovedNonAtmosphericLandings]: ";

        public static void Debug(String msg)
        {
            //UnityEngine.Debug.Log(prefix + msg);
        }

        public static void Info(String msg)
        {
            UnityEngine.Debug.Log(prefix + msg);
        }

        public static void Warn(String msg)
        {
            UnityEngine.Debug.LogWarning(prefix + msg);
        }

        public static void Error(String msg)
        {
            UnityEngine.Debug.LogError(prefix + msg);
        }
    }
}
