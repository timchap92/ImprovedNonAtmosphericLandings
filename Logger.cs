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

        public static void Info(String msg)
        {
            Debug.Log(prefix + msg);
        }

        public static void Warn(String msg)
        {
            Debug.LogWarning(prefix + msg);
        }

        public static void Error(String msg)
        {
            Debug.LogError(prefix + msg);
        }
    }
}
