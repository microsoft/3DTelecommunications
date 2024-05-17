using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace Assets.Scripts
{
    public static class Utils
    {
        public static byte[] ConvertToBytes(int[] values, int count)
        {
            var result = new byte[count * sizeof(int)];
            Buffer.BlockCopy(values, 0, result, 0, result.Length);
            return result;
        }

        public static byte[] ConvertToBytes(float[] values, int count)
        {
            var result = new byte[count * sizeof(float)];
            Buffer.BlockCopy(values, 0, result, 0, result.Length);
            return result;
        }

        public static float ComputeFPS(DateTimeOffset lastUpdate, DateTimeOffset currentTime, int numFramesRendered, float previousFPS, float smoothing = 0.5f)
        {
            if ((currentTime - lastUpdate).TotalSeconds <= 0)
            {
                return 0f;
            }
            float current = numFramesRendered / (float)(currentTime - lastUpdate).TotalSeconds;
            return Mathf.Lerp(previousFPS, current, smoothing);
        }
        public static float ComputeFPS(DateTimeOffset lastUpdate, DateTimeOffset currentTime, float numFramesRendered, float previousFPS, float smoothing = 0.5f)
        {
            if ((currentTime - lastUpdate).TotalSeconds <= 0)
            {
                return 0f;
            }
            float current = numFramesRendered / (float)(currentTime - lastUpdate).TotalSeconds;
            return Mathf.Lerp(previousFPS, current, smoothing);
        }
    }
}
