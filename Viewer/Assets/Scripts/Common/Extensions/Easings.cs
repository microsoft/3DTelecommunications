using System;
using UnityEngine;

namespace Assets.Scripts.Common.Extensions
{
    public delegate void EasingAssigner<T>(T easedValue);

    public delegate EasingResult<T> Easing<T>(DateTime currentTime);

    public static class Easings
    {
        public static Easing<int> Ease(this int start,
            int end,
            DateTime startTime,
            long durationMS,
            EaseType easeFn = EaseType.Linear,
            long delayMS = 0
        )
        {
            startTime = startTime.AddMilliseconds(delayMS);
            return (DateTime currentTime) =>
            {
                float progress = durationMS > 0 ? EaseFunction.Ease(Mathf.Clamp((float)(currentTime - startTime).TotalMilliseconds / durationMS, 0, 1), easeFn) : 1;
                int value = (int)(start + ((end - start) * progress));
                return new EasingResult<int>()
                {
                    complete = progress == 1.0f,
                    value = value,
                    time = currentTime
                };
            };
        }

        public static Easing<float> Ease(this float start,
            float end,
            DateTime startTime,
            long durationMS,
            EaseType easeFn = EaseType.Linear,
            long delayMS = 0
        )
        {
            startTime = startTime.AddMilliseconds(delayMS);
            return (DateTime currentTime) =>
            {
                float progress = durationMS > 0 ? EaseFunction.Ease(Mathf.Clamp((float)(currentTime - startTime).TotalMilliseconds / durationMS, 0, 1), easeFn) : 1;
                float value = start + ((end - start) * progress);
                return new EasingResult<float>()
                {
                    complete = progress == 1.0f,
                    value = value,
                    time = currentTime
                };
            };
        }

        public static Easing<Vector2> Ease(this Vector2 start, 
            Vector2 end,
            DateTime startTime,
            long durationMS,
            EaseType easeFn = EaseType.Linear,
            long delayMS = 0
        )
        {
            startTime = startTime.AddMilliseconds(delayMS);
            return (DateTime currentTime) =>
            {
                float progress = durationMS > 0 ? EaseFunction.Ease(Mathf.Clamp((float)(currentTime - startTime).TotalMilliseconds / durationMS, 0, 1), easeFn) : 1;
                Vector2 value = start + ((end - start) * progress);
                return new EasingResult<Vector2>()
                {
                    complete = progress == 1.0f,
                    value = value,
                    time = currentTime
                };
            };
        }

        public static Easing<Vector3> Ease(this Vector3 start,
            Vector3 end,
            DateTime startTime,
            long durationMS,
            EaseType easeFn = EaseType.Linear,
            long delayMS = 0
        )
        {
            startTime = startTime.AddMilliseconds(delayMS);
            return (DateTime currentTime) =>
            {
                float progress = durationMS > 0 ? EaseFunction.Ease(Mathf.Clamp((float)(currentTime - startTime).TotalMilliseconds / durationMS, 0, 1), easeFn) : 1;
                Vector3 value = start + ((end - start) * progress);
                return new EasingResult<Vector3>()
                {
                    complete = progress == 1.0f,
                    value = value,
                    time = currentTime
                };
            };
        }
    }

    public class EasingResult<T>
    {
        public bool complete;
        public T value;
        public DateTime time;
    }
}
