namespace Assets.Scripts.Common
{
    public enum EaseType
    {
        Linear,
        CubicIn,
        CubicOut
    }

    // Borrowed portions from https://github.com/d3/d3-ease
    public static class EaseFunction
    {
        public static float Ease(float t, EaseType fn)
        {
            switch (fn)
            {
                case EaseType.CubicIn:
                    return EaseCubicIn(t);
                case EaseType.CubicOut:
                    return EaseCubicOut(t);
                default:
                    return EaseLinear(t);

            }
        }


        public static float EaseLinear(float t)
        {
            return t;
        }

        public static float EaseCubicIn(float t)
        {
            return t * t * t;
        }

        public static float EaseCubicOut(float t)
        {
            return --t * t * t + 1;
        }
    }
}
