using System;
using UnityEngine;
namespace Assets.Scripts.Common
{
    public enum ColorType
    {
        Selected,
        Normal,
    }

    public static class Colorizer
    {
        public static Color32 Color(ColorType type)
        {
            switch (type)
            {
                case ColorType.Normal:
                    // Grey
                    return new Color32(134, 134, 133, 255);
                case ColorType.Selected:
                    // Blue highlight
                    return new Color32(0, 95, 174, 255);
                default:
                    return new Color32(134, 134, 133, 255);
            }
        }

    }
}
