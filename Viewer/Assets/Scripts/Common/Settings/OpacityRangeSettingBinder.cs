using UnityEngine;

namespace Assets.Scripts.Common.Settings
{
    public class OpacityRangeSettingBinder : RangeSettingBinderBase<Vector4>
    {
        /// <summary>
        /// Computes the value for the Settings based the given slider value
        /// </summary>
        /// <param name="value">The slider value</param>
        /// <returns></returns>
        protected override Vector4 ComputeSettingsValueFromUIValue(float value)
        {
            Color SpotlightColors = settings.SpotlightMaskColor;
            float alpha = Mathf.Clamp(
                minValue + Mathf.Pow(value - minValue, exponentialScale),
                minValue,
                maxValue
            );
            return new Color(SpotlightColors.r, SpotlightColors.b, SpotlightColors.g, alpha);
        }

        /// <summary>
        /// Computes the value for the Slider based the given settings value
        /// </summary>
        /// <param name="value">The settings value</param>
        /// <returns></returns>
        protected override float ComputeUIValueFromSettingsValue(Vector4 value)
        {
            Color SpotlightColors = settings.SpotlightMaskColor;
            float alpha = SpotlightColors.a;
            return Mathf.Clamp(
                minValue + Mathf.Pow(alpha - minValue, 1.0f / exponentialScale),
                minValue,
                maxValue
            );
        }

        /// <summary>
        /// <inheritdoc />
        /// </summary>
        /// <returns></returns>
        protected override Vector4 GetSettingDefaultValue()
        {
            return Color.black;
        }
    }
}
