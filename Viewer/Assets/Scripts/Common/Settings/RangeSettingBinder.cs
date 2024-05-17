
using UnityEngine;

namespace Assets.Scripts.Common.Settings
{
    public class RangeSettingBinder: RangeSettingBinderBase<float>
    {

        /// <summary>
        /// Computes the value for the Settings based the given slider value
        /// </summary>
        /// <param name="value">The slider value</param>
        /// <returns></returns>
        protected override float ComputeSettingsValueFromUIValue(float value)
        {
            return Mathf.Clamp(
                minValue + Mathf.Pow(value - minValue, exponentialScale),
                minValue,
                maxValue
            );
        }

        /// <summary>
        /// Computes the value for the Slider based the given settings value
        /// </summary>
        /// <param name="value">The settings value</param>
        /// <returns></returns>
        protected override float ComputeUIValueFromSettingsValue(float value)
        {
            return Mathf.Clamp(
                minValue + Mathf.Pow(value - minValue, 1.0f / exponentialScale),
                minValue,
                maxValue
            );
        }

        /// <summary>
        /// <inheritdoc />
        /// </summary>
        /// <returns></returns>
        protected override float GetSettingDefaultValue()
        {
            return 0f;
        }
    }
}
