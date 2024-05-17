using UnityEngine;
using UnityEngine.UI;
using Assets.Scripts.Common.Extensions;

namespace Assets.Scripts.Common.Settings
{
    public abstract class RangeSettingBinderBase<T> : MonoBehaviour
    {
        [SerializeField]
        public string section;

        [SerializeField]
        public string setting;

        [SerializeField]
        public float defaultValue = 0f;

        [SerializeField]
        public float minValue = 0f;

        [SerializeField]
        public float maxValue = 1f;

        [SerializeField]
        public Slider rangeSlider;

        [Header("Optional")]
        [SerializeField]
        public SettingsManager settings;

        [SerializeField]
        // The exponential scale to use when applying slider value changes.
        public float exponentialScale = 1.0f;

        /// <summary>
        /// The name of our setting
        /// </summary>
        protected SettingName rangeSetting;

        /// <summary>
        /// <inheritdoc/>
        /// </summary>
        protected virtual void OnEnable()
        {
            if (string.IsNullOrWhiteSpace(section))
            {
                Debug.LogWarning("Section required on setting binder!");
                return;
            }
            if (string.IsNullOrWhiteSpace(setting))
            {
                Debug.LogWarning("Section required on setting binder!");
                return;
            }

            if (rangeSlider != null)
            {
                rangeSlider.minValue = minValue;
                rangeSlider.maxValue = maxValue;
                rangeSlider.onValueChanged.AddListener(OnUIValueChanged);
            }

            if (settings == null)
            {
                settings = SettingsManager.Instance;
            }
            this.rangeSetting = new SettingName(section, setting);
            SettingsManager.Instance.OnChange += OnUnfilteredSettingChanged;

            OnSettingsValueChanged(settings.GetOrDefault(rangeSetting, GetSettingDefaultValue()));
        }

        /// <summary>
        /// <inheritdoc />
        /// </summary>
        protected virtual void OnDisable()
        {
            if (rangeSlider != null)
            {
                rangeSlider.onValueChanged.RemoveListener(OnUIValueChanged);
            }
            SettingsManager.Instance.OnChange -= OnUnfilteredSettingChanged;

        }

        /// <summary>
        /// Listener for when the settings value changes
        /// </summary>
        /// <param name="value">The settings value</param>
        protected virtual void OnSettingsValueChanged(T value)
        {
            if (rangeSlider != null)
            {
                rangeSlider.SetValueWithoutNotify(ComputeUIValueFromSettingsValue(value));
            }
        }

        /// <summary>
        /// Listener for when the slider value changes
        /// </summary>
        /// <param name="value">The slider value</param>
        protected virtual void OnUIValueChanged(float value)
        {
            settings.AddOrUpdate(rangeSetting, ComputeSettingsValueFromUIValue(value));
            settings.SaveUserPreferences();
        }

        /// <summary>
        /// Computes the value for the Settings based the given slider value
        /// </summary>
        /// <param name="value">The slider value</param>
        /// <returns></returns>
        protected abstract T ComputeSettingsValueFromUIValue(float value);

        /// <summary>
        /// Computes the value for the Slider based the given settings value
        /// </summary>
        /// <param name="value">The settings value</param>
        /// <returns></returns>
        protected abstract float ComputeUIValueFromSettingsValue(T value);

        /// <summary>
        /// Gets the default value for the setting
        /// </summary>
        /// <returns></returns>
        protected abstract T GetSettingDefaultValue();

        /// <summary>
        /// Listener for all settings changes
        /// </summary>
        /// <param name="updatedSetting">The setting that was updated</param>
        /// <param name="value">The value for the setting</param>
        private void OnUnfilteredSettingChanged(SettingName updatedSetting, object value)
        {
            // Filter to our setting
            if (rangeSetting == updatedSetting)
            {
                OnSettingsValueChanged((T)value);
            }
        }
    }
}
