using SharpConfig;
using System;
using UnityEngine;

namespace Assets.Scripts.Common.Extensions
{
    public static class SharpConfigurationExtensions
    {
        /// <summary>
        /// Gets the setting value or defaults to the given default value
        /// </summary>
        /// <typeparam name="T">The type of the setting</typeparam>
        /// <param name="config"></param>
        /// <param name="section">The section of the setting</param>
        /// <param name="setting">The name of the setting</param>
        /// <param name="defaultValue">The default value to return if the setting doesn't exist</param>
        /// <returns></returns>
        public static T GetOrDefault<T>(this Configuration config, string section, string setting, T defaultValue)
        {
            T result = defaultValue;
            if (config.Contains(section, setting))
            {
                try
                {
                    result = config[section][setting].GetValue<T>();
                }
                catch (Exception e)
                {
                    Debug.LogError(e);
                }
            }
            return result;
        }

        /// <summary>
        /// Gets the setting value or defaults to the given default value
        /// </summary>
        /// <typeparam name="T">The type of the setting</typeparam>
        /// <param name="config"></param>
        /// <param name="key">The configuration key</param>
        /// <param name="defaultValue">The default value to return if the setting doesn't exist</param>
        /// <returns></returns>
        public static T[] GetOrDefaultArray<T>(this Configuration config, SettingName key, T[] defaultValue)
        {
            return config.GetOrDefaultArray(key.Section, key.Setting, defaultValue);
        }

        /// <summary>
        /// Gets the setting value or defaults to the given default value
        /// </summary>
        /// <typeparam name="T">The type of the setting</typeparam>
        /// <param name="config"></param>
        /// <param name="section">The section of the setting</param>
        /// <param name="setting">The name of the setting</param>
        /// <param name="defaultValue">The default value to return if the setting doesn't exist</param>
        /// <returns></returns>
        public static T[] GetOrDefaultArray<T>(this Configuration config, string section, string setting, T[] defaultValue)
        {
            T[] result = defaultValue;
            if (config.Contains(section, setting))
            {
                try
                {
                    result = config[section][setting].GetValueArray<T>();
                }
                catch (Exception e)
                {
                    Debug.LogError(e);
                }
            }
            return result;
        }

        /// <summary>
        /// Gets the setting value or defaults to the given default value
        /// </summary>
        /// <typeparam name="T">The type of the setting</typeparam>
        /// <param name="config"></param>
        /// <param name="key">The configuration key</param>
        /// <param name="defaultValue">The default value to return if the setting doesn't exist</param>
        /// <returns></returns>
        public static T GetOrDefault<T>(this Configuration config, SettingName key, T defaultValue)
        {
            return config.GetOrDefault(key.Section, key.Setting, defaultValue);
        }


        /// <summary>
        /// Adds or updates the a setting
        /// </summary>
        /// <typeparam name="T">The type of the setting</typeparam>
        /// <param name="config"></param>
        /// <param name="section">The section of the setting</param>
        /// <param name="setting">The name of the setting</param>
        /// <param name="value">The new value of the settings</param>
        public static void AddOrUpdate<T>(this Configuration config, string section, string setting, T value)
        {
            if (!config.Contains(section))
            {
                config.Add(section);
            }
            Section s = config[section];
            if (s.Contains(setting))
            {
                s[setting].SetValue(value);
            }
            else
            {
                s.Add(setting, value);
            }
        }

        /// <summary>
        /// Adds or updates the a setting
        /// </summary>
        /// <typeparam name="T">The type of the setting</typeparam>
        /// <param name="config"></param>
        /// <param name="key">The configuration key</param>
        /// <param name="value">The new value of the settings</param>
        public static void AddOrUpdate<T>(this Configuration config, SettingName key, T value)
        {
            config.AddOrUpdate(key.Section, key.Setting, value);
        }
    }
}
