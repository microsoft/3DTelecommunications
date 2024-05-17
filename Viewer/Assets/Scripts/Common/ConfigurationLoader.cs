using SharpConfig;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Assets.Scripts.Common
{
    public static class ConfigurationLoader
    {
        public static Configuration LoadConfiguration(string baseConfigPath, params string[] fallbacks)
        {
            Configuration config = new Configuration();

            var pathsToCheck = new string[] { baseConfigPath }.Concat(fallbacks);
            var finalPaths =
                pathsToCheck
                    .Where(n => !string.IsNullOrWhiteSpace(n))
                    .SelectMany(n => new string[] {
                        GetConfigPath(n, "default"),
                        n,
                        GetConfigPath(n, "local")
                    })
                    .Where(n => File.Exists(n));

            foreach (string path in finalPaths) {
                MergeConfig(config, ReadConfiguration(path, path == baseConfigPath));
            }
            return config;
        }

        private static string GetConfigPath(string basePath, string type)
        {
            string ext = Path.GetExtension(basePath).ToLowerInvariant();
            string cfgPath = Path.GetFileNameWithoutExtension(basePath);
            return Path.GetFullPath(Path.Combine(Path.GetDirectoryName(basePath), $"./{cfgPath}.{type}{ext}"));
        }

        private static void MergeConfig(Configuration config, Configuration config2)
        {
            foreach (Section section in config2)
            {
                Section toUpdate;
                if (!config.Contains(section.Name))
                {
                    toUpdate = new Section(section.Name);
                    config.Add(toUpdate);
                } 
                else
                {
                    toUpdate = config[section.Name];
                }

                foreach (Setting updateSetting in section)
                {
                    if (!toUpdate.Contains(updateSetting.Name))
                    {
                        toUpdate.Add(updateSetting);
                    } 
                    else
                    {
                        toUpdate[updateSetting.Name].RawValue = updateSetting.RawValue;
                    }
                }
            }
        }

        private static Configuration ReadConfiguration(string path, bool errorLogIfMissing)
        {
            Configuration config = new Configuration();
            try
            {
                config = Configuration.LoadFromFile(path);

                if (config != null)
                {
                    OutputHelper.OutputLog($"[settings]Config file loaded successfully: {path}");
                }
            }
            catch (Exception e)
            {
                if (errorLogIfMissing)
                {
                    OutputHelper.OutputLog($"[settings]Failed to load Config file: {path}\n" + e.ToString());
                }
            }
            return config;
        }
    }
}
