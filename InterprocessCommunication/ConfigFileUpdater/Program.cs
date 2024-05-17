using System;
using System.Collections;
using System.IO;

namespace ConfigFileUpdater
{
    class Program
    {
        static void Main(string[] args)
        {
            SharpConfig.Configuration Config;
            const string configFilename = "3DTelemedicine.cfg";
            string configPath;
            configPath = Environment.GetEnvironmentVariable("3DTelemedicine_Dir");
            if (!String.IsNullOrEmpty(configPath))
            {
                configPath += Path.DirectorySeparatorChar + configFilename;
            }
            else
            {
                // Otherwise, try and load it from the current directory
                configPath = Environment.CurrentDirectory + Path.DirectorySeparatorChar + configFilename;
            }
            Console.WriteLine("Setting configPath to " + configPath);

            Config = SharpConfig.Configuration.LoadFromFile(configPath);

            // Grab the latest version from OneDrive
            if (!Config["Updates"].Contains("ConfigFileUrl"))
            {
                Console.WriteLine("ERROR.  URL of config file is not set in the config file.  Please set [Updates][ConfigFileUrl] in your 3DTelemedicine.cfg file before running.");
                return;
            }

            string webConfigUrl = Config["Updates"]["ConfigFileUrl"].StringValue;
            Console.WriteLine($"Grabbing config from cloud at {webConfigUrl}");
            string webConfigString = "";
            System.Net.WebClient webClient = new System.Net.WebClient();
            try
            {
                webConfigString = webClient.DownloadString(webConfigUrl);
            }
            catch (System.Net.WebException e)
            {
                Console.WriteLine($"Sorry, couldn't grab a new version of the config file from the web: {e.Message}");
            }

            SharpConfig.Configuration webConfig = SharpConfig.Configuration.LoadFromString(webConfigString);

            Console.WriteLine($"Comparing your config {configPath} with cloud...");

            foreach(SharpConfig.Section section in webConfig)
            {
                if(!Config.Contains(section.Name))
                {
                    Console.ForegroundColor = ConsoleColor.Green;
                    Console.Write($"Your file does not contain the {section.Name} section! Would you like me to add it?");
                    Console.ResetColor();
                    if (Console.ReadKey().Key == ConsoleKey.Y)
                    {
                        Config.Add(section);
                    }
                    Console.WriteLine();
                }
                foreach (SharpConfig.Setting setting in section)
                {
                    if (!Config[section.Name].Contains(setting.Name))
                    {
                        Console.ForegroundColor = ConsoleColor.Green;
                        if (!setting.IsArray)
                        {
                            Console.Write($"Your file does not contain [{section.Name}][{setting.Name}].  Would you like me to add [{section.Name}][{setting.Name}] = [{setting.StringValue}]?");
                        }
                        else
                        {
                            string[] values = setting.GetValueArray<string>();
                            Console.Write($"Your file does not contain [{section.Name}][{setting.Name}].  Would you like me to add [{section.Name}][{setting.Name}] = {{[{values}]}}?");
                        }
                        if (Console.ReadKey().Key == ConsoleKey.Y)
                        {
                            Config[section.Name].Add(setting);
                        }
                        Console.ResetColor();
                        Console.WriteLine();
                    }
                    else if (Config[section.Name][setting.Name].IsArray)
                    {
                        string[] values = Config[section.Name][setting.Name].GetValueArray<string>();
                        for(int i = 0; i < values.Length; i++)
                        {
                            if(values[i] != setting.StringValueArray[i])
                            {
                                Console.ForegroundColor = ConsoleColor.Yellow;
                                Console.WriteLine($"NOTICE: Your value for [{section.Name}][{setting.Name}][{i}] = [{values[i]}] does not match the web: [{setting.StringValueArray[i]}]");
                                Console.ResetColor();
                            }
                        }
                    }
                    else if (Config[section.Name][setting.Name].StringValue != setting.StringValue)
                    {
                        Console.ForegroundColor = ConsoleColor.Yellow;
                        Console.WriteLine($"NOTICE: Your file has [{section.Name}][{setting.Name}] = [{Config[section.Name][setting.Name].StringValue}] but the web has [{setting.StringValue}]");
                        Console.ResetColor();
                    }
                }
            }
            // Now remove now-deprecated sections or values that aren't in the web config file
            ArrayList sectionsToDelete = new ArrayList();
            ArrayList settingsToDelete = new ArrayList();
            foreach (SharpConfig.Section section in Config)
            {
                if(!webConfig.Contains(section.Name))
                {
                    Console.ForegroundColor = ConsoleColor.Red;
                    Console.WriteLine($"The web config no longer contains a section for [{section.Name}].  Should I delete it? [Y/N]");
                    Console.ResetColor();
                    if(Console.ReadKey().Key == ConsoleKey.Y)
                    {
                        sectionsToDelete.Add(section);
                    }
                    Console.WriteLine();
                }
                else
                {
                    foreach(SharpConfig.Setting setting in section)
                    {
                        if(!webConfig.Contains(section.Name, setting.Name))
                        {
                            Console.ForegroundColor = ConsoleColor.Red;
                            Console.WriteLine($"The web config no longer contains [{section.Name}][{setting.Name}].  Should I delete it? [Y/N]");
                            Console.ResetColor();
                            if(Console.ReadKey().Key == ConsoleKey.Y)
                            {
                                object[] element = { section, setting };
                                settingsToDelete.Add( element );
                            }
                            Console.WriteLine();
                        }
                    }
                }
            }

            foreach(SharpConfig.Section section in sectionsToDelete)
            {
                Config.Remove(section.Name);
            }
            foreach(object[] setting in settingsToDelete)
            {
                Config[(setting[0] as SharpConfig.Section).Name].Remove((setting[1] as SharpConfig.Setting).Name);
            }
            Console.ForegroundColor = ConsoleColor.Green;
            Console.WriteLine($"Saving {configPath}.  Are you sure you want to write all of the above changes? (Differing values will NOT be written)");
            Console.ResetColor();
            if (Console.ReadKey().Key == ConsoleKey.Y)
            {
                Config.SaveToFile(configPath);
            }
            Console.WriteLine();
        }
    }
}
