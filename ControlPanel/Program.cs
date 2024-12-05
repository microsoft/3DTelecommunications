namespace ControlPanel
{
    internal static class Program
    {
        /// <summary>
        ///  The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            OutputHelper.Init();
            // To customize application configuration such as set high DPI settings or default font,
            // see https://aka.ms/applica
            // tionconfiguration.
            ApplicationConfiguration.Initialize();
            // First initialize settings manager so we can get values from the config file
            SettingsManager.Init();
            ControlPanel controlPanel = new ControlPanel();
            OutputHelper.SetControlPanel(controlPanel);


            Application.Run(controlPanel);
        }

    }
}