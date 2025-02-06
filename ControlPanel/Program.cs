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
            string mutexName = "3D Telemedicine Control Panel";
            using (Mutex mutex = new Mutex(true, mutexName, out bool createdNew))
            {
                if (!createdNew)
                {
                    System.Windows.Forms.MessageBox.Show("Control Panel is already running. You cannot have two control panels running at the same time.", "Control Panel", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    OutputHelper.OutputLog("Control Panel is already running", OutputHelper.Verbosity.Warning);
                    return;
                }
                Application.SetHighDpiMode(HighDpiMode.SystemAware);
                Application.EnableVisualStyles();
                Application.SetCompatibleTextRenderingDefault(false);
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
}