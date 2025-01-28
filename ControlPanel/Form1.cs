

using AsyncIO;
using NLog;
using PeabodyNetworkingLibrary;
using System.Collections.Concurrent;
using System.ComponentModel;
using System.Net;
using System.Text;
using static ControlPanel.BotManager;

namespace ControlPanel
{
    public partial class ControlPanel : Form
    {
        private bool _isClosing = false;
        bool TelemedSessionRunning = false;
        bool CalibrationSessionRunning = false;
        Thread SessionStartupThread;
        private TimeSpan BotResponseTimeout = TimeSpan.FromSeconds(10);
        private List<BotManager.VersionContent> versionContentsToCheck;

        public ControlPanel()
        {
            // Required to get NetMQ to work correctly
            ForceDotNet.Force();

            InitializeComponent();

            this.Load += ControlPanel_Loaded;
            this.FormClosing += ControlPanel_FormClosing;

            dataGridView_config.CellValueChanged += dataGridView_config_CellValueChanged;
        }

        private async void ControlPanel_FormClosing(object? sender, FormClosingEventArgs e)
        {
            if (_isClosing)
            {
                return;
            }
            SetStartButtonClosing();
            _isClosing = true;
            // Make sure we close everything else first
            e.Cancel = true;
            this.Enabled = false;
            TelemedSessionRunning = false;
            CalibrationSessionRunning = false;
            await Task.Run(() => BotManager.Instance.OnDestroy());

            // Now finish closing
            this.Close();
        }

        private void ControlPanel_Loaded(object? sender, EventArgs e)
        {
            UpdateStatusText("Initializing Bot Manager...");
            BotManager.Init(this);
            UpdateStatusText("Creating Bots...");
            BotManager.Instance.Start();
            UpdateStatusText("Updating Daemon Bot Table...");
            // Bind the depthGenDaemonBots array to the DataGridView
            BindPodBotsToUI();
            BindFusionDaemonBotToUI();
            BindRenderDaemonBotToUI();
            BindFusionStatusBotToUI();
            BindRenderStatusBotToUI();
            dataGridView_broadcast_camera_applications.CellFormatting += DataGridView_broadcast_camera_applications_CellFormatting;
            dataGridView_broadcast_camera_daemons.CellFormatting += DataGridView_broadcast_camera_daemons_CellFormatting;
            BindVersionMapToUI();
            BindConfigToUI();
        }

        private void BindConfigToUI()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(BindConfigToUI));
                return;
            }

            dataGridView_config.Visible = true;
            dataGridView_config.AutoGenerateColumns = false;
            dataGridView_config.AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.AllCells;

            var configItems = LoadConfigItems();

            dataGridView_config.DataSource = new BindingList<ConfigItem>(configItems);

            dataGridView_config.Columns.Clear();

            dataGridView_config.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "Section",
                HeaderText = "Section"
            });
            dataGridView_config.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "Setting",
                HeaderText = "Setting"
            });
            dataGridView_config.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "Value",
                HeaderText = "Value"
            });

        }
        private void dataGridView_config_CellValueChanged(object sender, DataGridViewCellEventArgs e)
        {
            var configItems = (BindingList<ConfigItem>)dataGridView_config.DataSource;
            var config = new SharpConfig.Configuration();

            foreach (var item in configItems)
            {
                if (!string.IsNullOrEmpty(item.Section))
                {
                    var section = new SharpConfig.Section(item.Section);
                    config.Add(section);
                }
                else
                {
                    var section = config[config.Count() - 1];
                    section.Add(new SharpConfig.Setting(item.Setting, item.Value));
                }
            }
            SettingsManager.Instance.config = config;
            SettingsManager.Instance.SaveConfigToDisk(config);
        }

        private void BindVersionMapToUI()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(BindVersionMapToUI));
                return;
            }

            dataGridView_software_version_list.Visible = true;

            // Convert the versionMap to a BindingList of VersionInfo
            var versionList = new BindingList<VersionContent>(
                BotManager.Instance.versionContentList
            );

            // Bind the BindingList to the DataGridView
            dataGridView_software_version_list.DataSource = versionList;

            // Customize the DataGridView columns
            dataGridView_software_version_list.AutoGenerateColumns = false;
            dataGridView_software_version_list.Columns.Clear();

            dataGridView_software_version_list.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "unitName",
                HeaderText = "Software Component"
            });
            dataGridView_software_version_list.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "Version",
                HeaderText = "Local Version"
            });
            dataGridView_software_version_list.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "OnlineVersion",
                HeaderText = "Online Version"
            });
            dataGridView_software_version_list.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "UpdateButtonText",
                HeaderText = "Update"
            });
            // Refresh the DataGridView
            
            dataGridView_software_version_list.Refresh();
        }
        private void DataGridView_broadcast_camera_daemons_CellFormatting(object sender, DataGridViewCellFormattingEventArgs e)
        {
            if (dataGridView_broadcast_camera_daemons.Columns[e.ColumnIndex].HeaderText == "Min Voltage" || dataGridView_broadcast_camera_daemons.Columns[e.ColumnIndex].HeaderText == "Max Voltage")
            {
                if (e.Value != null && !string.IsNullOrEmpty(e.Value.ToString()))
                {
                    if (double.TryParse(e.Value.ToString(), out double voltage))
                    {
                        if (voltage <= 4.5 && voltage != 0)
                        {
                            e.CellStyle.BackColor = Color.Yellow;
                        }
                        else if (voltage > 4.5)
                        {
                            e.CellStyle.BackColor = Color.LightGreen;
                        }
                        else
                        {
                            e.CellStyle.BackColor = Color.White;
                        }
                    }
                }
                else
                {
                    e.CellStyle.BackColor = Color.White;
                }
            }
            if (dataGridView_broadcast_camera_daemons.Columns[e.ColumnIndex].HeaderText == "Temp")
            {
                if (e.Value != null && !string.IsNullOrEmpty(e.Value.ToString()))
                {
                    if (double.TryParse(e.Value.ToString(), out double temp))
                    {
                        if (temp >= 50 && temp < 70)
                        {
                            e.CellStyle.BackColor = Color.Yellow;
                        }
                        else if (temp >= 70)
                        {
                            e.CellStyle.BackColor = Color.Red;
                        }
                        else if (temp < 50 && temp != 0)
                        {
                            e.CellStyle.BackColor = Color.LightGreen;
                        }
                        else
                        {
                            e.CellStyle.BackColor = Color.White;
                        }
                    }
                }
                else
                {
                    e.CellStyle.BackColor = Color.White;
                }
            }
        }
        private void DataGridView_broadcast_camera_applications_CellFormatting(object sender, DataGridViewCellFormattingEventArgs e)
        {
            // Check if the column is the "FPS" column
            if (dataGridView_broadcast_camera_applications.Columns[e.ColumnIndex].HeaderText == "FPS" ||
                dataGridView_broadcast_camera_applications.Columns[e.ColumnIndex].HeaderText == "Min FPS" ||
                dataGridView_broadcast_camera_applications.Columns[e.ColumnIndex].HeaderText == "Max FPS")
            {
                // Check if the cell value is not null or empty
                if (e.Value != null && !string.IsNullOrEmpty(e.Value.ToString()))
                {
                    int expectedFPSIdx = SettingsManager.Instance.GetValueWithDefault("PodGlobalConfig", "CameraFPS", 1);
                    double expectedFPS = expectedFPSIdx == 0 ? 5 : (expectedFPSIdx == 1 ? 15 : 30);
                    // Try to parse the cell value as a double
                    if (double.TryParse(e.Value.ToString(), out double fps))
                    {
                        // Change the background color based on the value of FPS
                        if (fps > 0 && fps != 100 && (fps < expectedFPS-0.5 || fps > expectedFPS+0.5)) 
                        {
                            e.CellStyle.BackColor = Color.Yellow;
                        }
                        else if (fps > 0)
                        {
                            e.CellStyle.BackColor = Color.LightGreen;
                        }
                        else
                        {
                            e.CellStyle.BackColor = Color.White;
                        }
                    }
                }
                else
                {
                    // If the cell value is null or empty, set the background color to white
                    e.CellStyle.BackColor = Color.White;
                }
            }
            if (dataGridView_broadcast_camera_applications.Columns[e.ColumnIndex].HeaderText == "Temp")
            {
                if (e.Value != null && !string.IsNullOrEmpty(e.Value.ToString()))
                {
                    if (double.TryParse(e.Value.ToString(), out double temp))
                    {
                        if (temp >= 50 && temp < 70)
                        {
                            e.CellStyle.BackColor = Color.Yellow;
                        }
                        else if (temp >= 70)
                        {
                            e.CellStyle.BackColor = Color.Red;
                        }
                        else if (temp < 50 && temp != 0)
                        {
                            e.CellStyle.BackColor = Color.LightGreen;
                        }
                        else
                        {
                            e.CellStyle.BackColor = Color.White;
                        }
                    }
                }
                else
                {
                    e.CellStyle.BackColor = Color.White;
                }
            }
        }
        private void BindRenderDaemonBotToUI()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(BindRenderDaemonBotToUI));
                return;
            }
            // bind the textBox_render_daemon_status to the status parameter of the botManager's render daemon bot
            textBox_render_daemon_status.DataBindings.Clear();
            textBox_render_application_status.DataBindings.Clear();
            textBox_render_daemon_status.DataBindings.Add("Text", BotManager.Instance.renderDaemonBot, "ComponentStatus");
            textBox_render_application_status.DataBindings.Add("Text", BotManager.Instance.renderDaemonBot, "RenderStatus");
        }
        private void BindRenderStatusBotToUI()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(BindRenderStatusBotToUI));
                return;
            }
            textBox_render_fps.DataBindings.Clear();
            textBox_broadcastTab_render_application_status.DataBindings.Clear();
            textBox_render_fps.DataBindings.Add("Text", BotManager.Instance.renderStatusBot, "FPS");
            textBox_broadcastTab_render_application_status.DataBindings.Add("Text", BotManager.Instance.renderStatusBot.componentStatus, "Status");
        }

        private void BindFusionDaemonBotToUI()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(BindFusionDaemonBotToUI));
                return;
            }
            textBox_fusion_daemon_status.DataBindings.Clear();
            textBox_fusion_application_status.DataBindings.Clear();
            textBox_fusion_daemon_status.DataBindings.Add("Text", BotManager.Instance.fusionDaemonBot, "ComponentStatus");
            textBox_fusion_application_status.DataBindings.Add("Text", BotManager.Instance.fusionDaemonBot, "FusionStatus");
        }
        private void BindFusionStatusBotToUI()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(BindFusionStatusBotToUI));
                return;
            }
            textBox_fusion_fps.DataBindings.Clear();
            textBox_broadcastTab_fusion_application_status.DataBindings.Clear();
            textBox_fusion_fps.DataBindings.Add("Text", BotManager.Instance.fusionStatusBot, "FPS");
            textBox_broadcastTab_fusion_application_status.DataBindings.Add("Text", BotManager.Instance.fusionStatusBot.componentStatus, "Status");
        }

        internal void UpdateStatusText(string message)
        {
            toolStripStatusLabel_updateText.Text = message;
        }
        internal void UpdateLogText(string message)
        {
            if (InvokeRequired)
            {
                Invoke(new Action<string>(UpdateLogText), message);
                return;
            }
            // Prepend the new message to the existing text
            textBox_systemLog.Text = message + Environment.NewLine + textBox_systemLog.Text;

            // Optionally, you can limit the length of the text to avoid performance issues
            if (textBox_systemLog.Text.Length > 100000)
            {
                textBox_systemLog.Text = textBox_systemLog.Text.Substring(0, 100000);
            }
        }
        internal void BindPodBotsToUI()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(BindPodBotsToUI));
                return;
            }

            int dgNum = SettingsManager.Instance.GetValueWithDefault("DepthGeneration", "DepthCameraCount", 0, true);

            dataGridView_broadcast_camera_daemons.Visible = true;
            dataGridView_broadcast_camera_applications.Visible = true;
            // Create a BindingSource
            BindingList<StatusBot> bindingList = new BindingList<StatusBot>(BotManager.Instance.depthGenDaemonBots);

            // Bind the BindingSource to the DataGridView
            dataGridView_broadcast_camera_daemons.DataSource = bindingList;
            // Customize the DataGridView columns
            dataGridView_broadcast_camera_daemons.AutoGenerateColumns = false;
            dataGridView_broadcast_camera_daemons.Columns.Clear();

            // Add columns for specific properties of DaemonBot
            dataGridView_broadcast_camera_daemons.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "CameraName",
                HeaderText = "#"
            });
            dataGridView_broadcast_camera_daemons.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "ComponentStatus",
                HeaderText = "Service"
            });
            dataGridView_broadcast_camera_daemons.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "CaptureStatus",
                HeaderText = "Session Software"
            });
            dataGridView_broadcast_camera_daemons.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "CalibrationStatus",
                HeaderText = "Calibration Software"
            });
            dataGridView_broadcast_camera_daemons.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "MinVoltage",
                HeaderText = "Min Voltage"
            });
            dataGridView_broadcast_camera_daemons.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "MaxVoltage",
                HeaderText = "Max Voltage"
            });
            dataGridView_broadcast_camera_daemons.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "Temp",
                HeaderText = "Temp"
            });
            //dataGridView_broadcast_camera_daemons.Columns.Add(new DataGridViewTextBoxColumn
            //{
            //    DataPropertyName = "VersionString",
            //    HeaderText = "Version"
            //});

            bindingList = new BindingList<StatusBot>(BotManager.Instance.depthGenStatusBots);
            dataGridView_broadcast_camera_applications.DataSource = bindingList;
            dataGridView_broadcast_camera_applications.AutoGenerateColumns = false;
            dataGridView_calibration_camera_status.DataSource = bindingList;
            dataGridView_calibration_camera_status.AutoGenerateColumns = false;

            dataGridView_broadcast_camera_applications.Columns.Clear();
            dataGridView_calibration_camera_status.Columns.Clear();

            dataGridView_broadcast_camera_applications.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "CameraName",
                HeaderText = "#"
            });
            dataGridView_calibration_camera_status.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "CameraName",
                HeaderText = "#"
            });
            dataGridView_broadcast_camera_applications.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "FPS",
                HeaderText = "FPS"
            });
            dataGridView_broadcast_camera_applications.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "MaxFPS",
                HeaderText = "Max FPS"
            });
            dataGridView_broadcast_camera_applications.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "MinFPS",
                HeaderText = "Min FPS"
            });
            dataGridView_calibration_camera_status.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "FPS",
                HeaderText = "FPS"
            });
            dataGridView_broadcast_camera_applications.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "Temp",
                HeaderText = "Temp"
            });
            dataGridView_calibration_camera_status.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "Frames",
                HeaderText = "Frames Recorded"
            });
            //dataGridView_broadcast_camera_applications.Columns.Add(new DataGridViewTextBoxColumn
            //{
            //    DataPropertyName = "VersionString",
            //    HeaderText = "Version"
            //});
            // refresh the data grid view
            dataGridView_broadcast_camera_daemons.Refresh();
            dataGridView_broadcast_camera_applications.Refresh();
            dataGridView_calibration_camera_status.Refresh();
        }

        public void Refresh()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(Refresh));
                return;
            }
            dataGridView_broadcast_camera_daemons.Refresh();
            if (tabControl_application_selection.SelectedTab == tabPage_calibration)
            {
                dataGridView_calibration_camera_status.Refresh();
            }
            else
            {
                dataGridView_broadcast_camera_applications.Refresh();
            }
            textBox_fusion_application_status.Refresh();
            textBox_fusion_daemon_status.Refresh();
            textBox_render_application_status.Refresh();
            textBox_render_daemon_status.Refresh();
            button_start_session.Refresh();
            textBox_broadcastTab_render_application_status.Refresh();
            textBox_broadcastTab_fusion_application_status.Refresh();
            textBox_fusion_fps.Refresh();
            textBox_render_fps.Refresh();
        }
        public void RefreshVersionUI()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(RefreshVersionUI));
                return;
            }
            dataGridView_software_version_list.Refresh();
        }
        private void DataGridView_broadcast_camera_daemons_CellClick(object sender, DataGridViewCellEventArgs e)
        {
            // determine if the column the clicked cell belongs to is the "Service Status" column
            if (dataGridView_broadcast_camera_daemons.Columns[e.ColumnIndex].HeaderText == "Session Software")
            {
                // if the contents of the cell are "RUNNING" and the system is not in a session, send another stop session request
                if (dataGridView_broadcast_camera_daemons.Rows[e.RowIndex].Cells[e.ColumnIndex].Value.ToString() == "RUNNING" && !TelemedSessionRunning)
                {
                    BotManager.Instance.BroadcastEventOnce(PeabodyNetworkingLibrary.CONTROL_PANEL_EVENT.CONTROL_PANEL_STOP_REQUESTED);
                }
            }
        }

        private void DataGridView_broadcast_camera_applications_ColumnHeaderMouseClick(object sender, DataGridViewCellMouseEventArgs e)
        {
            if (dataGridView_broadcast_camera_applications.Columns[e.ColumnIndex].HeaderText == "Version")
            {
                BotManager.Instance.BroadcastEventOnce(PeabodyNetworkingLibrary.CONTROL_PANEL_EVENT.BUILD_VERSION_REQUESTED);
            }
        }

        private void DataGridView_broadcast_camera_daemons_ColumnHeaderMouseClick(object sender, DataGridViewCellMouseEventArgs e)
        {
            if (dataGridView_broadcast_camera_daemons.Columns[e.ColumnIndex].HeaderText == "Version")
            {
                // Perform the desired action when the "Version" header is clicked
                BotManager.Instance.BroadcastEventOnce(PeabodyNetworkingLibrary.CONTROL_PANEL_EVENT.BUILD_VERSION_REQUESTED);
            }
        }
        private void button_start_session_Click(object sender, EventArgs e)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(() => button_start_session_Click(sender, e)));
                return;
            }
            BotManager.Instance.StopBroadcastThread();
            if (button_start_session.Text == "Start Session")
            {
                OutputHelper.OutputLog("New session start button clicked!");
                StartingStartSessionButton();
                TelemedSessionRunning = true;

                SessionStartupThread = new Thread(() =>
                {
                    // Send the launch command to the daemon bots
                    int dgNum = SettingsManager.Instance.GetValueWithDefault("DepthGeneration", "DepthCameraCount", 0, true);
                    //kick off the sequence of tasks
                    //start the DG's status bots 
                    for (int i = 0; i < dgNum; ++i)
                    {
                        BotManager.Instance.depthGenStatusBots[i].Start();
                    }
                    //Tell the DG Daemons to start in Capture mode
                    OutputHelper.OutputLog($"Start button clicked.  Broadcasting the start capture signal.");
                    BotManager.Instance.BroadcastSoftwareStartUntilAllDGStart(SOFTWARE.CAPTURE);

                    // Wait for all DG to be running
                    while (TelemedSessionRunning && BotManager.Instance.GetBotsWithSoftwareRunning(SOFTWARE.CAPTURE) < dgNum)
                    {
                        Thread.Sleep(250);
                    }
                    if (!TelemedSessionRunning)
                    {
                        return;
                    }
                    // Now send the depth gen start capture command
                    OutputHelper.OutputLog("All bots have application running.  Sending the global start signal.");
                    BotManager.Instance.BroadcastSystemStartUntilAllDGStartTransmitting();
                    int botsSendingFPS = 0;
                    while (TelemedSessionRunning && botsSendingFPS < dgNum)
                    {
                        Thread.Sleep(250);
                        botsSendingFPS = BotManager.Instance.GetBotsSendingFPS();
                        OutputHelper.OutputLog($"Waiting for all DG's to start sending FPS {botsSendingFPS}/{BotManager.Instance.depthGenStatusBots.Length}");
                    }
                    if (!TelemedSessionRunning)
                    {
                        return;
                    }
                    if (!TelemedSessionRunning)
                        return;
                    // Start fusion
                    //BotManager.Instance.fusionStatusBot.Start();
                    //BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_START_REQUESTED, BitConverter.GetBytes((char)SOFTWARE.FUSION));

                    //while (TelemedSessionRunning && BotManager.Instance.fusionStatusBot.componentStatus.Status != Status.Running)
                    //{
                    //    Thread.Sleep(250);
                    //}

                    //BotManager.Instance.renderStatusBot.Start();
                    //BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_START_REQUESTED, BitConverter.GetBytes((char)SOFTWARE.RENDER));

                });
                SessionStartupThread.Start();
            }
            else // any other case, send a stop session request
            {
                if (button_start_session.Text == "Stop Session")
                {
                    button_start_session.Text = "Stopping...";
                }
                TelemedSessionRunning = false;
                BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_STOP_REQUESTED, BitConverter.GetBytes((char)SOFTWARE.RENDER));
                BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_STOP_REQUESTED, BitConverter.GetBytes((char)SOFTWARE.FUSION));
                BotManager.Instance.BroadcastStopUntilAllDGStop(SOFTWARE.CAPTURE);
            }
        }

        internal void EnableButtons()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(EnableButtons));
                return;
            }
            if (!TelemedSessionRunning && !CalibrationSessionRunning)
            {
                button_start_session.BackColor = Color.LightGreen;
                button_start_session.Text = "Start Session";

                button_start_calibration.BackColor = Color.LightGreen;
                button_start_calibration.Enabled = true;
                button_start_calibration.Text = "Start Calibration";
            }
        }
        private void SetStartButtonClosing()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(SetStartButtonClosing));
                return;
            }
            button_start_session.Text = "Closing...";
        }

        internal void StartingStartSessionButton()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(StartingStartSessionButton));
                return;
            }
            button_start_session.BackColor = Color.Yellow;
            button_start_session.Text = "Starting";
        }
        internal void DisableButtons()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(DisableButtons));
                return;
            }
            if (TelemedSessionRunning || !BotManager.Instance.AllDaemonBotsReadyToBeLaunched_Capture)
            {
                button_start_session.BackColor = Color.Red;
                button_start_session.Text = "Stop Session";
            }
            if ((CalibrationSessionRunning && button_start_calibration.Text == "Start Calibration")  //add any other button states where we might want "cancel" to be available
                || (!CalibrationSessionRunning && !BotManager.Instance.AllDaemonBotsReadyToBeLaunched_Calib))
            {
                button_start_calibration.BackColor = Color.Red;
                button_start_calibration.Enabled = true;
                button_start_calibration.Text = "Stop Calibration";
            }
        }

        public void StopCalibration()
        {
            BotManager.Instance.BroadcastStopUntilAllDGStop(SOFTWARE.CALIBRATION);
            CalibrationSessionRunning = false;
            EnableButtons();
        }
        public void UpdateCalibrationStatusTextbox(string message)
        {
            if (InvokeRequired)
            {
                Invoke(new Action<string>(UpdateCalibrationStatusTextbox), message);
                return;
            }
            textBox_calibration_software_status.Text = message + Environment.NewLine + textBox_calibration_software_status.Text;
        }

        private void button_start_calibration_Click(object sender, EventArgs e)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(() => button_start_session_Click(sender, e)));
                return;
            }
            BotManager.Instance.StopBroadcastThread();
            if (button_start_calibration.Text == "Start Calibration")
            {
                OutputHelper.OutputLog("New calibration start button clicked!");
                StartingCalibrationButton();
                CalibrationSessionRunning = true;

                SessionStartupThread = new Thread(() =>
                {
                    // Send the launch command to the daemon bots
                    int dgNum = SettingsManager.Instance.GetValueWithDefault("DepthGeneration", "DepthCameraCount", 0, true);
                    //kick off the sequence of tasks
                    //start the DG's status bots 
                    for (int i = 0; i < dgNum; ++i)
                    {
                        BotManager.Instance.depthGenStatusBots[i].Start();
                    }
                    //Tell the DG Daemons to start in Capture mode
                    OutputHelper.OutputLog($"Start button clicked.  Broadcasting the start calibration signal.");
                    BotManager.Instance.BroadcastSoftwareStartUntilAllDGStart(SOFTWARE.CALIBRATION, BitConverter.GetBytes((char)PeabodyNetworkingLibrary.CALIBRATION_SOFTWARE_COMPONENT.POD));

                    // Wait for all DG to be running
                    while (CalibrationSessionRunning && BotManager.Instance.GetBotsWithSoftwareRunning(SOFTWARE.CALIBRATION) < dgNum && BotManager.Instance.GetBotsSendingFPS() < dgNum)
                    {
                        UpdateStatusText($"Waiting for all bots to report calibration software running: {BotManager.Instance.GetBotsWithSoftwareRunning(SOFTWARE.CALIBRATION)}/{dgNum}");
                        Thread.Sleep(250);
                    }
                    if (!CalibrationSessionRunning)
                    {
                        StopCalibration();
                    }

                    CalibrationReadyToStartButton();
                });
                SessionStartupThread.Start();
            }
            else if (button_start_calibration.Text == "Record Video")
            {
                StartingVideoRecordingButton();
                CalibrationSessionRunning = true;
                BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CALIBRATION_SOFTWARE_START_CAPTURING_VIDEO);
                SessionStartupThread = new Thread(() =>
                {
                    int dgNum = SettingsManager.Instance.GetValueWithDefault("DepthGeneration", "DepthCameraCount", 0, true);
                    while (CalibrationSessionRunning && BotManager.Instance.GetBotsRecordingVideo() < dgNum)
                    {
                        OutputHelper.OutputLog($"Waiting for all cameras to start recording.  {BotManager.Instance.GetBotsRecordingVideo()}/{dgNum}");
                        Thread.Sleep(250);
                    }
                    if (!CalibrationSessionRunning)
                    {
                        StopCalibration();
                    }
                    OutputHelper.OutputLog("All cameras are now recording.");
                    CalibrationStopRecordingButton();
                    while (CalibrationSessionRunning && BotManager.Instance.GetBotsRecordingVideo() > 0)
                    {
                        // I really shouldn't have to do this, but I can't get the UI to update the frame count reliably
                        Refresh();
                        Thread.Sleep(250);
                    }
                });
                SessionStartupThread.Start();
            }
            else if (button_start_calibration.Text == "Stop Recording")
            {
                TransferringVideosButton();
                CalibrationSessionRunning = true;
                BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CALIBRATION_SOFTWARE_STOP_CAPTURING_VIDEO);
                SessionStartupThread = new Thread(() =>
                {
                    int dgNum = SettingsManager.Instance.GetValueWithDefault("DepthGeneration", "DepthCameraCount", 0, true);
                    while (CalibrationSessionRunning &&
                        (BotManager.Instance.GetBotsWithSoftwareRunning(SOFTWARE.CALIBRATION) > 0 ||
                        BotManager.Instance.GetBotsWithVideoTransferRunning() > 0 ||
                        BotManager.Instance.GetBotsWithVideoTransferComplete() < dgNum))
                    {
                        OutputHelper.OutputLog($"Waiting for all cameras to finish transferring video.  " +
                            $"Bots Running: {BotManager.Instance.GetBotsWithSoftwareRunning(SOFTWARE.CALIBRATION)}/{dgNum} " +
                            $"Bots Transferring: {BotManager.Instance.GetBotsWithVideoTransferRunning()}/{dgNum}" +
                            $"Bots Completed: {BotManager.Instance.GetBotsWithVideoTransferComplete()}/{dgNum}");
                        Thread.Sleep(250);
                    }
                    if (!CalibrationSessionRunning)
                    {
                        StopCalibration();
                    }
                    OutputHelper.OutputLog("All bots done transferring video.");
                    StartCalibrationStage2Button();
                });
                SessionStartupThread.Start();
            }
            else if (button_start_calibration.Text == "Calculate Calibration")
            {
                CancelCalibrationButton();
                CalibrationSessionRunning = true;
                BotManager.Instance.BroadcastStartEventOnce(SOFTWARE.CALIBRATION, BitConverter.GetBytes((char)1));
                SessionStartupThread = new Thread(() =>
                {
                    while (CalibrationSessionRunning && !BotManager.Instance.CalibrationComplete)
                    {
                        Thread.Sleep(250);
                    }
                    if (!CalibrationSessionRunning)
                    {
                        StopCalibration();
                    }
                    UpdateCalibrationStatusTextbox("Calibration complete.  Calibration files are being distributed.");
                    // Distribute calibration files

                    bool success = false;
                    int elapsed = 0;

                    while (!success && (elapsed < BotResponseTimeout.TotalMilliseconds))
                    {
                        if (BotManager.Instance.renderDaemonBot.componentStatus.NewDataReceived && BotManager.Instance.fusionDaemonBot.componentStatus.NewDataReceived)
                        {
                            OutputHelper.OutputLog($"Render and Fusion both report new data received. Successful transmission.");
                            success = true;
                            break;
                        }
                        Thread.Sleep(100);
                        elapsed += 100;
                    }
                    if (!BotManager.Instance.renderDaemonBot.componentStatus.NewDataReceived)
                    {
                        OutputHelper.OutputLog("ERROR: Render never acknowledged receiving new calibration data");
                    }
                    if (!BotManager.Instance.fusionDaemonBot.componentStatus.NewDataReceived)
                    {
                        OutputHelper.OutputLog("ERROR: Fusion never acknowledged receiving new calibration data");
                    }
                    if (success)
                    {
                        UpdateCalibrationStatusTextbox("Calibration files distributed.  Calibration complete.");
                    }
                    else
                    {
                        UpdateCalibrationStatusTextbox("Calibration files not distributed.  Calibration complete.");
                    }
                    StopCalibration();
                });
                SessionStartupThread.Start();
            }
            else
            {
                StopCalibration();
            }
        }

        private void CancelCalibrationButton()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(CancelCalibrationButton));
                return;
            }
            button_start_calibration.BackColor = Color.Red;
            button_start_calibration.Enabled = true;
            button_start_calibration.Text = "Cancel";
        }

        private void StartCalibrationStage2Button()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(StartCalibrationStage2Button));
                return;
            }
            button_start_calibration.BackColor = Color.ForestGreen;
            button_start_calibration.Enabled = true;
            button_start_calibration.Text = "Calculate Calibration";
            UpdateCalibrationStatusTextbox("Transfer complete.  Press Calculate Calibration to start the calculation.");
        }

        private void TransferringVideosButton()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(TransferringVideosButton));
                return;
            }
            button_start_calibration.BackColor = Color.CornflowerBlue;
            button_start_calibration.Enabled = false;
            button_start_calibration.Text = "Transferring videos...";
            UpdateCalibrationStatusTextbox("Recording stopped.  Videos are transferring from the cameras to the calibration PC.  Please wait.");
        }

        private void CalibrationStopRecordingButton()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(CalibrationStopRecordingButton));
                return;
            }
            button_start_calibration.BackColor = Color.Red;
            button_start_calibration.Enabled = true;
            button_start_calibration.Text = "Stop Recording";
            UpdateCalibrationStatusTextbox("Cameras are recording.  Slowly move the calibration board around in the volume so all cameras can see it.  Press Stop Recording when you are done.");
        }

        private void StartingVideoRecordingButton()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(StartingVideoRecordingButton));
                return;
            }
            button_start_calibration.BackColor = Color.CornflowerBlue;
            button_start_calibration.Enabled = true;
            button_start_calibration.Text = "Recording Starting...";
            UpdateCalibrationStatusTextbox("Telling the cameras to start recording.  Please wait.");
        }

        private void CalibrationReadyToStartButton()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(CalibrationReadyToStartButton));
                return;
            }
            button_start_calibration.BackColor = Color.ForestGreen;
            button_start_calibration.Enabled = true;
            button_start_calibration.Text = "Record Video";
            UpdateCalibrationStatusTextbox("Cameras are ready.  Click Record Video to start recording calibration video on all cameras.");
        }

        private void StartingCalibrationButton()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(StartingCalibrationButton));
                return;
            }
            button_start_calibration.BackColor = Color.Yellow;
            button_start_calibration.Enabled = false;
            button_start_calibration.Text = "Software Starting...";
            UpdateCalibrationStatusTextbox("New calibration start button clicked.  Starting K4ARecorder on the cameras.");
        }
        private List<ConfigItem> LoadConfigItems()
        {
            var configItems = new List<ConfigItem>();
            var config = SettingsManager.Instance.config;

            foreach (var section in config)
            {
                // Add a row for the section header
                configItems.Add(new ConfigItem
                {
                    Section = section.Name,
                    Setting = string.Empty,
                    Value = string.Empty
                });

                // Add rows for each setting in the section
                foreach (var setting in section)
                {
                    configItems.Add(new ConfigItem
                    {
                        Section = string.Empty,
                        Setting = setting.Name,
                        Value = setting.StringValue
                    });
                }
            }

            return configItems;

        }

        private void button_debug_StartFusion_Click(object sender, EventArgs e)
        {
            BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_START_REQUESTED, BitConverter.GetBytes((char)SOFTWARE.FUSION));
        }

        private void button_debug_StopFusion_Click(object sender, EventArgs e)
        {
            BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_STOP_REQUESTED, BitConverter.GetBytes((char)SOFTWARE.FUSION));
        }

        private void button_debug_StartRender_Click(object sender, EventArgs e)
        {
            BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_START_REQUESTED, BitConverter.GetBytes((char)SOFTWARE.RENDER));
        }

        private void button_debug_StopRender_Click(object sender, EventArgs e)
        {
            BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_STOP_REQUESTED, BitConverter.GetBytes((char)SOFTWARE.RENDER));
        }

        private void button_debug_TransmitConfig_Click(object sender, EventArgs e)
        {
            string configurationFileContents = SettingsManager.Instance.config.StringRepresentation;
            byte[] packet = new byte[configurationFileContents.Length + sizeof(int)];
            byte[] lengthInBytes = BitConverter.GetBytes(configurationFileContents.Length);
            Buffer.BlockCopy(lengthInBytes, 0, packet, 0, sizeof(int));
            Buffer.BlockCopy(Encoding.Default.GetBytes(configurationFileContents), 0, packet, sizeof(int), configurationFileContents.Length);
            BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.SYSTEM_CONFIGURATION_UPDATE, packet);
        }

        private void button_debug_CollectLogs_Click(object sender, EventArgs e)
        {
            string logCollectionFolderBase = SettingsManager.Instance.GetValueWithDefault("Debug", "LogCollectionFolder", Path.GetTempPath());
            string logCollectionFolder = "\\" + DateTime.Now.ToString("yyyy-MM-dd_H-mm-ss");
            // Create a new log collection folder
            // Check if the log collection folder in the config file exists
            if (!Directory.Exists(logCollectionFolderBase))
            {
                logCollectionFolder = Path.GetTempPath() + logCollectionFolder;
            }
            else
            {
                logCollectionFolder = logCollectionFolderBase + logCollectionFolder;
            }
            OutputHelper.OutputLog($"Requesting logs and collecting in {logCollectionFolder}");
            try
            {
                Directory.CreateDirectory(logCollectionFolder);
            }
            catch (Exception ex)
            {
                OutputHelper.OutputLog($"Couldn't create the directory {logCollectionFolder}: {ex.Message}.  Aborting log collection.");
                return;
            }
            // Update the bots with the new directory
            BotManager.Instance.fusionDaemonBot.SetLogCollectionPath(logCollectionFolder);
            BotManager.Instance.renderDaemonBot.SetLogCollectionPath(logCollectionFolder);
            foreach (StatusBot bot in BotManager.Instance.depthGenDaemonBots)
            {
                bot.SetLogCollectionPath(logCollectionFolder);
            }
            // Request logs from everyone
            BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.LOG_COLLECTION_REQUESTED);
            // Copy control panel logs there as well
            Directory.CreateDirectory(logCollectionFolder + "/ControlPanel");
            string logFilename = Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData) + "\\..\\LocalLow\\" + Application.CompanyName + "\\" + Application.ProductName + "\\Player.log";
            try
            {
                File.Copy(logFilename, logCollectionFolder + "/ControlPanel/Player.log");
            }
            catch (FileNotFoundException) { }

            logFilename = Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData) + "\\..\\LocalLow\\" + Application.CompanyName + "\\" + Application.ProductName + "\\Player-prev.log";

            try
            {
                File.Copy(logFilename, logCollectionFolder + "/ControlPanel/Player-prev.log");
            }
            catch (FileNotFoundException) { }

            // Copy any calibration logs as well
            string logPath = Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData) + SettingsManager.Instance.GetValueWithDefault("Calibration", "LogRelativePath", "\\..\\LocalLow\\Microsoft Research\\CalibrationLogs");
            if (Directory.Exists(logPath))
            {
                foreach (string file in Directory.GetFiles(logPath))
                {
                    try
                    {
                        File.Copy(file, logCollectionFolder + "/" + Path.GetFileName(file));
                    }
                    catch (Exception ex)
                    {
                        OutputHelper.OutputLog($"Couldn't copy {file} to {logCollectionFolder}/{file}.  Error: {ex.Message}");
                    }
                }
            }
        }

        private void button_debug_RequestVersions_Click(object sender, EventArgs e)
        {
            BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.BUILD_VERSION_REQUESTED);
        }

    }
}
