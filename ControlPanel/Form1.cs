

using AsyncIO;
using NLog;
using PeabodyNetworkingLibrary;
using System.ComponentModel;

namespace ControlPanel
{
    public partial class ControlPanel : Form
    {
        bool ContinueRunningSession = false;
        Thread BroadcastSessionStartupThread;
        public ControlPanel()
        {
            // Required to get NetMQ to work correctly
            ForceDotNet.Force();

            InitializeComponent();

            this.Load += ControlPanel_Loaded;
            this.FormClosing += ControlPanel_FormClosing;
        }

        private void ControlPanel_FormClosing(object? sender, FormClosingEventArgs e)
        {
            BotManager.Instance.OnDestroy();
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
            BindFusionDaemonBotToDataGridView();
            BindRenderDaemonBotToDataGridView();

        }

        private void BindRenderDaemonBotToDataGridView()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(BindRenderDaemonBotToDataGridView));
                return;
            }
            // bind the textBox_render_daemon_status to the status parameter of the botManager's render daemon bot
            textBox_render_daemon_status.DataBindings.Clear();
            textBox_render_daemon_status.DataBindings.Add("Text", BotManager.Instance.renderDaemonBot, "ComponentStatus");
            textBox_render_application_status.DataBindings.Add("Text", BotManager.Instance.renderDaemonBot, "RenderStatus");
        }
        private void BindRenderStatusBotToDataGridView()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(BindRenderStatusBotToDataGridView));
                return;
            }
            textBox_render_application_status.DataBindings.Clear();
            textBox_render_fps.DataBindings.Add("Text", BotManager.Instance.renderStatusBot, "FPS");
        }

        private void BindFusionDaemonBotToDataGridView()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(BindFusionDaemonBotToDataGridView));
                return;
            }
            textBox_fusion_daemon_status.DataBindings.Clear();
            textBox_fusion_daemon_status.DataBindings.Add("Text", BotManager.Instance.fusionDaemonBot, "ComponentStatus");
            textBox_fusion_application_status.DataBindings.Add("Text", BotManager.Instance.fusionDaemonBot, "FusionStatus");
        }
        private void BindFusionStatusBotToDataGridView()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(BindFusionStatusBotToDataGridView));
                return;
            }
            textBox_fusion_application_status.DataBindings.Clear();
            textBox_fusion_fps.DataBindings.Add("Text", BotManager.Instance.fusionStatusBot, "FPS");
        }

        internal void UpdateStatusText(string message)
        {
            toolStripStatusLabel_updateText.Text = message;
        }
        internal void UpdateLogText(string message)
        {
            textBox_systemLog.AppendText(message + Environment.NewLine);
            if (textBox_systemLog.Text.Length > 100000)
            {
                textBox_systemLog.Text = textBox_systemLog.Text.Substring(textBox_systemLog.Text.Length - 100000);
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
            dataGridView_broadcast_camera_applications.Visible = false;
            // Create a BindingSource
            BindingSource bindingSource = new BindingSource();

            // Set the DataSource of the BindingSource to the depthGenDaemonBots array
            bindingSource.DataSource = BotManager.Instance.depthGenDaemonBots;

            // Bind the BindingSource to the DataGridView
            dataGridView_broadcast_camera_daemons.DataSource = bindingSource;
            // Customize the DataGridView columns
            dataGridView_broadcast_camera_daemons.AutoGenerateColumns = false;
            dataGridView_broadcast_camera_daemons.Columns.Clear();

            // Add columns for specific properties of DaemonBot
            dataGridView_broadcast_camera_daemons.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "DepthGenID",
                HeaderText = "#"
            });
            dataGridView_broadcast_camera_daemons.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "VersionString",
                HeaderText = "Version"
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

            bindingSource = new BindingSource();
            bindingSource.DataSource = BotManager.Instance.depthGenStatusBots;
            dataGridView_broadcast_camera_applications.DataSource = bindingSource;
            dataGridView_broadcast_camera_applications.AutoGenerateColumns = false;
            dataGridView_broadcast_camera_applications.Columns.Clear();
            dataGridView_broadcast_camera_applications.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "DepthGenID",
                HeaderText = "#"
            });
            dataGridView_broadcast_camera_applications.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "VersionString",
                HeaderText = "Version"
            });
            dataGridView_broadcast_camera_applications.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "FPS",
                HeaderText = "FPS"
            });
            dataGridView_broadcast_camera_applications.Columns.Add(new DataGridViewTextBoxColumn
            {
                DataPropertyName = "Temperature",
                HeaderText = "Temp"
            });
            // refresh the data grid view
            dataGridView_broadcast_camera_daemons.Refresh();
            dataGridView_broadcast_camera_applications.Refresh();
        }

        public void Refresh()
        {
            dataGridView_broadcast_camera_daemons.Refresh();
            textBox_fusion_application_status.Refresh();
            textBox_fusion_daemon_status.Refresh();
            textBox_render_application_status.Refresh();
            textBox_render_daemon_status.Refresh();
            button_start_session.Refresh();
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
                button_start_session.Enabled = false;
                ContinueRunningSession = true;
                dataGridView_broadcast_camera_applications.Visible = true;

                BroadcastSessionStartupThread = new Thread(() =>
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
                    BotManager.Instance.BroadcastStartUntilAllDGStart(SOFTWARE.CAPTURE);

                    // Wait for all DG to be running
                    while(ContinueRunningSession && BotManager.Instance.GetBotsWithCaptureRunning() < dgNum)
                    {
                        Thread.Sleep(250);
                    }
                    /*  ENABLE WHEN READY
                    if (!ContinueRunningSession)
                        return;
                    // Start fusion
                    BotManager.Instance.fusionStatusBot.Start();
                    BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_START_REQUESTED, BitConverter.GetBytes((char)SOFTWARE.FUSION));

                    while(ContinueRunningSession && BotManager.Instance.fusionStatusBot.componentStatus.Status != Status.Running)
                    {
                        Thread.Sleep(250);
                    }

                    BotManager.Instance.renderStatusBot.Start();
                    BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_START_REQUESTED, BitConverter.GetBytes((char)SOFTWARE.RENDER));
                    */

                });
                BroadcastSessionStartupThread.Start();
            }
            else
            {
                ContinueRunningSession = false;
                dataGridView_broadcast_camera_applications.Visible = false;
                BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_STOP_REQUESTED, BitConverter.GetBytes((char)SOFTWARE.RENDER));
                BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_STOP_REQUESTED, BitConverter.GetBytes((char)SOFTWARE.FUSION));
                BotManager.Instance.BroadcastStopUntilAllDGStop(SOFTWARE.CAPTURE);
            }
        }

        internal void EnableStartSessionButton()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(EnableStartSessionButton));
                return;
            }
            button_start_session.BackColor = Color.LightGreen;
            button_start_session.Enabled = true;
            button_start_session.Text = "Start Session";
        }

        internal void DisableStartSessionButton()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(DisableStartSessionButton));
                return;
            }
            button_start_session.BackColor = Color.Red;
            button_start_session.Enabled = true;
            button_start_session.Text = "Stop Session";
        }
    }
}
