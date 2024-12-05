namespace ControlPanel
{
    partial class ControlPanel
    {
        /// <summary>
        ///  Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        ///  Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        ///  Required method for Designer support - do not modify
        ///  the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            DataGridViewCellStyle dataGridViewCellStyle1 = new DataGridViewCellStyle();
            tabControl1 = new TabControl();
            tabPage_broadcast = new TabPage();
            tableLayoutPanel1 = new TableLayoutPanel();
            panel_podStatus = new Panel();
            tableLayoutPanel3 = new TableLayoutPanel();
            dataGridView_broadcast_camera_daemons = new DataGridView();
            dataGridView_broadcast_camera_applications = new DataGridView();
            textBox_podStatusTitle = new TextBox();
            tableLayoutPanel_fusion_render_status = new TableLayoutPanel();
            tableLayoutPanel_fusion = new TableLayoutPanel();
            textBox_fusion_title = new TextBox();
            textBox_fusion_daemon_status_title = new TextBox();
            textBox_fusion_application_status_title = new TextBox();
            textBox_fusion_fps_title = new TextBox();
            textBox_fusion_daemon_status = new TextBox();
            textBox_fusion_application_status = new TextBox();
            textBox_fusion_fps = new TextBox();
            tableLayoutPanel2 = new TableLayoutPanel();
            textBox_render_title = new TextBox();
            textBox_render_daemon_status_title = new TextBox();
            textBox_render_application_status_title = new TextBox();
            textBox_render_fps_title = new TextBox();
            textBox_render_daemon_status = new TextBox();
            textBox_render_application_status = new TextBox();
            textBox_render_fps = new TextBox();
            button_start_session = new Button();
            statusStrip = new StatusStrip();
            toolStripStatusLabel_versionText = new ToolStripStatusLabel();
            toolStripStatusLabel_updateText = new ToolStripStatusLabel();
            tabPage_calibration = new TabPage();
            tabPage_config = new TabPage();
            tabPage_debug = new TabPage();
            tabPage_log = new TabPage();
            textBox_systemLog = new TextBox();
            pod_number = new DataGridViewTextBoxColumn();
            service_status = new DataGridViewTextBoxColumn();
            running_application = new DataGridViewTextBoxColumn();
            tabControl1.SuspendLayout();
            tabPage_broadcast.SuspendLayout();
            tableLayoutPanel1.SuspendLayout();
            panel_podStatus.SuspendLayout();
            tableLayoutPanel3.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)dataGridView_broadcast_camera_daemons).BeginInit();
            ((System.ComponentModel.ISupportInitialize)dataGridView_broadcast_camera_applications).BeginInit();
            tableLayoutPanel_fusion_render_status.SuspendLayout();
            tableLayoutPanel_fusion.SuspendLayout();
            tableLayoutPanel2.SuspendLayout();
            statusStrip.SuspendLayout();
            tabPage_log.SuspendLayout();
            SuspendLayout();
            // 
            // tabControl1
            // 
            tabControl1.Controls.Add(tabPage_broadcast);
            tabControl1.Controls.Add(tabPage_calibration);
            tabControl1.Controls.Add(tabPage_config);
            tabControl1.Controls.Add(tabPage_debug);
            tabControl1.Controls.Add(tabPage_log);
            tabControl1.Dock = DockStyle.Fill;
            tabControl1.Location = new Point(0, 0);
            tabControl1.Name = "tabControl1";
            tabControl1.SelectedIndex = 0;
            tabControl1.Size = new Size(1906, 820);
            tabControl1.TabIndex = 0;
            // 
            // tabPage_broadcast
            // 
            tabPage_broadcast.Controls.Add(tableLayoutPanel1);
            tabPage_broadcast.Controls.Add(statusStrip);
            tabPage_broadcast.Font = new Font("Segoe UI", 18F, FontStyle.Regular, GraphicsUnit.Point, 0);
            tabPage_broadcast.ForeColor = SystemColors.ControlText;
            tabPage_broadcast.Location = new Point(4, 34);
            tabPage_broadcast.Name = "tabPage_broadcast";
            tabPage_broadcast.Padding = new Padding(3);
            tabPage_broadcast.Size = new Size(1898, 782);
            tabPage_broadcast.TabIndex = 0;
            tabPage_broadcast.Text = "Broadcast";
            tabPage_broadcast.UseVisualStyleBackColor = true;
            // 
            // tableLayoutPanel1
            // 
            tableLayoutPanel1.ColumnCount = 1;
            tableLayoutPanel1.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
            tableLayoutPanel1.Controls.Add(panel_podStatus, 1, 0);
            tableLayoutPanel1.Controls.Add(textBox_podStatusTitle, 0, 0);
            tableLayoutPanel1.Controls.Add(tableLayoutPanel_fusion_render_status, 0, 2);
            tableLayoutPanel1.Controls.Add(button_start_session, 0, 3);
            tableLayoutPanel1.Dock = DockStyle.Fill;
            tableLayoutPanel1.Location = new Point(3, 3);
            tableLayoutPanel1.Name = "tableLayoutPanel1";
            tableLayoutPanel1.RowCount = 4;
            tableLayoutPanel1.RowStyles.Add(new RowStyle(SizeType.Absolute, 60F));
            tableLayoutPanel1.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
            tableLayoutPanel1.RowStyles.Add(new RowStyle());
            tableLayoutPanel1.RowStyles.Add(new RowStyle(SizeType.Absolute, 60F));
            tableLayoutPanel1.Size = new Size(1892, 744);
            tableLayoutPanel1.TabIndex = 2;
            // 
            // panel_podStatus
            // 
            panel_podStatus.Controls.Add(tableLayoutPanel3);
            panel_podStatus.Dock = DockStyle.Fill;
            panel_podStatus.Location = new Point(3, 63);
            panel_podStatus.Name = "panel_podStatus";
            panel_podStatus.Size = new Size(1886, 418);
            panel_podStatus.TabIndex = 2;
            // 
            // tableLayoutPanel3
            // 
            tableLayoutPanel3.ColumnCount = 2;
            tableLayoutPanel3.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50F));
            tableLayoutPanel3.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50F));
            tableLayoutPanel3.Controls.Add(dataGridView_broadcast_camera_daemons, 0, 0);
            tableLayoutPanel3.Controls.Add(dataGridView_broadcast_camera_applications, 1, 0);
            tableLayoutPanel3.Dock = DockStyle.Fill;
            tableLayoutPanel3.Location = new Point(0, 0);
            tableLayoutPanel3.Name = "tableLayoutPanel3";
            tableLayoutPanel3.RowCount = 1;
            tableLayoutPanel3.RowStyles.Add(new RowStyle(SizeType.Percent, 50F));
            tableLayoutPanel3.RowStyles.Add(new RowStyle(SizeType.Percent, 50F));
            tableLayoutPanel3.Size = new Size(1886, 418);
            tableLayoutPanel3.TabIndex = 2;
            // 
            // dataGridView_broadcast_camera_daemons
            // 
            dataGridView_broadcast_camera_daemons.AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.Fill;
            dataGridView_broadcast_camera_daemons.AutoSizeRowsMode = DataGridViewAutoSizeRowsMode.AllCells;
            dataGridView_broadcast_camera_daemons.ColumnHeadersHeightSizeMode = DataGridViewColumnHeadersHeightSizeMode.AutoSize;
            dataGridView_broadcast_camera_daemons.Dock = DockStyle.Fill;
            dataGridView_broadcast_camera_daemons.Location = new Point(3, 3);
            dataGridView_broadcast_camera_daemons.MinimumSize = new Size(0, 400);
            dataGridView_broadcast_camera_daemons.Name = "dataGridView_broadcast_camera_daemons";
            dataGridView_broadcast_camera_daemons.RowHeadersWidth = 62;
            dataGridViewCellStyle1.Font = new Font("Segoe UI", 10F);
            dataGridView_broadcast_camera_daemons.RowsDefaultCellStyle = dataGridViewCellStyle1;
            dataGridView_broadcast_camera_daemons.Size = new Size(937, 412);
            dataGridView_broadcast_camera_daemons.TabIndex = 1;
            // 
            // dataGridView_broadcast_camera_applications
            // 
            dataGridView_broadcast_camera_applications.ColumnHeadersHeightSizeMode = DataGridViewColumnHeadersHeightSizeMode.AutoSize;
            dataGridView_broadcast_camera_applications.Dock = DockStyle.Fill;
            dataGridView_broadcast_camera_applications.Location = new Point(946, 3);
            dataGridView_broadcast_camera_applications.Name = "dataGridView_broadcast_camera_applications";
            dataGridView_broadcast_camera_applications.RowHeadersWidth = 62;
            dataGridView_broadcast_camera_applications.Size = new Size(937, 412);
            dataGridView_broadcast_camera_applications.TabIndex = 2;
            // 
            // textBox_podStatusTitle
            // 
            textBox_podStatusTitle.Location = new Point(3, 3);
            textBox_podStatusTitle.Name = "textBox_podStatusTitle";
            textBox_podStatusTitle.Size = new Size(153, 55);
            textBox_podStatusTitle.TabIndex = 3;
            textBox_podStatusTitle.Text = "Cameras";
            // 
            // tableLayoutPanel_fusion_render_status
            // 
            tableLayoutPanel_fusion_render_status.ColumnCount = 2;
            tableLayoutPanel_fusion_render_status.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50F));
            tableLayoutPanel_fusion_render_status.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50F));
            tableLayoutPanel_fusion_render_status.Controls.Add(tableLayoutPanel_fusion, 0, 0);
            tableLayoutPanel_fusion_render_status.Controls.Add(tableLayoutPanel2, 1, 0);
            tableLayoutPanel_fusion_render_status.Dock = DockStyle.Fill;
            tableLayoutPanel_fusion_render_status.Location = new Point(3, 487);
            tableLayoutPanel_fusion_render_status.Name = "tableLayoutPanel_fusion_render_status";
            tableLayoutPanel_fusion_render_status.RowCount = 1;
            tableLayoutPanel_fusion_render_status.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
            tableLayoutPanel_fusion_render_status.RowStyles.Add(new RowStyle(SizeType.Absolute, 20F));
            tableLayoutPanel_fusion_render_status.Size = new Size(1886, 194);
            tableLayoutPanel_fusion_render_status.TabIndex = 5;
            // 
            // tableLayoutPanel_fusion
            // 
            tableLayoutPanel_fusion.ColumnCount = 2;
            tableLayoutPanel_fusion.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.3333321F));
            tableLayoutPanel_fusion.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.3333321F));
            tableLayoutPanel_fusion.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.3333321F));
            tableLayoutPanel_fusion.Controls.Add(textBox_fusion_title, 0, 0);
            tableLayoutPanel_fusion.Controls.Add(textBox_fusion_daemon_status_title, 0, 1);
            tableLayoutPanel_fusion.Controls.Add(textBox_fusion_application_status_title, 0, 2);
            tableLayoutPanel_fusion.Controls.Add(textBox_fusion_fps_title, 0, 3);
            tableLayoutPanel_fusion.Controls.Add(textBox_fusion_daemon_status, 1, 1);
            tableLayoutPanel_fusion.Controls.Add(textBox_fusion_application_status, 1, 2);
            tableLayoutPanel_fusion.Controls.Add(textBox_fusion_fps, 1, 3);
            tableLayoutPanel_fusion.Dock = DockStyle.Fill;
            tableLayoutPanel_fusion.Location = new Point(3, 3);
            tableLayoutPanel_fusion.Name = "tableLayoutPanel_fusion";
            tableLayoutPanel_fusion.RowCount = 4;
            tableLayoutPanel_fusion.RowStyles.Add(new RowStyle());
            tableLayoutPanel_fusion.RowStyles.Add(new RowStyle());
            tableLayoutPanel_fusion.RowStyles.Add(new RowStyle());
            tableLayoutPanel_fusion.RowStyles.Add(new RowStyle());
            tableLayoutPanel_fusion.Size = new Size(937, 188);
            tableLayoutPanel_fusion.TabIndex = 7;
            // 
            // textBox_fusion_title
            // 
            textBox_fusion_title.Font = new Font("Segoe UI", 18F);
            textBox_fusion_title.Location = new Point(3, 3);
            textBox_fusion_title.Name = "textBox_fusion_title";
            textBox_fusion_title.Size = new Size(153, 55);
            textBox_fusion_title.TabIndex = 7;
            textBox_fusion_title.Text = "Fusion";
            // 
            // textBox_fusion_daemon_status_title
            // 
            textBox_fusion_daemon_status_title.Font = new Font("Segoe UI", 10F);
            textBox_fusion_daemon_status_title.Location = new Point(3, 64);
            textBox_fusion_daemon_status_title.Name = "textBox_fusion_daemon_status_title";
            textBox_fusion_daemon_status_title.Size = new Size(240, 34);
            textBox_fusion_daemon_status_title.TabIndex = 8;
            textBox_fusion_daemon_status_title.Text = "Service Status";
            // 
            // textBox_fusion_application_status_title
            // 
            textBox_fusion_application_status_title.Font = new Font("Segoe UI", 10F);
            textBox_fusion_application_status_title.Location = new Point(3, 104);
            textBox_fusion_application_status_title.Name = "textBox_fusion_application_status_title";
            textBox_fusion_application_status_title.Size = new Size(223, 34);
            textBox_fusion_application_status_title.TabIndex = 9;
            textBox_fusion_application_status_title.Text = "Application Status";
            // 
            // textBox_fusion_fps_title
            // 
            textBox_fusion_fps_title.Font = new Font("Segoe UI", 10F);
            textBox_fusion_fps_title.Location = new Point(3, 144);
            textBox_fusion_fps_title.Name = "textBox_fusion_fps_title";
            textBox_fusion_fps_title.Size = new Size(223, 34);
            textBox_fusion_fps_title.TabIndex = 10;
            textBox_fusion_fps_title.Text = "FPS";
            // 
            // textBox_fusion_daemon_status
            // 
            textBox_fusion_daemon_status.Dock = DockStyle.Fill;
            textBox_fusion_daemon_status.Font = new Font("Segoe UI", 10F);
            textBox_fusion_daemon_status.Location = new Point(471, 64);
            textBox_fusion_daemon_status.Name = "textBox_fusion_daemon_status";
            textBox_fusion_daemon_status.Size = new Size(463, 34);
            textBox_fusion_daemon_status.TabIndex = 11;
            // 
            // textBox_fusion_application_status
            // 
            textBox_fusion_application_status.Dock = DockStyle.Fill;
            textBox_fusion_application_status.Font = new Font("Segoe UI", 10F);
            textBox_fusion_application_status.Location = new Point(471, 104);
            textBox_fusion_application_status.Name = "textBox_fusion_application_status";
            textBox_fusion_application_status.Size = new Size(463, 34);
            textBox_fusion_application_status.TabIndex = 12;
            // 
            // textBox_fusion_fps
            // 
            textBox_fusion_fps.Dock = DockStyle.Fill;
            textBox_fusion_fps.Font = new Font("Segoe UI", 10F);
            textBox_fusion_fps.Location = new Point(471, 144);
            textBox_fusion_fps.Name = "textBox_fusion_fps";
            textBox_fusion_fps.Size = new Size(463, 34);
            textBox_fusion_fps.TabIndex = 13;
            // 
            // tableLayoutPanel2
            // 
            tableLayoutPanel2.ColumnCount = 2;
            tableLayoutPanel2.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50F));
            tableLayoutPanel2.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50F));
            tableLayoutPanel2.Controls.Add(textBox_render_title, 0, 0);
            tableLayoutPanel2.Controls.Add(textBox_render_daemon_status_title, 0, 1);
            tableLayoutPanel2.Controls.Add(textBox_render_application_status_title, 0, 2);
            tableLayoutPanel2.Controls.Add(textBox_render_fps_title, 0, 3);
            tableLayoutPanel2.Controls.Add(textBox_render_daemon_status, 1, 1);
            tableLayoutPanel2.Controls.Add(textBox_render_application_status, 1, 2);
            tableLayoutPanel2.Controls.Add(textBox_render_fps, 1, 3);
            tableLayoutPanel2.Dock = DockStyle.Fill;
            tableLayoutPanel2.Location = new Point(946, 3);
            tableLayoutPanel2.Name = "tableLayoutPanel2";
            tableLayoutPanel2.RowCount = 4;
            tableLayoutPanel2.RowStyles.Add(new RowStyle());
            tableLayoutPanel2.RowStyles.Add(new RowStyle());
            tableLayoutPanel2.RowStyles.Add(new RowStyle());
            tableLayoutPanel2.RowStyles.Add(new RowStyle(SizeType.Absolute, 20F));
            tableLayoutPanel2.Size = new Size(937, 188);
            tableLayoutPanel2.TabIndex = 8;
            // 
            // textBox_render_title
            // 
            textBox_render_title.Font = new Font("Segoe UI", 18F);
            textBox_render_title.Location = new Point(3, 3);
            textBox_render_title.Name = "textBox_render_title";
            textBox_render_title.Size = new Size(153, 55);
            textBox_render_title.TabIndex = 6;
            textBox_render_title.Text = "Render";
            // 
            // textBox_render_daemon_status_title
            // 
            textBox_render_daemon_status_title.Font = new Font("Segoe UI", 10F);
            textBox_render_daemon_status_title.Location = new Point(3, 64);
            textBox_render_daemon_status_title.Name = "textBox_render_daemon_status_title";
            textBox_render_daemon_status_title.Size = new Size(240, 34);
            textBox_render_daemon_status_title.TabIndex = 9;
            textBox_render_daemon_status_title.Text = "Service Status";
            // 
            // textBox_render_application_status_title
            // 
            textBox_render_application_status_title.Font = new Font("Segoe UI", 10F);
            textBox_render_application_status_title.Location = new Point(3, 104);
            textBox_render_application_status_title.Name = "textBox_render_application_status_title";
            textBox_render_application_status_title.Size = new Size(223, 34);
            textBox_render_application_status_title.TabIndex = 10;
            textBox_render_application_status_title.Text = "Application Status";
            // 
            // textBox_render_fps_title
            // 
            textBox_render_fps_title.Font = new Font("Segoe UI", 10F);
            textBox_render_fps_title.Location = new Point(3, 144);
            textBox_render_fps_title.Name = "textBox_render_fps_title";
            textBox_render_fps_title.Size = new Size(223, 34);
            textBox_render_fps_title.TabIndex = 11;
            textBox_render_fps_title.Text = "FPS";
            // 
            // textBox_render_daemon_status
            // 
            textBox_render_daemon_status.Dock = DockStyle.Fill;
            textBox_render_daemon_status.Font = new Font("Segoe UI", 10F);
            textBox_render_daemon_status.Location = new Point(471, 64);
            textBox_render_daemon_status.Name = "textBox_render_daemon_status";
            textBox_render_daemon_status.Size = new Size(463, 34);
            textBox_render_daemon_status.TabIndex = 12;
            // 
            // textBox_render_application_status
            // 
            textBox_render_application_status.Dock = DockStyle.Fill;
            textBox_render_application_status.Font = new Font("Segoe UI", 10F);
            textBox_render_application_status.Location = new Point(471, 104);
            textBox_render_application_status.Name = "textBox_render_application_status";
            textBox_render_application_status.Size = new Size(463, 34);
            textBox_render_application_status.TabIndex = 13;
            // 
            // textBox_render_fps
            // 
            textBox_render_fps.Dock = DockStyle.Fill;
            textBox_render_fps.Font = new Font("Segoe UI", 10F);
            textBox_render_fps.Location = new Point(471, 144);
            textBox_render_fps.Name = "textBox_render_fps";
            textBox_render_fps.Size = new Size(463, 34);
            textBox_render_fps.TabIndex = 14;
            // 
            // button_start_session
            // 
            button_start_session.Anchor = AnchorStyles.None;
            button_start_session.Location = new Point(819, 687);
            button_start_session.Name = "button_start_session";
            button_start_session.Size = new Size(254, 54);
            button_start_session.TabIndex = 6;
            button_start_session.Text = "Start Session";
            button_start_session.UseVisualStyleBackColor = true;
            button_start_session.Click += button_start_session_Click;
            // 
            // statusStrip
            // 
            statusStrip.ImageScalingSize = new Size(24, 24);
            statusStrip.Items.AddRange(new ToolStripItem[] { toolStripStatusLabel_versionText, toolStripStatusLabel_updateText });
            statusStrip.Location = new Point(3, 747);
            statusStrip.Name = "statusStrip";
            statusStrip.Size = new Size(1892, 32);
            statusStrip.TabIndex = 0;
            statusStrip.Text = "Version:";
            // 
            // toolStripStatusLabel_versionText
            // 
            toolStripStatusLabel_versionText.Name = "toolStripStatusLabel_versionText";
            toolStripStatusLabel_versionText.Size = new Size(131, 25);
            toolStripStatusLabel_versionText.Text = "Version: 0.0.0.0";
            // 
            // toolStripStatusLabel_updateText
            // 
            toolStripStatusLabel_updateText.Name = "toolStripStatusLabel_updateText";
            toolStripStatusLabel_updateText.Size = new Size(60, 25);
            toolStripStatusLabel_updateText.Text = "Ready";
            // 
            // tabPage_calibration
            // 
            tabPage_calibration.Location = new Point(4, 34);
            tabPage_calibration.Name = "tabPage_calibration";
            tabPage_calibration.Padding = new Padding(3);
            tabPage_calibration.Size = new Size(1898, 782);
            tabPage_calibration.TabIndex = 1;
            tabPage_calibration.Text = "Calibration";
            tabPage_calibration.UseVisualStyleBackColor = true;
            // 
            // tabPage_config
            // 
            tabPage_config.Location = new Point(4, 34);
            tabPage_config.Name = "tabPage_config";
            tabPage_config.Size = new Size(1898, 782);
            tabPage_config.TabIndex = 2;
            tabPage_config.Text = "Configuration";
            tabPage_config.UseVisualStyleBackColor = true;
            // 
            // tabPage_debug
            // 
            tabPage_debug.Location = new Point(4, 34);
            tabPage_debug.Name = "tabPage_debug";
            tabPage_debug.Size = new Size(1898, 782);
            tabPage_debug.TabIndex = 3;
            tabPage_debug.Text = "Debug";
            tabPage_debug.UseVisualStyleBackColor = true;
            // 
            // tabPage_log
            // 
            tabPage_log.Controls.Add(textBox_systemLog);
            tabPage_log.Location = new Point(4, 34);
            tabPage_log.Name = "tabPage_log";
            tabPage_log.Size = new Size(1898, 782);
            tabPage_log.TabIndex = 4;
            tabPage_log.Text = "Log";
            tabPage_log.UseVisualStyleBackColor = true;
            // 
            // textBox_systemLog
            // 
            textBox_systemLog.Dock = DockStyle.Fill;
            textBox_systemLog.Location = new Point(0, 0);
            textBox_systemLog.Multiline = true;
            textBox_systemLog.Name = "textBox_systemLog";
            textBox_systemLog.ScrollBars = ScrollBars.Both;
            textBox_systemLog.Size = new Size(1898, 782);
            textBox_systemLog.TabIndex = 0;
            textBox_systemLog.WordWrap = false;
            // 
            // pod_number
            // 
            pod_number.FillWeight = 17.045454F;
            pod_number.HeaderText = "#";
            pod_number.MinimumWidth = 8;
            pod_number.Name = "pod_number";
            pod_number.ReadOnly = true;
            pod_number.Width = 150;
            // 
            // service_status
            // 
            service_status.FillWeight = 141.477417F;
            service_status.HeaderText = "Service Status";
            service_status.MinimumWidth = 8;
            service_status.Name = "service_status";
            service_status.ReadOnly = true;
            service_status.Width = 150;
            // 
            // running_application
            // 
            running_application.FillWeight = 141.477417F;
            running_application.HeaderText = "Running Application";
            running_application.MinimumWidth = 8;
            running_application.Name = "running_application";
            running_application.ReadOnly = true;
            running_application.Width = 150;
            // 
            // ControlPanel
            // 
            AutoScaleDimensions = new SizeF(10F, 25F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(1906, 820);
            Controls.Add(tabControl1);
            Name = "ControlPanel";
            Text = "3D Telemedicine Control Panel";
            tabControl1.ResumeLayout(false);
            tabPage_broadcast.ResumeLayout(false);
            tabPage_broadcast.PerformLayout();
            tableLayoutPanel1.ResumeLayout(false);
            tableLayoutPanel1.PerformLayout();
            panel_podStatus.ResumeLayout(false);
            tableLayoutPanel3.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)dataGridView_broadcast_camera_daemons).EndInit();
            ((System.ComponentModel.ISupportInitialize)dataGridView_broadcast_camera_applications).EndInit();
            tableLayoutPanel_fusion_render_status.ResumeLayout(false);
            tableLayoutPanel_fusion.ResumeLayout(false);
            tableLayoutPanel_fusion.PerformLayout();
            tableLayoutPanel2.ResumeLayout(false);
            tableLayoutPanel2.PerformLayout();
            statusStrip.ResumeLayout(false);
            statusStrip.PerformLayout();
            tabPage_log.ResumeLayout(false);
            tabPage_log.PerformLayout();
            ResumeLayout(false);
        }

        #endregion

        private TabControl tabControl1;
        private TabPage tabPage_broadcast;
        private TabPage tabPage_calibration;
        private StatusStrip statusStrip;
        private ToolStripStatusLabel toolStripStatusLabel_versionText;
        private TabPage tabPage_config;
        private TabPage tabPage_debug;
        private DataGridViewTextBoxColumn pod_number;
        private DataGridViewTextBoxColumn service_status;
        private DataGridViewTextBoxColumn running_application;
        private ToolStripStatusLabel toolStripStatusLabel_updateText;
        private TabPage tabPage_log;
        private TextBox textBox_systemLog;
        private TableLayoutPanel tableLayoutPanel1;
        private Panel panel_podStatus;
        private DataGridView dataGridView_broadcast_camera_daemons;
        private TextBox textBox_podStatusTitle;
        private TableLayoutPanel tableLayoutPanel_fusion_render_status;
        private Button button_start_session;
        private TextBox textBox_render_title;
        private TableLayoutPanel tableLayoutPanel_fusion;
        private TextBox textBox_fusion_title;
        private TextBox textBox_fusion_daemon_status_title;
        private TextBox textBox_fusion_application_status_title;
        private TextBox textBox_fusion_fps_title;
        private TableLayoutPanel tableLayoutPanel2;
        private TextBox textBox_render_daemon_status_title;
        private TextBox textBox_render_application_status_title;
        private TextBox textBox_render_fps_title;
        private TextBox textBox_fusion_daemon_status;
        private TextBox textBox_render_daemon_status;
        private TextBox textBox_fusion_application_status;
        private TextBox textBox_fusion_fps;
        private TextBox textBox_render_application_status;
        private TextBox textBox_render_fps;
        private TableLayoutPanel tableLayoutPanel3;
        private DataGridView dataGridView_broadcast_camera_applications;
    }
}
