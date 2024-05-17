using System;

namespace Assets.Scripts.Viewer.State.Actions
{
    [Serializable]
    public enum ViewerActionType : int
    {
        SettingsButtonClicked = 0,
        ChangeViewState = 1,
        ViewCycleButtonClicked = 2,
        ToggleDualViewClicked = 3,
        SnapshotButtonClicked = 4,
        SnapshotCaptured = 5,
        HelpButtonClick = 6,
        ConnectButtonClick = 7,
        StartConnectionButtonClick = 8,
        DisconnectConnectionButtonClick = 9,
        ConnectionStatusChanged = 10,

        // This is the "Traditional" app mode setting
        TwoDModeButtonClick = 11,

        // This is the "Holoportation" app mode setting
        ThreeDModeButtonClick = 12,
        PanelClosed = 13,
        NavigationButtonClicked = 14,

        ChangeViewCamera = 15,
        ChangeViewDimension = 16,

        SetHoloportationHQ = 17,

        ToolSelected = 18,

        Initialize = 19,

        ServerQualityChangeRequested = 20,

        HQFrameReceived = 21,

        Color2DFramesReceived = 22,
        HoloportationFrameReceived = 23,
        HQToggleButtonClick = 24,

        ViewExportPrompt = 25,
        HQRequestCancelButtonClick = 26,
        ShowNotification = 27,
        StartTutorialButtonClick = 28,
        ServerQualityChanged = 29,
        PlayPauseButtonClick = 30,
        ReporterViewButtonClick = 31,
    }
}
