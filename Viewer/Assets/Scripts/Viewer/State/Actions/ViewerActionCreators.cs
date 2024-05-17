using Assets.Scripts.Uniflux;
using Assets.Scripts.Viewer.Common;
using Assets.Scripts.Viewer.Models;
using System;

namespace Assets.Scripts.Viewer.State.Actions
{
    public class ViewerActionCreators
    {
        public static UnifluxAction<ViewerActionType, object> viewCycleButtonClick()
        {
            return new UnifluxAction<ViewerActionType, object>(ViewerActionType.ViewCycleButtonClicked);
        }
        public static UnifluxAction<ViewerActionType, object> toggle3DModeButtonClick()
        {
            return new UnifluxAction<ViewerActionType, object>(ViewerActionType.ToggleDualViewClicked);
        }

        public static UnifluxAction<ViewerActionType, ViewDimension> setViewDimension(int viewId, Dimension dimension)
        {
            return new UnifluxAction<ViewerActionType, ViewDimension>(ViewerActionType.ChangeViewDimension, new ViewDimension(viewId, dimension));
        }
        public static UnifluxAction<ViewerActionType, ViewCamera> setViewCamera(int viewId, int cameraNum)
        {
            return new UnifluxAction<ViewerActionType, ViewCamera>(ViewerActionType.ChangeViewCamera, new ViewCamera(viewId, cameraNum));
        }
        public static UnifluxAction<ViewerActionType, bool> setHoloportationHQ(bool hq)
        {
            return new UnifluxAction<ViewerActionType, bool>(ViewerActionType.SetHoloportationHQ, hq);
        }

        public static UnifluxAction<ViewerActionType, bool> setFilePathPrompt(bool display)
        {
            return new UnifluxAction<ViewerActionType, bool>(ViewerActionType.ViewExportPrompt, display);
        }

        public static UnifluxAction<ViewerActionType, object> navigationButtonClicked()
        {
            return new UnifluxAction<ViewerActionType, object>(ViewerActionType.NavigationButtonClicked);
        }

        public static UnifluxAction<ViewerActionType, object> snapshotButtonClick()
        {
            return new UnifluxAction<ViewerActionType, object>(ViewerActionType.SnapshotButtonClicked);
        }
        public static UnifluxAction<ViewerActionType, SnapshotInfo> snapshotCaptured(SnapshotInfo info)
        {
            return new UnifluxAction<ViewerActionType, SnapshotInfo>(ViewerActionType.SnapshotCaptured, info);
        }

        public static UnifluxAction<ViewerActionType, object> helpButtonClick()
        {
            return new UnifluxAction<ViewerActionType, object>(ViewerActionType.HelpButtonClick);
        }

        public static UnifluxAction<ViewerActionType, object> connectButtonClick()
        {
            return new UnifluxAction<ViewerActionType, object>(ViewerActionType.ConnectButtonClick);
        }

        public static UnifluxAction<ViewerActionType, InitConnectionData> startConnectionButtonClick(InitConnectionData data)
        {
            return new UnifluxAction<ViewerActionType, InitConnectionData>(ViewerActionType.StartConnectionButtonClick, data);
        }
        public static UnifluxAction<ViewerActionType, object> startTutorialButtonClick()
        {
            return new UnifluxAction<ViewerActionType, object>(ViewerActionType.StartTutorialButtonClick);
        }

        public static UnifluxAction<ViewerActionType, ConnectionStatusUpdate> connectionStatusUpdate(ConnectionStatusUpdate update)
        {
            return new UnifluxAction<ViewerActionType, ConnectionStatusUpdate>(ViewerActionType.ConnectionStatusChanged, update);
        }

        public static UnifluxAction<ViewerActionType, ViewerPanel> panelClosed(ViewerPanel panel)
        {
            return new UnifluxAction<ViewerActionType, ViewerPanel>(ViewerActionType.PanelClosed, panel);
        }

        public static UnifluxAction<ViewerActionType, ViewerTool> activeToolSelected(ViewerTool tool)
        {
            return new UnifluxAction<ViewerActionType, ViewerTool>(ViewerActionType.ToolSelected, tool);
        }

        public static UnifluxAction<ViewerActionType, object> disconnectButtonClick()
        {
            return new UnifluxAction<ViewerActionType, object>(ViewerActionType.DisconnectConnectionButtonClick);
        }

        public static UnifluxAction<ViewerActionType, object> serverQualityChangeRequested(bool hq)
        {
            return new UnifluxAction<ViewerActionType, object>(ViewerActionType.ServerQualityChangeRequested, hq);
        }

        public static UnifluxAction<ViewerActionType, object> hqFrameReceived()
        {
            return new UnifluxAction<ViewerActionType, object>(ViewerActionType.HQFrameReceived);
        }

        public static UnifluxAction<ViewerActionType, object> serverQualityChanged(bool hq)
        {
            return new UnifluxAction<ViewerActionType, object>(ViewerActionType.ServerQualityChanged, hq);
        }

        public static UnifluxAction<ViewerActionType, object> color2dFramesReceived()
        {
            return new UnifluxAction<ViewerActionType, object>(ViewerActionType.Color2DFramesReceived, null, false);
        }

        public static UnifluxAction<ViewerActionType, object> holoportationFrameReceived()
        {
            return new UnifluxAction<ViewerActionType, object>(ViewerActionType.HoloportationFrameReceived, null, false);
        }

        public static UnifluxAction<ViewerActionType, object> hqToggleButtonClick()
        {
            return new UnifluxAction<ViewerActionType, object>(ViewerActionType.HQToggleButtonClick);
        }

        public static UnifluxAction<ViewerActionType, object> hqCancelRequestButtonClick()
        {
            return new UnifluxAction<ViewerActionType, object>(ViewerActionType.HQRequestCancelButtonClick);
        }

        public static UnifluxAction<ViewerActionType, object> showNotification(string title, string contents, Action callback)
        {
            return new UnifluxAction<ViewerActionType, object>(ViewerActionType.ShowNotification, new Toast()
            {
                Title = title,
                Contents = contents,
                Callback = callback
            });
        }
    }
}
