using Assets.Scripts.Uniflux;
using Assets.Scripts.Viewer.Models;
using Assets.Scripts.Viewer.State.Actions;
using System;
using System.Linq;
using System.Text;

namespace Assets.Scripts.Viewer.State.Middleware
{
    public class ViewerReducers
    {
        public ViewerReducers(IUnifluxStore<ViewerActionType, ViewerState> store)
        {
            if (store == null)
            {
                throw new ArgumentNullException("store", "Store cannot be null");
            }

            store.ReducerMiddleware += OnReduce;
        }

        public void OnReduce(UnifluxAction<ViewerActionType, object> action, ViewerState state)
        {
            OnInitialize(action, state);
            OnPanelClickActionPerformed(action, state);
            OnConnectionStatusChangedAction(action, state);
            OnToggleDualViewAction(action, state);
            OnPanelClosed(action, state);
            OnModeChanged(action, state);
            OnViewCameraChanged(action, state);
            OnViewDimensionChanged(action, state);
            OnToolSelected(action, state);
            OnViewExportPrompt(action, state);
            OnHQRequestCancelButtonClicked(action, state);
            OnHQToggleButtonClicked(action, state);
            OnTutorialModeButtonClicked(action, state);
            OnServerQualityChanged(action, state);
            OnServerQualityChangeRequested(action, state);
            OnPlayPauseButtonClicked(action, state);
            OnReporterViewButtonClicked(action, state);
        }

        private void OnHQToggleButtonClicked(UnifluxAction<ViewerActionType, object> action, ViewerState state)
        {
            // We cannot do this if we are currently requesting an HQ image
            if (action.actionType == ViewerActionType.HQToggleButtonClick)
            {
                state.ClientInHQMode.Value = !state.ClientInHQMode.Value;
            }
        }

        private void OnReporterViewButtonClicked(UnifluxAction<ViewerActionType, object> action, ViewerState state)
        {
            if (action.actionType == ViewerActionType.ReporterViewButtonClick)
            {
                state.ReporterViewVisible.Value = !state.ReporterViewVisible.Value;
            }
        }

        private void OnPlayPauseButtonClicked(UnifluxAction<ViewerActionType, object> action, ViewerState state)
        {
            if (action.actionType == ViewerActionType.PlayPauseButtonClick)
            {
                state.Paused.Value = !state.Paused.Value;
            }
        }

        private void OnInitialize(UnifluxAction<ViewerActionType, object> action, ViewerState state)
        {
            if (action.actionType == ViewerActionType.Initialize)
            {
                if (state.AppMode.Value == ViewerMode.None)
                {
                    state.VisiblePanel.Value = ViewerPanel.Mode;
                }
                state.ReporterViewVisible.Value = false;
            }
        }

        private void OnTutorialModeButtonClicked(UnifluxAction<ViewerActionType, object> action, ViewerState state)
        {
            if (action.actionType == ViewerActionType.StartTutorialButtonClick)
            {
                state.InTutorialMode.Value = !state.InTutorialMode.Value;
            }
        }

        private static void OnViewExportPrompt(UnifluxAction<ViewerActionType, object> action, ViewerState state)
        {
            if (action.actionType == ViewerActionType.ViewExportPrompt)
            {
                state.VisiblePanel.Value = ViewerPanel.Export;
            }
        }

        private void OnServerQualityChanged(UnifluxAction<ViewerActionType, object> action, ViewerState state)
        {
            if (action.actionType == ViewerActionType.ServerQualityChanged)
            {
                bool quality = (bool)action.payload;
                state.ClientInHQMode.Value = quality;
                state.LoadingHQFrame.Value = false;

            }
        }

        private static void OnServerQualityChangeRequested(UnifluxAction<ViewerActionType, object> action, ViewerState state)
        {
            if (action.actionType == ViewerActionType.ServerQualityChangeRequested)
            {
                state.LoadingHQFrame.Value = (bool)action.payload;
            }
        }

        private void OnHQRequestCancelButtonClicked(UnifluxAction<ViewerActionType, object> action, ViewerState state)
        {
            if (action.actionType == ViewerActionType.HQRequestCancelButtonClick)
            {
                state.LoadingHQFrame.Value = false;
                state.ClientInHQMode.Value = false;
            }
        }

        private static void OnViewCameraChanged(UnifluxAction<ViewerActionType, object> action, ViewerState state)
        {
            if (action.actionType == ViewerActionType.ChangeViewCamera)
            {
                var data = (ViewCamera)action.payload;
                var viewId = data.ViewId;
                state.ViewActiveCamera.Value = new ViewCamera[] { data }
                    .Concat(state.ViewActiveCamera.Value.Where(n => n.ViewId != data.ViewId)).ToArray();

                // If we selected a new view, force the view to use the 2D dimension
                state.ViewActiveDimension.Value = new ViewDimension[] {
                    new ViewDimension(viewId, Dimension.TwoD),
                }.Concat(state.ViewActiveDimension.Value.Where(n => n.ViewId != data.ViewId)).ToArray();
            }
        }

        private static void OnViewDimensionChanged(UnifluxAction<ViewerActionType, object> action, ViewerState state)
        {
            if (action.actionType == ViewerActionType.ChangeViewDimension)
            {
                ViewDimension finalViewDimension = (ViewDimension)action.payload;

                // Traditional can only ever be 2D
                if (state.AppMode.Value == ViewerMode.Traditional)
                {
                    finalViewDimension = new ViewDimension() { 
                        ViewId = finalViewDimension.ViewId, 
                        Dimension = Dimension.ThreeD 
                    };
                }

                state.ViewActiveDimension.Value = new ViewDimension[] { finalViewDimension }.Concat(state.ViewActiveDimension.Value.Where(n => n.ViewId != finalViewDimension.ViewId)).ToArray();
            }
        }

        private static void OnToolSelected(UnifluxAction<ViewerActionType, object> action, ViewerState state)
        {
            if (action.actionType == ViewerActionType.ToolSelected)
            {
                var tool = (ViewerTool)action.payload;
                var currentTool = state.ActiveTool.Value;

                // If the user selected the same tool, deselect it
                if (currentTool == tool)
                {
                    state.ActiveTool.Value = ViewerTool.None;
                }
                else
                {
                    state.ActiveTool.Value = tool;
                }
            }
        }

        private static void OnModeChanged(UnifluxAction<ViewerActionType, object> action, ViewerState state)
        {
            if (action.actionType == ViewerActionType.ThreeDModeButtonClick)
            {
                state.AppMode.Value = ViewerMode.Holoportation;
                state.VisiblePanel.Value = ViewerPanel.None;
                state.DualView.Value = state.GetDualViewDefaultValue(ViewerMode.Holoportation);
            }
            else if (action.actionType == ViewerActionType.TwoDModeButtonClick)
            {
                state.AppMode.Value = ViewerMode.Traditional;
                state.VisiblePanel.Value = ViewerPanel.None;
                state.DualView.Value = state.GetDualViewDefaultValue(ViewerMode.Traditional);
            }
        }

        private static void OnPanelClosed(UnifluxAction<ViewerActionType, object> action, ViewerState state)
        {
            if (action.actionType == ViewerActionType.PanelClosed)
            {
                var closedPanel = (ViewerPanel)action.payload;
                if (closedPanel == state.VisiblePanel.Value)
                {
                    state.VisiblePanel.Value = ViewerPanel.None;
                }
                // Otherwise, action was emitted and it wasn't the "active" one, ignore it
            }
        }

        private static void OnToggleDualViewAction(UnifluxAction<ViewerActionType, object> action, ViewerState state)
        {
            if (action.actionType == ViewerActionType.ToggleDualViewClicked)
            {
                state.DualView.Value = !state.DualView.Value;

                // We are no longer in dual view mode, reset the viewer cameras/dimensions
                if (!state.DualView.Value)
                {
                    var view2CameraInitial = state.GetInitialViewCameras().First(n => n.ViewId == 2);
                    var view2DimensionInitial = state.GetInitialViewDimensions().First(n => n.ViewId == 2);
                    state.ViewActiveCamera.Value = 
                        state.ViewActiveCamera.Value.Where(n => n.ViewId != 2)
                            .Concat(new ViewCamera[] { view2CameraInitial })
                            .ToArray();
                    state.ViewActiveDimension.Value =
                        state.ViewActiveDimension.Value.Where(n => n.ViewId != 2)
                            .Concat(new ViewDimension[] { view2DimensionInitial })
                            .ToArray();
                }
            }
        }

        private static void OnPanelClickActionPerformed(UnifluxAction<ViewerActionType, object> action, ViewerState state)
        {
            ViewerPanel newPanel = ViewerPanel.None;
            if (action.actionType == ViewerActionType.SettingsButtonClicked)
            {
                newPanel = ViewerPanel.Settings;
            }
            else if (action.actionType == ViewerActionType.ConnectButtonClick)
            {
                newPanel = ViewerPanel.Connect;
            }
            else if (action.actionType == ViewerActionType.HelpButtonClick)
            {
                newPanel = ViewerPanel.Help;
            }
            else if (action.actionType == ViewerActionType.NavigationButtonClicked)
            {
                newPanel = ViewerPanel.Navigation;
            }

            // If we're switching to a known panel from above
            if (newPanel != ViewerPanel.None)
            {
                // Toggle if the same panel
                if (newPanel == state.VisiblePanel.Value)
                {
                    newPanel = ViewerPanel.None;
                }

                state.VisiblePanel.Value = newPanel;
            }
        }

        private static void OnConnectionStatusChangedAction(UnifluxAction<ViewerActionType, object> action, ViewerState state)
        {
            if (action.actionType == ViewerActionType.ConnectionStatusChanged)
            {
                ConnectionStatusUpdate update = (ConnectionStatusUpdate)action.payload;
                if (update.subSystem == ConnectionSubsystem.Holoportation3D)
                {
                    state.Holoport3DConnectionStatus.Value = update.status;
                    state.Holoport3DConnectionError.Value = update.error;
                } 
                else if (update.subSystem == ConnectionSubsystem.Color2D)
                {
                    state.Color2DConnectionStatus.Value = update.status;
                    state.Color2DConnectionError.Value = update.error;
                }

                state.OverallConnectionStatus.Value = ComputeOverallConnectionStatus(state);
                state.OverallConnectionError.Value = ComputeOverallConnectionError(state);

                if (state.OverallConnectionStatus.Value == ConnectionStatus.Disconnected)
                {
                    state.ConnectionInfo.Value = new InitConnectionData()
                    {
                        target = null
                    };

                    // When we disconnect, we are no longer in tutorial mode
                    state.InTutorialMode.Value = false;
                }
                else if (state.OverallConnectionStatus.Value == ConnectionStatus.Connected)
                {
                    // Reset the active cameras
                    state.ViewActiveCamera.Value = state.GetInitialViewCameras();
                    state.ViewActiveDimension.Value = state.GetInitialViewDimensions();
                    state.DualView.Value = state.GetDualViewDefaultValue(state.AppMode.Value);

                    state.VisiblePanel.Value = ViewerPanel.None;
                }
            }
            else if (action.actionType == ViewerActionType.StartConnectionButtonClick)
            {
                state.ConnectionInfo.Value = (InitConnectionData)action.payload;
            }
        }

        private static ConnectionStatus ComputeOverallConnectionStatus(ViewerState state)
        {
            ConnectionStatus holoStatus = state.Holoport3DConnectionStatus.Value;
            ConnectionStatus color2DStatus = state.Color2DConnectionStatus.Value;
            if (state.AppMode.Value == ViewerMode.Traditional)
            {
                return color2DStatus;
            }
            else
            {
                if (holoStatus == color2DStatus)
                {
                    return color2DStatus;
                }
                else if (holoStatus == ConnectionStatus.Error || color2DStatus == ConnectionStatus.Error)
                {
                    return ConnectionStatus.Error;
                }
                else
                {
                    return ConnectionStatus.Connecting;
                }
            }
        }

        private static string ComputeOverallConnectionError(ViewerState state)
        {
            if (state.AppMode.Value == ViewerMode.Traditional)
            {
                return state.Color2DConnectionError.Value;
            }
            else
            {
                StringBuilder sb = new StringBuilder();
                string error = state.Holoport3DConnectionError.Value;
                if (!string.IsNullOrWhiteSpace(error))
                {
                    sb.AppendLine($"3D Renderer Connection Error: {error}");
                }
                error = state.Color2DConnectionError.Value;
                if (!string.IsNullOrWhiteSpace(error))
                {
                    sb.AppendLine($"2D Renderer Connection Error: {error}");
                }
                return sb.Length > 0 ? sb.ToString() : null;
            }
        }
    }
}
