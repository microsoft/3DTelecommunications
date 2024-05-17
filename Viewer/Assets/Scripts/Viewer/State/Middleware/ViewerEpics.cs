using Assets.Scripts.Viewer.State.Actions;
using System;
using UnityEngine;
using Assets.Scripts.Common.Extensions;
using Assets.Scripts.Viewer.Common;
using Assets.Scripts.Viewer.Models;
using System.IO;
using System.Threading;
using Assets.Scripts.Uniflux;
using Assets.Scripts.Viewer.Behaviors;
using System.Collections.Generic;
using HoloportationDecoders.Color2D;
using HoloportationDecoders.Holoportation;

namespace Assets.Scripts.Viewer.State.Middleware
{
    public class ViewerEpics
    {
        private SynchronizationContext context;
        private Color2DDecoder color2DDecoder;
        private HoloportDecoder holoportDecoder;
        private HoloportRawFrameViewer hqHoloportViewer;
        private IUnifluxStore<ViewerActionType, ViewerState> store;
        private CursorManager.CursorInfo zoomCursor;
        private CursorManager.CursorInfo panCursor;
        private CursorManager.CursorInfo orbitCursor;
        private CursorManager.CursorInfo appliedCursor;
        private Toaster toaster;
        private SettingsManager settings;

        public ViewerEpics(
            // TODO: Gross, way too many deps
            IUnifluxStore<ViewerActionType, ViewerState> store,
            ViewerManager manager,
            SettingsManager settings
        ): this(store, manager.color2DDecoder, manager.holoportDecoder, manager.hqHoloportViewer, manager.Toaster, manager.zoomCursor, manager.panCursor, manager.orbitCursor, settings)
        { }
        public ViewerEpics(
            // TODO: Gross, way too many deps
            IUnifluxStore<ViewerActionType, ViewerState> store,
            Color2DDecoder color2DDecoder,
            HoloportDecoder holoportDecoder,
            HoloportRawFrameViewer hqHoloportViewer,
            Toaster toaster,
            Texture2D zoomCursorTexture,
            Texture2D panCursorTexture,
            Texture2D orbitCursorTexture,
            SettingsManager settings
        )
        {
            context = SynchronizationContext.Current;
            this.store = store;
            this.color2DDecoder = color2DDecoder;
            this.holoportDecoder = holoportDecoder;
            this.hqHoloportViewer = hqHoloportViewer;
            this.toaster = toaster;
            this.settings = settings;
            this.zoomCursor =
                new CursorManager.CursorInfo(
                    zoomCursorTexture.ApplyTint(settings.ZoomToolColor),
                    new Vector2(zoomCursorTexture.width / 2f, zoomCursorTexture.height / 2f)
                );
            this.panCursor =
                new CursorManager.CursorInfo(
                    panCursorTexture.ApplyTint(settings.PanToolColor),
                    new Vector2(panCursorTexture.width / 2f, panCursorTexture.height / 2f)
                );
            this.orbitCursor =
                new CursorManager.CursorInfo(
                    orbitCursorTexture.ApplyTint(settings.OrbitToolColor),
                    new Vector2(orbitCursorTexture.width / 2f, orbitCursorTexture.height / 2f)
                );

            ConnectionSubsystem system = ConnectionSubsystem.Color2D;
            color2DDecoder.Connecting += DefineConnectionStatusHandler(store, system, ConnectionStatus.Connecting);
            color2DDecoder.Connected += DefineConnectionStatusHandler(store, system, ConnectionStatus.Connected);
            color2DDecoder.Disconnected += DefineConnectionStatusHandler(store, system, ConnectionStatus.Disconnected);

            system = ConnectionSubsystem.Holoportation3D;
            holoportDecoder.Connecting += DefineConnectionStatusHandler(store, system, ConnectionStatus.Connecting);
            holoportDecoder.Connected += DefineConnectionStatusHandler(store, system, ConnectionStatus.Connected);
            holoportDecoder.Disconnected += DefineConnectionStatusHandler(store, system, ConnectionStatus.Disconnected);

            color2DDecoder.HQ3DFrameReady += (frame) => {
                SafeInvoke(() =>
                {
                    store.Dispatch(ViewerActionCreators.hqFrameReceived());
                }, context);
            };

            color2DDecoder.ServerQualityChanged += (value) =>
            {
                SafeInvoke(() =>
                {
                    store.Dispatch(ViewerActionCreators.serverQualityChanged(value));
                }, context);
            };

            color2DDecoder.ServerQualityChangeRequested += (value) =>
            {
                SafeInvoke(() =>
                {
                    store.Dispatch(ViewerActionCreators.serverQualityChangeRequested(value));
                }, context);
            };

            color2DDecoder.FrameReceived2D += () =>
            {
                SafeInvoke(() =>
                {
                    store.Dispatch(ViewerActionCreators.color2dFramesReceived());
                }, context);
            };

            holoportDecoder.FrameDecoded += (HoloportDataFrame _) =>
            {
                SafeInvoke(() =>
                {
                    store.Dispatch(ViewerActionCreators.holoportationFrameReceived());
                }, context);
            };

            store.DispatchMiddleware += OnDispatch;
            store.StateChangeMiddleware += OnStateChanged;
        }

        /// <summary>
        /// State change handler for uniflux
        /// </summary>
        /// <param name="newState">The new state</param>
        /// <param name="oldState">The old state</param>
        /// <param name="changes">The list of changes to the state</param>
        public void OnStateChanged(ViewerState newState, ViewerState oldState, IReadOnlyDictionary<string, (object, object)> changes)
        {
            var isConnected = newState.OverallConnectionStatus.Value == ConnectionStatus.Connected;

            // Make the server quality match the client
            if (newState.ClientInHQMode.Value != oldState.ClientInHQMode.Value)
            {
                RequestServerQuality(newState.ClientInHQMode.Value, isConnected);
            }

            bool isPaused = newState.Paused.Value;
            if (isPaused != oldState.Paused.Value)
            {
                PauseDecoders(isPaused);
            }

            // Show the 3D models based on whether or not the server is in HQ mode
            SetModelVisibility(color2DDecoder.ServerInHQMode);
        }

        /// <summary>
        /// Dispatch handler for Uniflux
        /// </summary>
        /// <param name="action">The action that occurred</param>
        public void OnDispatch(UnifluxAction<ViewerActionType, object> action)
        {
            var state = store.GetState();
            var isConnected = state.OverallConnectionStatus.Value == ConnectionStatus.Connected;
            if (action.actionType == ViewerActionType.ConnectButtonClick)
            {
                store.Dispatch(ViewerActionCreators.startConnectionButtonClick(new InitConnectionData()
                {
                    target = "Glasgow"
                }));
            }
            else if (action.actionType == ViewerActionType.ShowNotification)
            {
                toaster.ShowToast((Toast)action.payload);
            }
            else if (action.actionType == ViewerActionType.Initialize)
            {
                // Initialize the models to match the mode
                SetModelVisibility(false);
            }

            // Server quality changed
            else if (action.actionType == ViewerActionType.ServerQualityChanged)
            {
                bool serverQuality = (bool)action.payload;
                SetModelVisibility(serverQuality);

                // We didn't request the quality change, but we received a quality change response
                // and it is in HQ mode
                if (!color2DDecoder.RequestingQualityChange)
                {
                    DispatchServerHQToggleNotification(serverQuality);
                }
            }
            else if (action.actionType == ViewerActionType.SnapshotButtonClicked)
            {
                CaptureSnapshot(state.Paused.Value);
            }

            else if (action.actionType == ViewerActionType.ToolSelected)
            {
                CursorManager.CursorInfo newCursor = null;
                var tool = state.ActiveTool.Value;
                if (tool == ViewerTool.Zoom)
                {
                    newCursor = zoomCursor;
                }
                else if (tool == ViewerTool.Pan)
                {
                    newCursor = panCursor;
                }
                else if (tool == ViewerTool.Orbit)
                {
                    newCursor = orbitCursor;
                }

                // Remove the old one
                CursorManager.RemoveCursor(appliedCursor);
                appliedCursor = newCursor;
                CursorManager.AddCursor(newCursor);
            }
            else if (action.actionType == ViewerActionType.SnapshotCaptured)
            {
                string path = ((SnapshotInfo)action.payload).filename;
                store.Dispatch(ViewerActionCreators.showNotification("Snapshot Captured", $"Saved to {path}", () => Application.OpenURL(path)));
            }
            else if (action.actionType == ViewerActionType.StartConnectionButtonClick || action.actionType == ViewerActionType.StartTutorialButtonClick)
            {
                HoloportDecoder holoportDecoder = this.holoportDecoder;
                Color2DDecoder color2DDecoder = this.color2DDecoder;
                if (color2DDecoder.IsConnecting || color2DDecoder.IsConnected)
                {
                    color2DDecoder.Disconnect();
                    if (state.AppMode.Value == ViewerMode.Holoportation)
                    {
                        holoportDecoder.ClientDisconnect();
                    }
                }
                else
                {
                    bool isLocal = action.actionType == ViewerActionType.StartTutorialButtonClick;
                    color2DDecoder.Connect(isLocal ? settings.TutorialColor2DFolder : null);
                    if (state.AppMode.Value == ViewerMode.Holoportation)
                    {
                        holoportDecoder.ClientConnect(isLocal ? settings.TutorialHoloportationFolder : null);
                    }
                }
            }
            else if (action.actionType == ViewerActionType.DisconnectConnectionButtonClick)
            {
                HoloportDecoder holoportDecoder = this.holoportDecoder;
                Color2DDecoder color2DDecoder = this.color2DDecoder;
                if (color2DDecoder.IsConnecting || color2DDecoder.IsConnected)
                {
                    color2DDecoder.Disconnect();
                    if (state.AppMode.Value == ViewerMode.Holoportation)
                    {
                        holoportDecoder.ClientDisconnect();
                    }
                }
            }
        }

        /// <summary>
        /// Captures a snapshot of the system
        /// <param name="isSystemPaused">If the system is currently paused</param>
        /// </summary>
        protected virtual void CaptureSnapshot(bool isSystemPaused)
        {
            // Pause the stream
            PauseDecoders(true);

            string snapshotsDirectory = Utils.EnsureDirectory(
            Path.Combine(
                settings.SnapshotsDirectory,
                DateTime.Now.ToString("yyyy-MM-dd--HH-mm-ss")
            ));

            if (settings.CaptureLQ3DModel)
            {
                holoportDecoder.ExportModelOBJ(snapshotsDirectory, "Patient3DModel-LowQuality");
            }

            if (settings.CaptureHQ3DModel)
            {
                hqHoloportViewer.ExportModelOBJ(snapshotsDirectory, "Patient3DModel-HighQuality");
            }

            if (settings.CaptureScreenshot)
            {
                CaptureScreen(snapshotsDirectory);
            }

            if (settings.CaptureViews)
            {
                GameObject view1 = GameObject.FindGameObjectWithTag(ViewTags.GetTags(ViewTagsTypes.view1));
                GameObject view2 = GameObject.FindGameObjectWithTag(ViewTags.GetTags(ViewTagsTypes.view2));
                CaptureView(view1, snapshotsDirectory, "LeftView");
                CaptureView(view2, snapshotsDirectory, "RightView");
            }

            if (settings.CaptureAllRawFrames)
            {
                //flip image upside down since encoded incoming jpeg is upside down. 
                CaptureRawFrames(snapshotsDirectory, true);
            }

            DispatchSnapshotCapturedDelayed(snapshotsDirectory);

            // Restore the decoders to their previous state
            PauseDecoders(isSystemPaused);
        }

        /// <summary>
        /// Captures all the color2d raw frames of the system
        /// </summary>
        /// <param name="directory">The directory to save the frames to</param>
        /// <returns></returns>

        protected virtual bool CaptureRawFrames(string directory, bool flipUpsideDown = false)
        {
            // Capture all of the raw images
            int cameraCount = color2DDecoder.GetTotalColorCounts();
            for (int i = 0; i < cameraCount; i++)
            {
                using (ICameraFeed feed = color2DDecoder.CreateCameraFeed(i))
                {
                    string filename = Path.Combine(directory, $"Camera_{i}_raw.jpg");
                    File.WriteAllBytes(filename, feed.RawImageJPEG);
                }
            }
            return true;
        }

        /// <summary>
        /// Captures the given view to a png and saves it to the given directory/name
        /// </summary>
        /// <param name="view">The view to capture</param>
        /// <param name="directory">The directory to save the capture to</param>
        /// <param name="name">The output file name</param>
        /// <returns></returns>
        protected virtual bool CaptureView(GameObject view, string directory, string name)
        {
            if (view != null && view.activeSelf)
            {
                var behavior = view.GetComponent<ViewBehavior>();
                if (behavior != null && behavior.isActiveAndEnabled)
                {
                    byte[] bytes = behavior.Snapshot();
                    if (bytes != null)
                    {
                        string filename = Path.Combine(directory, $"{name}.png");
                        File.WriteAllBytes(filename, bytes);
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            }

            // We will return true if it is disabled.
            return true;
        }

        /// <summary>
        /// Captures the current screen
        /// </summary>
        /// <param name="directory">The directory to save the screenshot to</param>
        protected virtual void CaptureScreen(string directory)
        {
            // Capture the screen
            ScreenCapture.CaptureScreenshot(Path.Combine(directory, "Screen.png"));
        }

        /// <summary>
        /// Dispatches a snapshot captured action
        /// </summary>
        /// <param name="directory">The directory of the snapshot</param>
        protected virtual void DispatchSnapshotCapturedDelayed(string directory)
        {
            // Capture the 3D model
            System.Threading.Tasks.Task.Run(() =>
            {
                // Wait for the screenshot to finish
                Thread.Sleep(1000);

                context.Post((ignore) =>
                {
                    store.Dispatch(ViewerActionCreators.snapshotCaptured(new SnapshotInfo() { filename = Path.GetFullPath(directory) }));
                }, null);
            });
        }

        /// <summary>
        /// Dispatches a snapshot notification to uniflux
        /// </summary>
        /// <param name="directory">The directory of the snapshot</param>
        protected virtual void DispatchSnapshotCapturedNotification(string directory)
        {
            store.Dispatch(ViewerActionCreators.showNotification("Snapshot Captured", $"Saved to {directory}", () => Application.OpenURL(directory)));
        }

        /// <summary>
        /// Dispatches a notification notifying the user that the server changed the quality 
        /// </summary>
        /// <param name="serverHQ"></param>
        protected virtual void DispatchServerHQToggleNotification(bool serverHQ)
        {
            store.Dispatch(ViewerActionCreators.showNotification(
                "Server quality changed",
                "The server quality was changed externally",
                null
            ));
        }

        /// <summary>
        /// Requests the given server quality
        /// </summary>
        /// <param name="hq">If true, high quality is requested</param>
        /// <param name="isConnected">If true, the system is currently connected</param>
        protected virtual void RequestServerQuality(bool hq, bool isConnected)
        {
            if (color2DDecoder.ServerInHQMode != hq && isConnected)
            {
                color2DDecoder.RequestToggleFusionQuality(hq);
            }
        }

        /// <summary>
        /// Pauses/Resumes the decoders
        /// </summary>
        /// <param name="paused">If true, the decoders will be paused, otherwise resumed</param>
        protected virtual void PauseDecoders(bool paused)
        {
            if (paused)
            {
                color2DDecoder.Pause();
                holoportDecoder.Pause();
                hqHoloportViewer.Pause();
            }
            else
            {
                color2DDecoder.Resume();
                holoportDecoder.Resume();
                hqHoloportViewer.Resume();
            }
        }

        /// <summary>
        /// Switches the visible patient models between the hq version and the lq version
        /// </summary>
        /// <param name="hq">If true, the high quality patient model should be visible</param>
        protected virtual void SetModelVisibility(bool hq)
        {
            holoportDecoder.IsModelVisible = !hq;
            hqHoloportViewer.IsModelVisible = hq;
        }

        private Action DefineConnectionStatusHandler(IUnifluxStore<ViewerActionType, ViewerState> store, ConnectionSubsystem system, ConnectionStatus status)
        {
            return InContext(() =>
            {
                store.Dispatch(ViewerActionCreators.connectionStatusUpdate(new ConnectionStatusUpdate(status, system)));
            }, context);
        }

        private Action InContext(Action a, SynchronizationContext context)
        {
            return () =>
            {
                SafeInvoke(a, context);
            };
        } 

        private void SafeInvoke(Action a, SynchronizationContext context)
        {
            if (SynchronizationContext.Current == context)
            {
                a?.Invoke();
            } 
            else
            {
                context.Post(ignore =>
                {
                    a?.Invoke();
                }, null);
            }
        }
    }
}
