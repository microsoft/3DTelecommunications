using NUnit.Framework;
using UnityEngine;
using Assets.Scripts.Viewer.State.Middleware;
using Assets.Scripts.Viewer.State;
using Assets.Scripts.Viewer.Models;
using Assets.Scripts.Viewer;
using Moq;
using Assets.Scripts.Viewer.Common;
using System.Linq;
using Assets.Scripts.Viewer.State.Actions;
using HoloportationDecoders.Color2D;
using HoloportationDecoders.Holoportation;
using System.IO;
using System;

namespace Tests
{
    public class ViewerEpicsTests
    {
        private string tempDirectory;

        [SetUp]
        public void Setup()
        {
            tempDirectory = string.Concat(Application.temporaryCachePath, "/tests");
        }

        [TearDown]
        public void TearDown()
        {
            if (Directory.Exists(tempDirectory))
            {
                Directory.Delete(tempDirectory, true);
            }
        }

        [Test]
        public void Constructor_Basic_DoesntCrash()
        {
            CreateInstance();
        }

        [Test]
        public void InHQMode_SetToTrue_ToggleHQSentToServer()
        {
            var instance = CreateInstance();
            var oldState = CreateDefaultState();
            var newState = CreateDefaultState();

            // We need to be connected for this to work
            oldState.OverallConnectionStatus.Value = ConnectionStatus.Connected;
            newState.OverallConnectionStatus.Value = ConnectionStatus.Connected;

            // Changing "ClientInHQMode" from false to true
            oldState.ClientInHQMode.Value = false;
            newState.ClientInHQMode.Value = true;

            instance.color2DDecoder.Setup(n => n.ServerInHQMode).Returns(false);
            instance.color2DDecoder
                .Setup(n => n.RequestToggleFusionQuality(false));

            instance.store.RunStateChange(
                newState,
                oldState,
                new string[1]
                    .ToDictionary(n => "ClientInHQMode", n => ((object)false, (object)true))
            );

            instance.color2DDecoder.Verify(n => n.RequestToggleFusionQuality(true), Times.Once);
        }

        [Test]
        public void InHQMode_SetToFalse_ToggleHQSentToServer()
        {
            var instance = CreateInstance();
            var oldState = CreateDefaultState();
            var newState = CreateDefaultState();

            // We need to be connected for this to work
            oldState.OverallConnectionStatus.Value = ConnectionStatus.Connected;
            newState.OverallConnectionStatus.Value = ConnectionStatus.Connected;

            // Changing "ClientInHQMode" from true to false
            oldState.ClientInHQMode.Value = true;
            newState.ClientInHQMode.Value = false;

            instance.color2DDecoder.Setup(n => n.ServerInHQMode).Returns(true);
            instance.color2DDecoder
                .Setup(n => n.RequestToggleFusionQuality(false));

            instance.store.RunStateChange(
                newState,
                oldState,
                new string[1]
                    .ToDictionary(n => "ClientInHQMode", n => ((object)true, (object)false))
            );

            instance.color2DDecoder.Verify(n => n.RequestToggleFusionQuality(false), Times.Once);
        }

        [Test]
        public void StartTutorialButtonClicked_DecodersConnected()
        {
            var instance = CreateInstance();
            var newState = CreateDefaultState();

            // Setup the mock store to return our fake state
            instance.store.SetMockState(newState);

            // Setup mocks
            instance.color2DDecoder
                .Setup(n => n.Connect(null));

            instance.holoportDecoder
                .Setup(n => n.ClientConnect(null));

            // Setup settings to return the correct mocked folders
            instance.settingsManager.Setup(n => n.TutorialColor2DFolder).Returns("SOME_LOCAL_FOLDER_COLOR2D");
            instance.settingsManager.Setup(n => n.TutorialHoloportationFolder).Returns("SOME_LOCAL_FOLDER_HOLO");

            // Trigger the tutorial button click
            instance.store.RunDispatch(ViewerActionType.StartTutorialButtonClick, null);

            // Make sure connects get called with the correct mocked folders
            instance.color2DDecoder.Verify(n => n.Connect("SOME_LOCAL_FOLDER_COLOR2D"), Times.Once);
            instance.holoportDecoder.Verify(n => n.ClientConnect("SOME_LOCAL_FOLDER_HOLO"), Times.Once);
        }

        [Test]
        public void StartConnectionButtonClicked_DecodersConnected()
        {
            var instance = CreateInstance();
            var newState = CreateDefaultState();

            // Setup the mock store to return our fake state
            instance.store.SetMockState(newState);

            // Setup mocks
            instance.color2DDecoder
                .Setup(n => n.Connect(null));

            instance.holoportDecoder
                .Setup(n => n.ClientConnect(null));

            // Trigger the tutorial button click
            instance.store.RunDispatch(ViewerActionType.StartConnectionButtonClick, null);

            // Verify the mocks actually get called
            instance.color2DDecoder.Verify(n => n.Connect(null), Times.Once);
            instance.holoportDecoder.Verify(n => n.ClientConnect(null), Times.Once);
        }

        [Test]
        public void Color2DFrameReceievedFromServer_DispatchesColor2DFrameReceieved()
        {
            var instance = CreateInstance();

            // Make sure no actions were raised before
            Assert.AreEqual(instance.store.lastActionType, null);

            // Raise the event
            instance.color2DDecoder.Raise(n => n.FrameReceived2D += null);

            // Make sure the frame received events is raised
            Assert.AreEqual(instance.store.lastActionType, ViewerActionType.Color2DFramesReceived);
        }

        [Test]
        public void HQFrameReceievedFromServer_DispatchesHQFrameReceieved()
        {
            var instance = CreateInstance();

            // Make sure no actions were raised before
            Assert.AreEqual(instance.store.lastActionType, null);

            // Raise the event
            instance.color2DDecoder.Raise(n => n.HQ3DFrameReady += null, (object)null);

            // Make sure the frame received events is raised
            Assert.AreEqual(instance.store.lastActionType, ViewerActionType.HQFrameReceived);
        }

        [Test]
        // https://dev.azure.com/msrp/PeabodyMain/_workitems/edit/9349/
        public void HQModeTurnedOnClient_ServerAlreadyInHQMode_NoToggleHQRequestSent()
        {
            var instance = CreateInstance();
            var oldState = CreateDefaultState();
            var newState = CreateDefaultState();

            // User turned on HQ mode (or toggled on automatically with capture)
            oldState.ClientInHQMode.Value = false;
            newState.ClientInHQMode.Value = true;

            // Pretend we are connected
            oldState.OverallConnectionStatus.Value = ConnectionStatus.Connected;
            newState.OverallConnectionStatus.Value = ConnectionStatus.Connected;

            instance.color2DDecoder.Setup(n => n.ServerInHQMode).Returns(true);
            instance.color2DDecoder
                .Setup(n => n.RequestToggleFusionQuality());

            instance.store.RunStateChange(
                newState,
                oldState,
                new string[1]
                    .ToDictionary(n => "ClientInHQMode", n => ((object)false, (object)true))
            );

            // Make sure it wasn't called, since the server is already in HQ mode
            instance.color2DDecoder.Verify(n => n.RequestToggleFusionQuality(It.IsAny<bool>()), Times.Never);
        }

        [Test]
        public void ReceivedAnHQFrameFromServer_HQWasntRequestedByTheUser_NotifyTheUser()
        {
            var instance = CreateInstance();
            var newState = CreateDefaultState();

            // Setup the mock store to return our fake state
            instance.store.SetMockState(newState);

            // The color2ddecoder also isn't requesting it
            instance.color2DDecoder.Setup(n => n.RequestingQualityChange).Returns(false);

            // Viewer is not currently in HQ mode or requesting HQ
            newState.ClientInHQMode.Value = false;
            newState.LoadingHQFrame.Value = false;

            // Make sure no actions were raised before
            Assert.AreEqual(instance.store.lastActionType, null);

            // Pretend we just received an HQ frame
            instance.store.RunDispatch(ViewerActionType.ServerQualityChanged, true);

            // Make sure the user was notified of the event
            Assert.AreEqual(instance.store.lastActionType, ViewerActionType.ShowNotification);
        }

        [Test]
        public void ServerQualityChangedToHQ_HQModelVisible()
        {
            var instance = CreateInstance();
            var newState = CreateDefaultState();

            // Pretend we're already connected
            newState.OverallConnectionStatus.Value = ConnectionStatus.Connected;

            // Setup the mock store to return our fake state
            instance.store.SetMockState(newState);

            // When called return true
            instance.color2DDecoder.Setup(n => n.ServerInHQMode).Returns(true);
            instance.holoportDecoder.Setup(n => n.IsModelVisible).Returns(false);
            instance.hqDecoder.Setup(n => n.IsModelVisible).Returns(false);

            // Trigger the server quality changed event
            instance.store.RunDispatch(ViewerActionType.ServerQualityChanged, true);

            // Make sure only the HQ model is true
            instance.holoportDecoder.VerifySet(n => n.IsModelVisible = false);
            instance.hqDecoder.VerifySet(n => n.IsModelVisible = true);
        }

        [Test]
        public void ServerQualityChangedToLQ_LQModelVisible()
        {
            var instance = CreateInstance();
            var newState = CreateDefaultState();

            // Pretend we're already connected
            newState.OverallConnectionStatus.Value = ConnectionStatus.Connected;

            // Setup the mock store to return our fake state
            instance.store.SetMockState(newState);

            // When called return true
            instance.color2DDecoder.Setup(n => n.ServerInHQMode).Returns(false);
            instance.holoportDecoder.Setup(n => n.IsModelVisible).Returns(false);
            instance.hqDecoder.Setup(n => n.IsModelVisible).Returns(false);

            // Trigger the server quality changed event
            instance.store.RunDispatch(ViewerActionType.ServerQualityChanged, false);

            // Make sure only the HQ model is true
            instance.holoportDecoder.VerifySet(n => n.IsModelVisible = true);
            instance.hqDecoder.VerifySet(n => n.IsModelVisible = false);
        }

        [Test]
        public void ServerQualityChangeIsRequested_DispatchesServerQualityChangeRequestAction()
        {
            var instance = CreateInstance();
            var oldState = CreateDefaultState();
            var newState = CreateDefaultState();

            // Make sure no actions were raised before
            Assert.AreEqual(instance.store.lastActionType, null);

            // Raise the event
            instance.color2DDecoder.Raise(n => n.ServerQualityChangeRequested += null, true);

            // Make sure the frame received events is raised
            Assert.AreEqual(instance.store.lastActionType, ViewerActionType.ServerQualityChangeRequested);
        }

        [Test]
        public void SystemPlaying_PauseStateChanged_DecodersPaused()
        {
            var instance = CreateInstance();
            var newState = CreateDefaultState();
            var oldState = CreateDefaultState();

            // Pretend we were playing, but now we're paused
            oldState.Paused.Value = false;
            newState.Paused.Value = true;

            string lastColorMethod = null;
            string lastHoloMethod = null;

            // Setup the decoder mocks
            instance.color2DDecoder
                .Setup(n => n.Pause()).Callback(() => lastColorMethod = "Pause");
            instance.holoportDecoder
                .Setup(n => n.Pause()).Callback(() => lastHoloMethod = "Pause");
            instance.color2DDecoder
                .Setup(n => n.Resume()).Callback(() => lastColorMethod = "Resume");
            instance.holoportDecoder
                .Setup(n => n.Resume()).Callback(() => lastHoloMethod = "Resume");

            // Trigger the pause state change
            instance.store.RunStateChange(
                newState,
                oldState,
                new string[1]
                    .ToDictionary(n => "Paused", n => ((object)oldState.Paused.Value, (object)newState.Paused.Value))
            );

            // Make sure that the last thing that was called was "Pause"
            Assert.That(lastColorMethod, Is.EqualTo("Pause"));
            Assert.That(lastHoloMethod, Is.EqualTo("Pause"));
        }

        [Test]
        public void SystemPaused_PauseStateChanged_DecodersResumed()
        {
            var instance = CreateInstance();
            var newState = CreateDefaultState();
            var oldState = CreateDefaultState();

            // Pretend we were paused, but now we're playing
            oldState.Paused.Value = true;
            newState.Paused.Value = false;

            string lastColorMethod = null;
            string lastHoloMethod = null;

            // Setup the decoder mocks
            instance.color2DDecoder
                .Setup(n => n.Pause()).Callback(() => lastColorMethod = "Pause");
            instance.holoportDecoder
                .Setup(n => n.Pause()).Callback(() => lastHoloMethod = "Pause");
            instance.color2DDecoder
                .Setup(n => n.Resume()).Callback(() => lastColorMethod = "Resume");
            instance.holoportDecoder
                .Setup(n => n.Resume()).Callback(() => lastHoloMethod = "Resume");

            // Trigger the pause state change
            instance.store.RunStateChange(
                newState,
                oldState,
                new string[1]
                    .ToDictionary(n => "Paused", n => ((object)oldState.Paused.Value, (object)newState.Paused.Value))
            );

            // Make sure that the last thing that was called was "Resume"
            Assert.That(lastColorMethod, Is.EqualTo("Resume"));
            Assert.That(lastHoloMethod, Is.EqualTo("Resume"));
        }

        [Test]
        public void CaptureButtonClicked_AppIsCurrentlyPaused_DecodersPausedThenRestoredToPaused()
        {
            var instance = CreateInstance();
            var newState = CreateDefaultState();

            // Pretend we're currently paused in the app, the decoders should also be paused at this point
            newState.Paused.Value = true;

            // Setup the mock store to return our fake state
            instance.store.SetMockState(newState);

            string lastColorMethod = null;
            string lastHoloMethod = null;

            // Setup the decoder mocks
            instance.color2DDecoder
                .Setup(n => n.Pause()).Callback(() => lastColorMethod = "Pause");
            instance.holoportDecoder
                .Setup(n => n.Pause()).Callback(() => lastHoloMethod = "Pause");
            instance.color2DDecoder
                .Setup(n => n.Resume()).Callback(() => lastColorMethod = "Resume");
            instance.holoportDecoder
                .Setup(n => n.Resume()).Callback(() => lastHoloMethod = "Resume");

            // Trigger the snapshot button event
            instance.store.RunDispatch(ViewerActionType.SnapshotButtonClicked, false);

            // Make sure that the last thing that was called was "Paused", since the app started paused
            Assert.That(lastColorMethod, Is.EqualTo("Pause"));
            Assert.That(lastHoloMethod, Is.EqualTo("Pause"));
        }

        [Test]
        public void CaptureButtonClicked_AppIsCurrentlyPlaying_DecodersPausedThenRestoredToPlaying()
        {
            var instance = CreateInstance();
            var newState = CreateDefaultState();

            // Pretend we're not paused in the app, the decoders should be playing
            newState.Paused.Value = false;

            // Setup the mock store to return our fake state
            instance.store.SetMockState(newState);

            string lastColorMethod = null;
            string lastHoloMethod = null;

            // Setup the decoder mocks
            instance.color2DDecoder
                .Setup(n => n.Pause()).Callback(() => lastColorMethod = "Pause");
            instance.holoportDecoder
                .Setup(n => n.Pause()).Callback(() => lastHoloMethod = "Pause");
            instance.color2DDecoder
                .Setup(n => n.Resume()).Callback(() => lastColorMethod = "Resume");
            instance.holoportDecoder
                .Setup(n => n.Resume()).Callback(() => lastHoloMethod = "Resume");

            // Trigger the snapshot button event
            instance.store.RunDispatch(ViewerActionType.SnapshotButtonClicked, false);

            // Make sure that the last thing that was called was "Paused", since the app started paused
            Assert.That(lastColorMethod, Is.EqualTo("Resume"));
            Assert.That(lastHoloMethod, Is.EqualTo("Resume"));
        }

        [Test]
        public void CaptureButtonClicked_DecodersPausedWhileCapturing()
        {
            var instance = CreateInstance();
            var newState = CreateDefaultState();

            newState.Paused.Value = true;

            instance.store.SetMockState(newState);

            string lastColorMethod = null;
            string lastHoloMethod = null;

            // Setup the decoder mocks
            instance.color2DDecoder
                .Setup(n => n.Pause()).Callback(() => lastColorMethod = "Pause");
            instance.holoportDecoder
                .Setup(n => n.Pause()).Callback(() => lastHoloMethod = "Pause");
            instance.color2DDecoder
                .Setup(n => n.Resume()).Callback(() => lastColorMethod = "Resume");
            instance.holoportDecoder
                .Setup(n => n.Resume()).Callback(() => lastHoloMethod = "Resume");

            bool captureRawFramesCalled = false;

            // Hook the RawFrames capture call to see if the decoders are paused during it
            ((TestableViewerEpics)instance.epics).OnCaptureRawFrames += (object _, EventArgs ea) =>
            {
                captureRawFramesCalled = true;

                // Make sure that the last thing that was called was "Paused", since the app started paused
                Assert.That(lastColorMethod, Is.EqualTo("Pause"));
                Assert.That(lastHoloMethod, Is.EqualTo("Pause"));
            };

            // Trigger the snapshot button event
            instance.store.RunDispatch(ViewerActionType.SnapshotButtonClicked, false);

            // Make sure the function was actually called
            Assert.That(captureRawFramesCalled, Is.True);
        }

        private (
            ViewerEpics epics, 
            Mock<ViewerManager> manager, 
            MockViewerStore store,
            Mock<Color2DDecoder> color2DDecoder,
            Mock<HoloportDecoder> holoportDecoder,
            Mock<SettingsManager> settingsManager,
            Mock<HoloportRawFrameViewer> hqDecoder
        ) CreateInstance()
        {
            var store = new MockViewerStore();
            var color2DDecoder = new Mock<Color2DDecoder>();
            var holoportDecoder = new Mock<HoloportDecoder>();
            var hqDecoder = new Mock<HoloportRawFrameViewer>();
            var toaster = new Mock<Toaster>();
            var manager = new Mock<ViewerManager>();
            var settingsManager = new Mock<SettingsManager>();

            // Dump snapshots into a temp directory
            settingsManager.Setup(n => n.SnapshotsDirectory).Returns(Path.Combine(tempDirectory, $"./Snapshots"));
            settingsManager.Setup(n => n.CaptureAllRawFrames).Returns(true);

            manager.Object.panCursor = new Texture2D(1, 1);
            manager.Object.zoomCursor = new Texture2D(1, 1);
            manager.Object.orbitCursor = new Texture2D(1, 1);
            manager.Object.color2DDecoder = color2DDecoder.Object;
            manager.Object.holoportDecoder = holoportDecoder.Object;
            manager.Object.hqHoloportViewer = hqDecoder.Object;

            manager.SetupGet(n => n.Toaster).Returns(toaster.Object);

            var epics = new TestableViewerEpics(
                store,
                manager.Object,
                settingsManager.Object
            );
            return (epics, manager, store, color2DDecoder, holoportDecoder, settingsManager, hqDecoder);
        }

        // TODO: Break this hard dep
        private static ViewerState /* IUnifluxState */ CreateDefaultState()
        {
            return new ViewerState(
                ViewerMode.Holoportation,
                0
            );
        }

        private class TestableViewerEpics : ViewerEpics
        {
            public TestableViewerEpics(
                MockViewerStore store,
                ViewerManager manager,
                SettingsManager settings
            ): base(store, manager, settings) { }

            public event EventHandler OnCaptureRawFrames;

            protected override bool CaptureRawFrames(string directory, bool flipUpsideDown = true)
            {
                OnCaptureRawFrames?.Invoke(this, EventArgs.Empty);
                return true;
            }

            protected override void CaptureScreen(string directory)
            {
            }
            protected override bool CaptureView(GameObject view, string directory, string name)
            {
                return true;
            }
        }
    }
}
