using System.Collections;
using NUnit.Framework;
using UnityEngine.TestTools;
using Assets.Scripts.Viewer.State.Middleware;
using System;
using Assets.Scripts.Viewer.State;
using Assets.Scripts.Viewer.Models;
using Assets.Scripts.Viewer;

namespace Tests
{
    public class ViewerReducersTests
    {
        [Test]
        public void Constructor_NullStore_ThrowsArgumentNullException()
        {
            Assert.Throws(typeof(ArgumentNullException), () =>  {
                // Use the Assert class to test conditions
                new ViewerReducers(null);
            });
        }

        [Test]
        public void SomePanelVisible_NewPanelRequested_NewPanelVisible()
        {
            var instance = CreateInstance();
            var state = CreateDefaultState();

            // Pretend the Navigation panel is open
            state.VisiblePanel.Value = ViewerPanel.Navigation;

            // User clicks on the help button
            instance.Item2.RunReducers(
                Assets.Scripts.Viewer.State.Actions.ViewerActionType.HelpButtonClick,
                null,
                state
            );

            // Make sure the visible panel has been updated
            Assert.AreEqual(ViewerPanel.Help, state.VisiblePanel.Value);
        }

        [Test]
        public void HelpButtonClicked_Once_HelpPanelVisible()
        {
            var instance = CreateInstance();
            var state = CreateDefaultState();
            instance.Item2.RunReducers(
                Assets.Scripts.Viewer.State.Actions.ViewerActionType.HelpButtonClick,
                null,
                state
            );

            Assert.AreEqual(ViewerPanel.Help, state.VisiblePanel.Value);
        }

        [Test]
        public void HelpButtonClicked_Twice_HelpPanelHidden()
        {
            var instance = CreateInstance();
            var state = CreateDefaultState();
            instance.Item2.RunReducers(
                Assets.Scripts.Viewer.State.Actions.ViewerActionType.HelpButtonClick,
                null,
                state
            );
            instance.Item2.RunReducers(
                Assets.Scripts.Viewer.State.Actions.ViewerActionType.HelpButtonClick,
                null,
                state
            );

            Assert.AreEqual(ViewerPanel.None, state.VisiblePanel.Value);
        }

        [Test]
        public void TutorialButtonClicked_Once_InTutorialModeTrue()
        {
            var instance = CreateInstance();
            var state = CreateDefaultState();
            instance.Item2.RunReducers(
                Assets.Scripts.Viewer.State.Actions.ViewerActionType.StartTutorialButtonClick,
                null,
                state
            );

            Assert.IsTrue(state.InTutorialMode.Value);
        }

        [Test]
        public void TutorialButtonClicked_Twice_InTutorialModeFalse()
        {
            var instance = CreateInstance();
            var state = CreateDefaultState();
            instance.Item2.RunReducers(
                Assets.Scripts.Viewer.State.Actions.ViewerActionType.StartTutorialButtonClick,
                null,
                state
            );
            instance.Item2.RunReducers(
                Assets.Scripts.Viewer.State.Actions.ViewerActionType.StartTutorialButtonClick,
                null,
                state
            );

            Assert.IsFalse(state.InTutorialMode.Value);
        }

        [Test]
        public void ServerQualityChangedWasChangedAndLoadingHQFrameIsTrue_LoadingHQFrameShouldBeChangedToFalse()
        {
            var instance = CreateInstance();
            var state = CreateDefaultState();

            state.LoadingHQFrame.Value = true;

            instance.Item2.RunReducers(
                Assets.Scripts.Viewer.State.Actions.ViewerActionType.ServerQualityChanged,
                true,
                state
            );

            Assert.IsFalse(state.LoadingHQFrame.Value);
        }

        [Test]
        public void ConnectionStatusChanged_Disconnected_InTutorialModeShouldBeFalse()
        {
            var instance = CreateInstance();
            var state = CreateDefaultState();

            // We're in tutorial mode
            state.InTutorialMode.Value = true;

            // One of our connections drop
            instance.Item2.RunReducers(
                Assets.Scripts.Viewer.State.Actions.ViewerActionType.ConnectionStatusChanged,
                new ConnectionStatusUpdate(ConnectionStatus.Disconnected, ConnectionSubsystem.Holoportation3D),
                state
            );

            // We are no longer in tutorial mode
            Assert.IsFalse(state.InTutorialMode.Value);
        }


        // 
        //[Test]
        //public void Color2DFrameReceived_LoadingHQFrameTrue__LoadingHQModeShouldBeSetFalse()
        //{
        //    var instance = CreateInstance();
        //    var state = CreateDefaultState();

        //    state.LoadingHQFrame.Value = true;

        //    instance.Item2.RunReducers(
        //        Assets.Scripts.Viewer.State.Actions.ViewerActionType.HQFrameReceived,
        //        null,
        //        state
        //    );

        //    Assert.IsFalse(state.LoadingHQFrame.Value);
        //}

        [Test]
        public void Initialize_AppModeIsNone_ModePanelShowBeVisible()
        {
            var instance = CreateInstance();
            var state = CreateDefaultState();
            state.AppMode.Value = ViewerMode.None;

            instance.Item2.RunReducers(
                Assets.Scripts.Viewer.State.Actions.ViewerActionType.Initialize,
                null,
                state
            );

            Assert.AreEqual(ViewerPanel.Mode, state.VisiblePanel.Value);
        }

        // A UnityTest behaves like a coroutine in Play Mode. In Edit Mode you can use
        // `yield return null;` to skip a frame.
        [UnityTest]
        public IEnumerator ViewerReducersTestsWithEnumeratorPasses()
        {
            // Use the Assert class to test conditions.
            // Use yield to skip a frame.
            yield return null;
        }

        private static (ViewerReducers, MockViewerStore) CreateInstance()
        {
            var store = new MockViewerStore();
            var reducers = new ViewerReducers(store);
            return (reducers, store);
        }

        // TODO: Break this hard dep
        private static ViewerState /* IUnifluxState */ CreateDefaultState()
        {
            return new ViewerState(
                ViewerMode.Holoportation,
                0
            );
        }
    }
}
