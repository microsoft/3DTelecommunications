using Assets.Scripts.Viewer;
using Assets.Scripts.Viewer.Models;
using Assets.Scripts.Viewer.State;
using Moq;
using NUnit.Framework;
using System.Linq;

namespace Assets.Tests
{
    public class ViewerStateTests
    {
        [Test]
        public void Constructor_Basic_DoesntCrash()
        {
            CreateInstance();
        }

        [Test]
        public void CreateWithADifferentDefault2DCamera_ViewCamerasUseDefaultCamera()
        {
            int testCamera = 1234;
            var instance = CreateInstanceWithDefault2DCamera(testCamera);

            // Grab the cameras from the state
            var result = instance.ViewActiveCamera.Value.Select(n => n.CameraNum).ToArray();

            // Make sure they're the correct default camera
            Assert.That(result, Is.EquivalentTo(new int[] { testCamera, testCamera }));
        }

        public ViewerState CreateInstance()
        {
            return CreateInstanceWithDefault2DCamera(0);
        }

        public ViewerState CreateInstanceWithDefault2DCamera(int defaultCam)
        {
            return new ViewerState(ViewerMode.Holoportation, defaultCam);
        }
    }
}
