using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Assets.Scripts.Viewer.Models
{
    /// <summary>
    /// A binding between a view and a camera
    /// </summary>
    public struct ViewCamera
    {
        public int ViewId;
        public int CameraNum;

        public ViewCamera(int viewId, int cameraNum)
        {
            ViewId = viewId;
            CameraNum = cameraNum;
        }

        public override string ToString()
        {
            return $"{ViewId}: {CameraNum}";
        }
    }
}
