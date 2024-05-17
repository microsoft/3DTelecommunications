using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using PeabodyNetworkingLibrary;

namespace ControlPanel
{    
    public class ComponentStatus
    {
        public int ID { get; set; }
        public string Name { get; set; }
        public string IP { get; set; }        
        public bool Subscribe { get; set; }        
        public bool IsReady { get; set; }
        public bool IsRunning { get; set; }
        public double FPS { get; set; }
        public double FPS_min { get; set; }
        public double FPS_max { get; set; }
        public CPC_ERROR ErrState { get; set; }
        public bool CanLaunch { get; set; }
        public int FrameNum { get; set; }
        public bool NewDataReceived { get; set; }
    }
}
