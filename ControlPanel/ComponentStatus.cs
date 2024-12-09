using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using PeabodyNetworkingLibrary;

namespace ControlPanel
{
    public enum Status
    {
        Stopped,
        Ready,
        Running,
        TimedOut
    }
    public class ComponentStatus : INotifyPropertyChanged
    {
        private double _fps;
        private double _temperature;
        private Status _status;
        private bool _newDataReceived;

        public int ID { get; set; }
        public string Name { get; set; }
        public string IP { get; set; }        
        public bool Subscribe { get; set; }
        public double FPS
        {
            get => _fps;
            set
            {
                double roundedValue = Math.Round(value, 2);
                if (_fps != roundedValue)
                {
                    _fps = roundedValue;
                    OnPropertyChanged(nameof(FPS));
                }
            }
        }
        public double FPS_min { get; set; }
        public double FPS_max { get; set; }
        public CPC_ERROR ErrState { get; set; }
        public int FrameNum { get; set; }
        public bool NewDataReceived 
        { 
            get => _newDataReceived;
            set
            {
                if (_newDataReceived != value)
                {
                    _newDataReceived = value;
                    OnPropertyChanged(nameof(NewDataReceived));
                }
            } 
        }
        public double Temperature
        {
            get => _temperature;
            set
            {
                double roundedValue = Math.Round(value, 2);
                if (_temperature != roundedValue)
                {
                    _temperature = roundedValue;
                    OnPropertyChanged(nameof(Temperature));
                }
            }
        }
        public Status Status
        {
            get => _status;
            set
            {
                if (_status != value)
                {
                    _status = value;
                    OnPropertyChanged(nameof(Status));
                }
            }
        }

        public event PropertyChangedEventHandler PropertyChanged;

        protected void OnPropertyChanged(string propertyName)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }
    }
}
