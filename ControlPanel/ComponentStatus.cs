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
        private double _fps_min;
        private double _fps_max;
        private double _temperature;
        private Status _status;
        private double _voltage_min;
        private double _voltage_max;
        private double _voltage;
        private bool _newDataReceived;
        private bool _videoRecordingStarted;
        private bool _videoTransferStarted;
        private bool _videoTransferFinished;
        private bool _bgCaptureFinished;
        private int _frameNum;
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
                    if (_fps == 0)
                    {
                        _fps_min = roundedValue;
                    }
                    _fps = roundedValue;
                    if (_fps < _fps_min || _fps_max > 2*_fps_min)
                    {
                        _fps_min = _fps;
                    }
                    if (_fps > _fps_max)
                    {
                        _fps_max = _fps;
                    }
                    //OnPropertyChanged(nameof(FPS));
                }
            }
        }
        public double Voltage
        {
            get => _voltage;
            set
            {
                double roundedValue = Math.Round(value, 2);
                if (_voltage != roundedValue)
                {
                    if (_voltage == 0)
                    {
                        _voltage_min = roundedValue;
                    }
                    _voltage = roundedValue;
                    if (_voltage < _voltage_min || _voltage_max > 2 * _voltage_min)
                    {
                        _voltage_min = _voltage;
                    }
                    if (_voltage > _voltage_max)
                    {
                        _voltage_max = _voltage;
                    }
                    //OnPropertyChanged(nameof(Voltage));
                }
            }
        }
        public double FPS_min
        {
            get => _fps_min;
            set
            {
                double roundedValue = Math.Round(value, 2);
                if (_fps_min != roundedValue)
                {
                    _fps_min = roundedValue;
                }
            }
        }
        public double FPS_max
        {
            get => _fps_max;
            set
            {
                double roundedValue = Math.Round(value, 2);
                if (_fps_max != roundedValue)
                {
                    _fps_max = roundedValue;
                }
            }
        }
        public double Voltage_min
        {
            get => _voltage_min;
            set
            {
                double roundedValue = Math.Round(value, 2);
                if (_voltage_min != roundedValue)
                {
                    _voltage_min = roundedValue;
                }
            }
        }
        public double Voltage_max
        {
            get => _voltage_max;
            set
            {
                double roundedValue = Math.Round(value, 2);
                if (_voltage_max != roundedValue)
                {
                    _voltage_max = roundedValue;
                }
            }
        }
        public CPC_ERROR ErrState { get; set; }
        public int FrameNum 
        { 
            get => _frameNum; 
            set 
            { 
                _frameNum = value; 
                //OnPropertyChanged(nameof(FrameNum)); 
            } 
        }
        public bool NewDataReceived 
        { 
            get => _newDataReceived;
            set
            {
                if (_newDataReceived != value)
                {
                    _newDataReceived = value;
                    //OnPropertyChanged(nameof(NewDataReceived));
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
                    //OnPropertyChanged(nameof(Temperature));
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
                    //OnPropertyChanged(nameof(Status));
                }
            }
        }

        public bool VideoRecordingStarted
        {
            get => _videoRecordingStarted;
            set
            {
                if (_videoRecordingStarted != value)
                {
                    _videoRecordingStarted = value;
                    //OnPropertyChanged(nameof(VideoRecordingStarted));
                }
            }
        }
        public bool VideoTransferStarted
        {
            get => _videoTransferStarted;
            set
            {
                if (_videoTransferStarted != value)
                {
                    _videoTransferStarted = value;
                    //OnPropertyChanged(nameof(VideoTransferStarted));
                }
            }
        }
        public bool VideoTransferFinished
        {
            get => _videoTransferFinished;
            set
            {
                if (_videoTransferFinished != value)
                {
                    _videoTransferFinished = value;
                    //OnPropertyChanged(nameof(VideoTransferFinished));
                }
            }
        }
        public bool BGCaptureFinished
        {
            get => _bgCaptureFinished;
            set
            {
                if (_bgCaptureFinished != value)
                {
                    _bgCaptureFinished = value;
                    //OnPropertyChanged(nameof(BGCaptureFinished));
                }
            }
        }

        public event PropertyChangedEventHandler PropertyChanged;

        protected void OnPropertyChanged(string propertyName)
        {
            //PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }
    }
}
