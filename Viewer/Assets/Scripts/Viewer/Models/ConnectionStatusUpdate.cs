using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Assets.Scripts.Viewer.Models
{
    public struct ConnectionStatusUpdate
    {
        public readonly ConnectionStatus status;
        public readonly string error;
        public readonly DateTime timeStamp;
        public readonly ConnectionSubsystem subSystem;

        public ConnectionStatusUpdate(ConnectionStatus status, ConnectionSubsystem system, string error)
        {
            this.status = status;
            this.error = error;
            this.timeStamp = DateTime.Now;
            subSystem = system;
        }

        public ConnectionStatusUpdate(ConnectionStatus status, ConnectionSubsystem system) : this(status, system, null) { }

        public override string ToString()
        {
            string prettyError = string.IsNullOrEmpty(error) ? "" : error;
            return $"{subSystem}: {status} {prettyError}";
        }
    }

    public enum ConnectionStatus
    {
        Connected,
        Connecting,
        Disconnected,
        Error,
    }

    public enum ConnectionSubsystem
    {
        Holoportation3D,
        Color2D,
        None,
    }
}
