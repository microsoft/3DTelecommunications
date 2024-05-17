using System;

namespace Assets.Scripts
{
    /// <summary>
    /// Event handler for when a frame is ready
    /// </summary>
    /// <param name="sender"></param>
    /// <param name="frame"></param>
    public delegate void FrameReadyHandler(object sender, HoloportObjectStruct frame);

    public interface IFusionClient: IDisposable
    {
        int CountSkippedFrames { get; }
        float FPS { get; }
        bool Paused { get; set; }

        event FrameReadyHandler FrameReady;

        void Pause();
        void Start();
        void Stop();
    }
}