using Assets.Scripts.Common;
using System;

namespace Assets.Scripts.Uniflux
{
    public delegate void UnifluxStateChangeHandler(IObservableProperty prop, object newValue, object oldValue);

    public interface IUnifluxState : ICloneable
    {
        event UnifluxStateChangeHandler OnChange;

        /// <summary>
        /// Enters a readonly state
        /// </summary>
        void Readonly();
    }
}
