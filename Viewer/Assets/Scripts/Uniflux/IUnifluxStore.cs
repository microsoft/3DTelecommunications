using Assets.Scripts.Common;
using System.Collections.Generic;

namespace Assets.Scripts.Uniflux
{
    public delegate void ReducerMiddleware<A, S>(UnifluxAction<A, object> action, S state) where S : IUnifluxState;
    public delegate void DispatchMiddleware<A, S>(UnifluxAction<A, object> action);
    public delegate void StateChangeMiddleware<S>(S newState, S oldState, IReadOnlyDictionary<string, (object, object)> changes);

    public interface IUnifluxStore<A, S> where S : IUnifluxState
    {
        event DispatchMiddleware<A, S> DispatchMiddleware;
        event ReducerMiddleware<A, S> ReducerMiddleware;
        event StateChangeMiddleware<S> StateChangeMiddleware;
        event StateChangeMiddleware<S> LateStateChangeMiddleware;

        S GetState();

        void Dispatch<P>(UnifluxAction<A, P> action);
    }
}
