using Assets.Scripts.Common;
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Assets.Scripts.Uniflux
{

    public class UnifluxStoreImpl<A, S> : IUnifluxStore<A, S> where S: IUnifluxState
    {
        private S state;
        private string name;

        #region events
        public event ReducerMiddleware<A, S> ReducerMiddleware;
        public event DispatchMiddleware<A, S> DispatchMiddleware;
        public event StateChangeMiddleware<S> StateChangeMiddleware;
        public event StateChangeMiddleware<S> LateStateChangeMiddleware;
        #endregion

        public UnifluxStoreImpl(string name, S state)
        {
            if (state == null)
            {
                throw new ArgumentNullException("state");
            }
            if (name == null)
            {
                throw new ArgumentNullException("name");
            }
            this.name = name;
            this.state = state;
        }

        public void Dispatch<ActionPayload>(UnifluxAction<A, ActionPayload> action)
        {
            Dictionary<string, (object, object)> stateChanges =
                new Dictionary<string, (object, object)>();

            var newState = (S)state.Clone();
            var oldState = state;

            UnifluxStateChangeHandler handler = (prop, newValue, oldValue) => stateChanges.Add(prop.Name, (newValue, oldValue));
            try
            {
                newState.OnChange += handler;
                ReducerMiddleware?.Invoke(new UnifluxAction<A, object>(action.actionType, action.payload), newState);
            } 
            finally
            {
                newState.OnChange -= handler;
            }

            state = newState;
            state.Readonly();

            if (stateChanges.Count > 0)
            {
                StateChangeMiddleware?.Invoke(state, oldState, stateChanges);
                LateStateChangeMiddleware?.Invoke(state, oldState, stateChanges);
            }

            DispatchMiddleware?.Invoke(new UnifluxAction<A, object>(action.actionType, action.payload));
        }

        public S GetState()
        {
            return state;
        }
    }
}
