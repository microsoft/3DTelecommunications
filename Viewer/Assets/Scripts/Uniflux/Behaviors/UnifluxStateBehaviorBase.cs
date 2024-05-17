using Assets.Scripts.Common;
using System.Collections.Generic;
using UnityEngine;

namespace Assets.Scripts.Uniflux.Behaviors
{
    public delegate IObservableProperty<T> StatePropertyGetter<T, S>(S state) where S : IUnifluxState;

    public class UnifluxStateBehaviorBase<A, S> : MonoBehaviour where S : IUnifluxState
    {
        private bool isEnabled = false;
        private Dictionary<string, HandlerData> changeHandlers = new Dictionary<string, HandlerData>();
        public void Dispatch<T>(IUnifluxStore<A, S> store, UnifluxAction<A, T> action)
        {
            store.Dispatch<T>(action);
        }

        protected IObservableProperty<T> GetStateProperty<T>(IUnifluxStore<A, S> store, StatePropertyGetter<T, S> getter)
        {
            return getter.Invoke(store.GetState());
        }

        protected void RegisterStateChangeHandler<T>(IUnifluxStore<A, S> store, StatePropertyGetter<T, S> getter, PropertyChangedEventHandler<T> handler)
        {
            string propName = GetStateProperty(store, getter).Name;

            // Ensure that the same property doesn't get subscribed to twice
            if (!changeHandlers.ContainsKey(propName))
            {
                HandlerData data = new HandlerData()
                {
                    store = store,
                    handler = (S newState, S oldState, IReadOnlyDictionary<string, (object, object)> changes) =>
                    {
                        // The isEnabled check is necessary because sometimes
                        // during the StateChanged event from the store, a parent component gets disabled (thereby disabiling a child).
                        // The child's StateChanged event handler does actually get unsubscribed from the store,
                        // but since we're currently in a StateChanged event the child handler still gets triggered for the original event
                        (object, object) change;
                        if (isEnabled && changes.TryGetValue(propName, out change))
                        {
                            handler((T)change.Item1, (T)change.Item2);
                        }
                    }
                };

                changeHandlers.Add(propName, data);

                if (isEnabled)
                {
                    AttachHandlers();
                }
            }
        }

        protected virtual void OnEnable()
        {
            isEnabled = true;

            AttachHandlers();
        }

        protected virtual void OnDisable()
        {
            isEnabled = false;

            DetachHandlers();
        }

        protected virtual void OnDestroy()
        {
            DetachHandlers();
        }

        protected void AttachHandlers()
        {
            DetachHandlers();
            foreach (var data in changeHandlers.Values)
            {
                if (!data.attached)
                {
                    data.store.StateChangeMiddleware += data.handler;
                    data.attached = true;
                }
            }
        }

        protected void DetachHandlers()
        {
            foreach (var data in changeHandlers.Values)
            {
                if (data.attached)
                {
                    data.store.StateChangeMiddleware -= data.handler;
                    data.attached = false;
                }
            }
        }

        private class HandlerData
        {
            public IUnifluxStore<A, S> store;
            public StateChangeMiddleware<S> handler;
            public bool attached;
        }
    }
}
