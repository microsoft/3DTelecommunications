using Assets.Scripts.Common;
using Assets.Scripts.Uniflux;
using Assets.Scripts.Uniflux.Behaviors;
using Assets.Scripts.Viewer.State;
using Assets.Scripts.Viewer.State.Actions;

namespace Assets.Scripts.Viewer
{
    public class ViewerStateBehavior : UnifluxStateBehaviorBase<ViewerActionType, ViewerState>
    {
        public void Dispatch<T>(UnifluxAction<ViewerActionType, T> action)
        {
            base.Dispatch<T>(ViewerManager.Current().Store, action);
        }

        protected IObservableProperty<T> GetStateProperty<T>(StatePropertyGetter<T, ViewerState> getter)
        {
            return base.GetStateProperty<T>(ViewerManager.Current().Store, getter);
        }

        protected void RegisterStateChangeHandler<T>(StatePropertyGetter<T, ViewerState> getter, PropertyChangedEventHandler<T> handler)
        {
            base.RegisterStateChangeHandler<T>(ViewerManager.Current().Store, getter, handler);
        }
    }
}
