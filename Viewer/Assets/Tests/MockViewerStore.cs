using Assets.Scripts.Uniflux;
using Assets.Scripts.Viewer.State;
using Assets.Scripts.Viewer.State.Actions;
using System.Collections.Generic;

public class MockViewerStore : IUnifluxStore<ViewerActionType, ViewerState>
{
    private ViewerState mockState;

    public event DispatchMiddleware<ViewerActionType, ViewerState> DispatchMiddleware;
    public event ReducerMiddleware<ViewerActionType, ViewerState> ReducerMiddleware;
    public event StateChangeMiddleware<ViewerState> StateChangeMiddleware;
    public event StateChangeMiddleware<ViewerState> LateStateChangeMiddleware;

    public ViewerActionType? lastActionType = null;
    public object lastPayload;

    public void Dispatch<P>(UnifluxAction<ViewerActionType, P> action)
    {
        lastActionType = action.actionType;
        lastPayload = action.payload;
    }

    public void RunDispatch(ViewerActionType type, object payload)
    {
        DispatchMiddleware?.Invoke(new UnifluxAction<ViewerActionType, object>(type, payload));
    }

    public void RunReducers(ViewerActionType type, object payload, ViewerState state)
    {
        ReducerMiddleware?.Invoke(new UnifluxAction<ViewerActionType, object>(type, payload), state);
    }

    public void RunStateChange(ViewerState state, ViewerState oldState, IReadOnlyDictionary<string, (object, object)> changes)
    {
        StateChangeMiddleware?.Invoke(state, oldState, changes);
        LateStateChangeMiddleware?.Invoke(state, oldState, changes);
    }

    public ViewerState GetState()
    {
        return mockState;
    }


    public void SetMockState(ViewerState state)
    {
        mockState = state;
    }
}
