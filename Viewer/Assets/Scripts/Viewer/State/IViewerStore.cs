using Assets.Scripts.Uniflux;
using Assets.Scripts.Viewer.State.Actions;

namespace Assets.Scripts.Viewer.State
{
    public interface IViewerStore: IUnifluxStore<ViewerActionType, ViewerState>
    {
    }
}
