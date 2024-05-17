using Assets.Scripts.Uniflux;
using Assets.Scripts.Viewer.State.Actions;
using System;
using UnityEngine;
using System.Linq;
using System.Text;
using Assets.Scripts.Common;
using System.Collections.Generic;

namespace Assets.Scripts.Viewer.State.Middleware
{
    public class ActionLogger
    {
        public ActionLogger(IUnifluxStore<ViewerActionType, ViewerState> store)
        {
            store.DispatchMiddleware += OnDispatch;
            store.StateChangeMiddleware += OnStateChange;
        }

        private void OnDispatch(UnifluxAction<ViewerActionType, object> action)
        {
            Log($"Action dispatched {action.actionType} = {PrettyPrint(action.payload)}", action.logInProduction);
        }

        private void OnStateChange(ViewerState newState, ViewerState oldState, IReadOnlyDictionary<string, (object, object)> changes)
        {
            if (changes.Count >= 0)
            {
                StringBuilder sb = new StringBuilder();
                foreach (KeyValuePair<string, (object, object)> change in changes)
                {
                    sb.Append($"\t{change.Key}: {PrettyPrint(change.Value.Item2)} -> {PrettyPrint(change.Value.Item1)}\n");
                }

                Log($"State changed: \n" + sb.ToString());
            }
        }

        private void Log(string message, bool logInProduction = true)
        {
#if UNITY_EDITOR
            // If we're in the editor, just log, for days
            OutputHelper.OutputLog(message);
#else
            if (logInProduction)
            {
                OutputHelper.OutputLog(message);
            }
#endif
        }

        private object PrettyPrint(object toPrint)
        {
            if (toPrint != null && toPrint.GetType().IsArray)
            {
                StringBuilder sb = new StringBuilder();
                foreach (object element in (Array)toPrint)
                {
                    if (sb.Length > 0)
                    {
                        sb.Append(", ");
                    }
                    sb.Append($"{{{element}}}");
                }
                toPrint = $"[{sb}]";
            }
            return toPrint;
        }
    }
}
