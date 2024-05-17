using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class UpdateUIInteractableListener : MonoBehaviour
{
    public ConcurrentDictionary<Selectable, bool> selectableToBool;
    public UpdateUIInteractableEvent triggerGameEvent;
    public void OnEnable()
    { triggerGameEvent.RegisterListener(this); }

    public void OnDisable()
    { triggerGameEvent.UnregisterListener(this); }

    public void OnEventRaised(Selectable uiElement, bool target)
    {
        selectableToBool.AddOrUpdate(uiElement, target, (key, oldvalue) => target);
    }
    public void Awake()
    {
        selectableToBool = new ConcurrentDictionary<Selectable, bool>();
    }

    public void Update()
    {
        foreach (var pair in selectableToBool)
        {
            if (pair.Key == null)
            {
                OutputHelper.OutputLog("Warning: trying to update an empty Selectable");
                continue;
            }
            pair.Key.interactable = pair.Value;
        }
        selectableToBool.Clear();
    }
}
