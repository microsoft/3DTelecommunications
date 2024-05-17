using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System;
using System.Collections.Concurrent;

[Serializable]
public class UpdateUITextListener : MonoBehaviour
{
    public ConcurrentDictionary<Text, string> textToString;
    public UpdateUITextEvent twoParameterGameEvent;
    public void OnEnable()
    { twoParameterGameEvent.RegisterListener(this); }

    public void OnDisable()
    { twoParameterGameEvent.UnregisterListener(this); }

    public void OnEventRaised(Text t1, string t2)
    {
        if (textToString.ContainsKey(t1))
        {
            textToString[t1] = t2;
        }
        else
        {
            textToString.TryAdd(t1, t2);
        }
    }
    public void Awake()
    {
        textToString = new ConcurrentDictionary<Text, string>();


    }

    public void Update()
    {
        foreach (var pair in textToString)
        {
            if (pair.Key == null)
            {
                OutputHelper.OutputLog("Warning: trying to update an empty Text!");
                continue;
            }
            pair.Key.text = pair.Value;
        }
        textToString.Clear();
    }

}

