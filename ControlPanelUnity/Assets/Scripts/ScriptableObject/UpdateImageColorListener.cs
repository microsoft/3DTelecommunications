using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.UI;

[Serializable]
public class UpdateImageColorListener : MonoBehaviour
{
    public ConcurrentDictionary<Image, Color> imageToColor;
    public UpdateImageColorEvent twoParameterGameEvent;
    public void OnEnable()
    { twoParameterGameEvent.RegisterListener(this); }

    public void OnDisable()
    { twoParameterGameEvent.UnregisterListener(this); }

    public void OnEventRaised(Image t1, Color t2)
    {
        if(imageToColor.ContainsKey(t1))
        {
            imageToColor[t1] = t2;
        }
        else
        {
            imageToColor.TryAdd(t1, t2);
        }
    }
    public void Awake()
    {
        imageToColor = new ConcurrentDictionary<Image, Color>();


    }

    public void Update()
    {
        foreach (var pair in imageToColor)
        {
            if(pair.Key == null)
            {
                OutputHelper.OutputLog("Warning: trying to update an empty Image!");
                continue;
            }
            pair.Key.color = pair.Value;
        }
        imageToColor.Clear();
    }

}

