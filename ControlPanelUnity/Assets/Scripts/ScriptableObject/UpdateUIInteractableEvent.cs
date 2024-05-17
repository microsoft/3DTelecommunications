using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

[CreateAssetMenu]
[Serializable]
public class UpdateUIInteractableEvent : ScriptableObject
{
	protected List<UpdateUIInteractableListener> listeners =
		new List<UpdateUIInteractableListener>();

	public void Raise(Selectable t1, bool t2)
	{
		for (int i = 0; i < listeners.Count; i++)
		{
			UpdateUIInteractableListener listener = listeners[i];
			if (listener != null)
				listener.OnEventRaised(t1, t2);
		}
	}

	public void RegisterListener(UpdateUIInteractableListener listener)
	{ listeners.Add(listener); }

	public void UnregisterListener(UpdateUIInteractableListener listener)
	{ listeners.Remove(listener); }
}
