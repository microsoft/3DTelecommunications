using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System;
[CreateAssetMenu]
[Serializable]
public class UpdateUITextEvent : ScriptableObject
{

	protected List<UpdateUITextListener> listeners =
	new List<UpdateUITextListener>();

	public void Raise(Text t1, string t2)
	{
		for (int i = listeners.Count - 1; i >= 0; i--)
		{
			UpdateUITextListener gel = (UpdateUITextListener)listeners[i];
			if (gel != null)
				gel.OnEventRaised(t1, t2);
		}
	}

	public void RegisterListener(UpdateUITextListener listener)
	{ listeners.Add(listener); }

	public void UnregisterListener(UpdateUITextListener listener)
	{ listeners.Remove(listener); }

}
