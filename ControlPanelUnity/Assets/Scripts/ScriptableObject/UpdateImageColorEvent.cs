using System.Collections;
using System.Collections.Generic;
using UnityEngine.UI;
using UnityEngine;
using System;
[CreateAssetMenu]
[Serializable]
public class UpdateImageColorEvent : ScriptableObject
{
	
		protected List<UpdateImageColorListener> listeners =
		new List<UpdateImageColorListener>();

		public void Raise(Image t1, Color t2)
		{
			for (int i = listeners.Count - 1; i >= 0; i--)
			{
			UpdateImageColorListener gel = (UpdateImageColorListener)listeners[i];
				if (gel != null)
					gel.OnEventRaised(t1, t2);
			}
		}

		public void RegisterListener(UpdateImageColorListener listener)
		{ listeners.Add(listener); }

		public void UnregisterListener(UpdateImageColorListener listener)
		{ listeners.Remove(listener); }
	
}