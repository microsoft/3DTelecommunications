using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.UI;
namespace Assets.Scripts.Common.UI
{
    [Serializable]
    public class BetterToggleGroup : ToggleGroup
    {
        public delegate void ChangedEventHandler(Toggle newActive);

        public event ChangedEventHandler OnChange;
        protected override void Start()
        {
            foreach (Toggle toggle in this.GetToggles())
            {

                toggle.onValueChanged.AddListener((isSelected) =>
                {
                    if (!isSelected)
                    {
                        return;
                    }
                    var activeToggle = Active();
                    DoOnChange(activeToggle);
                });
            }
        }
        public Toggle Active()
        {
            return ActiveToggles().FirstOrDefault();
        }
        public IEnumerable<Toggle> GetToggles()
        {
            foreach (Transform transformToggle in gameObject.transform)
            {
                var toggle = transformToggle.gameObject.GetComponent<Toggle>();

                if (toggle != null)
                {
                    yield return toggle;
                }
            }

        }

        protected virtual void DoOnChange(Toggle newactive)
        {
            var handler = OnChange;
            if (handler != null) handler(newactive);
        }
    }
}
