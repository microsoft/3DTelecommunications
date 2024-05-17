using Assets.Scripts.Viewer.Models;
using Assets.Scripts.Viewer.State.Actions;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.UI;

namespace Assets.Scripts.Viewer.Behaviors
{
    public class PanelCloseBehavior: ViewerStateBehavior
    {
        [SerializeField]
        public ViewerPanel panelType;

        [SerializeField]
        public Button closeButton;

        protected override void OnEnable()
        {
            base.OnEnable();
            closeButton.onClick.AddListener(OnCloseButtonClick);
        }

        protected override void OnDisable()
        {
            base.OnDisable();
            closeButton.onClick.RemoveListener(OnCloseButtonClick);
        }

        private void OnCloseButtonClick()
        {
            Dispatch(ViewerActionCreators.panelClosed(panelType));
        }
    }
}
