using Assets.Scripts.Viewer.Behaviors;
using Assets.Scripts.Viewer.Components;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace Assets.Scripts.Viewer.Common
{
    public class Toaster
    {
        private ToastContainer toastContainer;

        public void ShowToast(Toast toast)
        {
            if (toastContainer != null)
            {
                toastContainer.AddToast(toast, toast.Callback);
            }
        }

        public ToastContainer ToastContainer
        {
            set
            {
                toastContainer = value;
            }
        }
    }
}
