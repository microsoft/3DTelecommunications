using Assets.Scripts.Common;
using Assets.Scripts.Uniflux;
using Assets.Scripts.Viewer.Models;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using UnityEngine;

namespace Assets.Scripts.Viewer.State
{
    // TODO: Not efficient

    public class ViewerState : IUnifluxState, ICloneable
    {
        private bool isReadOnly = false;

        public event UnifluxStateChangeHandler OnChange;

        public readonly ObservableProperty<ViewerPanel> VisiblePanel;
        public readonly ObservableProperty<InitConnectionData> ConnectionInfo = new ObservableProperty<InitConnectionData>("ConnectionInfo", new InitConnectionData()
        {
            target = null
        });
        public readonly ObservableProperty<ConnectionStatus> OverallConnectionStatus = new ObservableProperty<ConnectionStatus>("OverallConnectionStatus", ConnectionStatus.Disconnected);
        public readonly ObservableProperty<string> OverallConnectionError = new ObservableProperty<string>("OverallConnectionError", null);

        public readonly ObservableProperty<ConnectionStatus> Holoport3DConnectionStatus = new ObservableProperty<ConnectionStatus>("Holoport3DConnectionStatus", ConnectionStatus.Disconnected);
        public readonly ObservableProperty<string> Holoport3DConnectionError = new ObservableProperty<string>("Holoport3DConnectionError", null);

        public readonly ObservableProperty<ConnectionStatus> Color2DConnectionStatus = new ObservableProperty<ConnectionStatus>("Color2DConnectionStatus", ConnectionStatus.Disconnected);
        public readonly ObservableProperty<string> Color2DConnectionError = new ObservableProperty<string>("Color2DConnectionError", null);

        public readonly ObservableProperty<bool> DualView;
        public readonly ObservableProperty<ViewerTool> ActiveTool = new ObservableProperty<ViewerTool>("ActiveTool", ViewerTool.None);
        public readonly ObservableProperty<ViewerMode> AppMode;
        public readonly ObservableProperty<bool> InTutorialMode = new ObservableProperty<bool>("InTutorialMode", false);
        public readonly ObservableProperty<bool> ClientInHQMode = new ObservableProperty<bool>("ClientInHQMode", false);
        public readonly ObservableProperty<bool> LoadingHQFrame = new ObservableProperty<bool>("RequestingHQImage", false);
        public readonly ObservableProperty<bool> Paused = new ObservableProperty<bool>("Paused", false);
        public readonly ObservableProperty<bool> ReporterViewVisible = new ObservableProperty<bool>("ReporterViewVisible", false);

        /// <summary>
        /// The active dimension for each view
        /// </summary>
        public readonly ObservableProperty<ViewDimension[]> ViewActiveDimension;

        /// <summary>
        /// The active camera for each view
        /// </summary>
        public readonly ObservableProperty<ViewCamera[]> ViewActiveCamera;

        private static IEnumerable<FieldInfo> propertyFields = 
            typeof(ViewerState).GetFields()
                .Where(n => typeof(IObservableProperty).IsAssignableFrom(n.FieldType));

        private static IEnumerable<FieldInfo> boolPropertyFields =
            typeof(ViewerState).GetFields()
                .Where(n => typeof(IObservableProperty<bool>).IsAssignableFrom(n.FieldType));
        
        private int default2DCamera;

        public ViewerState(
            ViewerMode appMode,
            int default2DCamera
        ) : this(null, appMode, default2DCamera) { }

        public ViewerState(ViewerState toCopy, ViewerMode appMode, int default2DCamera)
        {
            this.default2DCamera = default2DCamera;
            AppMode = new ObservableProperty<ViewerMode>("AppMode", appMode);

            // If we already have a mode, then no need to show the viewer panel
            ViewerPanel panel = ViewerPanel.Mode;
            if (AppMode.Value != ViewerMode.None)
            {
                panel = ViewerPanel.None;
            }

            VisiblePanel = new ObservableProperty<ViewerPanel>("VisiblePanel", panel);
            ViewActiveDimension = new ObservableProperty<ViewDimension[]>("ViewActiveDimension", GetInitialViewDimensions());
            ViewActiveCamera = new ObservableProperty<ViewCamera[]>("ViewActiveCamera", GetInitialViewCameras());
            DualView = new ObservableProperty<bool>("DualView", GetDualViewDefaultValue(AppMode.Value));

            // Gross, but makes it easier to just add new state properties
            foreach (FieldInfo f in propertyFields)
            {
                var prop = ((IObservableProperty)f.GetValue(this));
                if (toCopy != null)
                {
                    prop.Value = ((IObservableProperty)f.GetValue(toCopy)).Value;
                }
                prop.OnChange += DefineHandler(prop);
            }
        }

        public bool GetDualViewDefaultValue(ViewerMode mode)
        {
            return mode == ViewerMode.Traditional ? false : true;
        }

        public ViewDimension[] GetInitialViewDimensions()
        {
            return new ViewDimension[] {
                new ViewDimension(1, Dimension.TwoD),
                new ViewDimension(2, Dimension.ThreeD)
            };
        }

        public ViewCamera[] GetInitialViewCameras()
        {
            return new ViewCamera[] {
                new ViewCamera(1, default2DCamera),
                new ViewCamera(2, default2DCamera)
            };
        }

        public static IEnumerable<string> GetPropertyNames()
        {
            return propertyFields.Select(n => n.Name);
        }

        public static IEnumerable<string> GetBoolPropertyNames()
        {
            return boolPropertyFields.Select(n => n.Name);
        }

        public IObservableProperty GetPropertyByName(string name)
        {
            FieldInfo f = propertyFields.FirstOrDefault(n => n.Name == name);
            if (f != null)
            {
                return ((IObservableProperty)f.GetValue(this));
            }
            return null;
        }

        private PropertyChangedEventHandler DefineHandler(IObservableProperty prop)
        {
            return delegate (object newValue, object oldValue)
            {
                OnChange?.Invoke(prop, newValue, oldValue);
            };
        }

        public object Clone()
        {
            return new ViewerState(this, AppMode.Value, default2DCamera);
        }

        public void Readonly()
        {
            isReadOnly = true;
            foreach (FieldInfo f in propertyFields)
            {
                ((IObservableProperty)f.GetValue(this)).Readonly();
            }
        }
    }
}
