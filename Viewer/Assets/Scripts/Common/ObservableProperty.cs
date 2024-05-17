using System;
using System.Collections.Generic;
using System.Runtime.Remoting.Messaging;

namespace Assets.Scripts.Common
{
    public class ObservableProperty<T>: IObservableProperty, IObservableProperty<T>
    {
        private bool isReadOnly = false;
        private Dictionary<PropertyChangedEventHandler<T>, PropertyChangedEventHandler> handlerMappings = 
            new Dictionary<PropertyChangedEventHandler<T>, PropertyChangedEventHandler>();

        private PropertyChangedEventHandler _OnChange;
        private T _value;

        public ObservableProperty(string name, T initialValue)
        {
            this._value = initialValue;
            this.Name = name;
        }

        public string Name
        {
            get;
            private set;
        }

        public T Value
        {
            get
            {
                return this._value;
            } 
            set
            {
                if (!isReadOnly)
                {
                    if (!object.Equals(value, this._value))
                    {
                        T oldValue = this._value;
                        this._value = value;
                        this._OnChange?.Invoke(this._value, oldValue);
                    }
                } 
                else
                {
                    OutputHelper.OutputLog($"Attempting to change: {Name} when in readonly mode!");
                }
            }
        }

        object IObservableProperty.Value
        {
            get
            {
                return this.Value;
            }
            set
            {
                this.Value = (T)value;
            }
        }

        public event PropertyChangedEventHandler<T> OnChange
        {
            add
            {
                PropertyChangedEventHandler wrapper = (object propVal, object oldPropValue) =>
                {
                    value((T)propVal, (T)oldPropValue);
                };
                this.handlerMappings.Add(value, wrapper);
                this._OnChange += wrapper;
            }
            remove
            {
                PropertyChangedEventHandler wrapper = this.handlerMappings[value];
                if (wrapper != null) { 
                    this._OnChange -= wrapper;
                }
            }
        }

        event PropertyChangedEventHandler IObservableProperty.OnChange
        {
            add
            {
                this._OnChange += value;
            }
            remove
            {
                this._OnChange -= value;
            }
        }

        public void Readonly()
        {
            isReadOnly = true;
        }
    }
}
