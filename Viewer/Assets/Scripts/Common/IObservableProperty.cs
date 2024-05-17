using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Assets.Scripts.Common
{
    public delegate void PropertyChangedEventHandler<T>(T value, T oldValue);
    public delegate void PropertyChangedEventHandler(object propertyValue, object oldValue);



    public interface IObservableProperty
    {
        event PropertyChangedEventHandler OnChange;

        string Name
        {
            get;
        }

        object Value
        {
            get;
            set;
        }

        void Readonly();
    }

    public interface IObservableProperty<T>
    {
        event PropertyChangedEventHandler<T> OnChange;
        string Name
        {
            get;
        }

        T Value
        {
            get;
            set;
        }

        void Readonly();
    }
}
