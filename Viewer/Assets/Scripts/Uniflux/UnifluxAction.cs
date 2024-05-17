namespace Assets.Scripts.Uniflux
{
    public class UnifluxAction<T, P>
    {
        public UnifluxAction(T type)
        {
            this.actionType = type;
        }
        public UnifluxAction(T type, P payload): this(type)
        {
            this.payload = payload;
        }
        public UnifluxAction(T type, P payload, bool logInProd) : this(type, payload)
        {
            this.logInProduction = logInProd;
        }

        public T actionType;
        public P payload;

        /**
         * True if this action should be logged in production
         */
        public bool logInProduction = true;
    }
}
