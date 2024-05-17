namespace Assets.Scripts.Common
{

    /// <summary>
    /// A simple key which represents the path to a sharp config setting
    /// </summary>
    public class SettingName: System.IEquatable<SettingName>
    {
        public string Section { get; private set; }
        public string Setting { get; private set; }

        public SettingName(string section, string setting)
        {
            Section = section;
            Setting = setting;
        }

        public static bool operator ==(SettingName key1, SettingName key2)
        {
            return key1.Section == key2.Section && key1.Setting == key2.Setting;
        }

        public static bool operator !=(SettingName key1, SettingName key2) => !(key1 == key2);
        public override bool Equals(object obj)
        {
            if (obj == null || GetType() != obj.GetType())
                return false;

            var setting2 = (SettingName)obj;
            return (Section == setting2.Section && Setting == setting2.Setting);
        }

        public override int GetHashCode()
        {
            return Section.GetHashCode() * Setting.GetHashCode(); 
        }

        public bool Equals(SettingName other)
        {
            throw new System.NotImplementedException();
        }
    }
}
