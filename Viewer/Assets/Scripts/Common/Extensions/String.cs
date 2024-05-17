using System.Linq;

namespace Assets.Scripts.Common.Extensions
{
    /// <summary>
    /// Extensions related to strings
    /// </summary>
    public static class StringExtensions
    {
        private static char[] whiteSpaceChars = new char[] { '\t', '\n', '\r', ' ' };

        /// <summary>
        /// Trims whitespace from both ends of the string
        /// </summary>
        /// <param name="str">The string to trim</param>
        /// <param name="additional">The additional characters to treat as whitespace</param>
        /// <returns></returns>
        public static string TrimWhiteSpace(this string str, params char[] additional)
        {
            char[] newParams = additional.Length > 0 ? whiteSpaceChars.Concat(additional).ToArray() : whiteSpaceChars;
            return str.Trim(newParams);
        }
    }
}
