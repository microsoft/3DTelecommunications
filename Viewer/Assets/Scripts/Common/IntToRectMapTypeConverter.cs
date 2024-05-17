using Assets.Scripts.Common.Extensions;
using Assets.Scripts.Viewer.Common;
using SharpConfig;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using UnityEngine;

namespace Assets.Scripts.Common
{
    public class IntToRectMapTypeConverter : TypeStringConverter<IReadOnlyDictionary<int, Rect>>
    {
        private static IReadOnlyDictionary<int, Rect> emptyDictionary = new ReadOnlyDictionary<int, Rect>(new Dictionary<int, Rect>());

        public override string ConvertToString(object value)
        {
            var entries = value as IEnumerable<KeyValuePair<int, Rect>>;
            if (entries != null)
            {
                StringBuilder sb = new StringBuilder();
                sb.Append("(");
                sb.Append(
                    $"({string.Join("),(", entries.Select(n => $"{n.Key},{Utils.RectToString(n.Value)}"))})"
                );
                sb.Append(")");
                return sb.ToString();
            }
            return string.Empty;
        }

        public object ConvertFromString(string value, Type hint)
        {
            value = value.Trim();
            if (!string.IsNullOrWhiteSpace(value))
            {
                // Trim off leading '(', and trailing ')'
                value = value.Substring(1, value.Length - 2);
                Dictionary<int, Rect> dictionary = new Dictionary<int, Rect>();
                string[] entries = Regex.Split(value, "(\\s*\\(\\s*\\d+\\s*,\\s*\\([^\\)]+\\)\\s*\\),?\\s*)");
                foreach (string entry in entries)
                {
                    if (!string.IsNullOrWhiteSpace(entry))
                    {
                        string[] keyValuePair = entry.TrimWhiteSpace(',', '(', ')').Split(',');

                        // key, x, y, z, w
                        if (keyValuePair.Length == 5)
                        {
                            dictionary.Add(
                                int.Parse(keyValuePair[0]), 
                                new Rect(
                                    float.Parse(keyValuePair[1].TrimWhiteSpace('(', ')')),
                                    float.Parse(keyValuePair[2].TrimWhiteSpace('(', ')')),
                                    float.Parse(keyValuePair[3].TrimWhiteSpace('(', ')')),
                                    float.Parse(keyValuePair[4].TrimWhiteSpace('(', ')'))
                                )
                            );
                        } 
                        else
                        {
                            throw new FormatException($"Could not parse: {entry} as a KeyValuePair<int, Rect>");
                        }
                    }
                }
                return new ReadOnlyDictionary<int, Rect>(dictionary);
            }
            return emptyDictionary;
        }
        public override object TryConvertFromString(string value, Type hint)
        {
            try
            {
                return ConvertFromString(value, hint);
            }
            catch (Exception e)
            {
                throw e;
            }
        }
    }
}
