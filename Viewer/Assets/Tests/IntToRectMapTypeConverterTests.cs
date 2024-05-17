using Assets.Scripts.Common;
using NUnit.Framework;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Assets.Tests
{
    public class IntToRectMapTypeConverterTests
    {
        [Test]
        public void ConvertsSingleItemMap()
        {
            var converter = CreateInstance();
            var converted = converter.ConvertFromString(
                "(123, (5, 6, 7, 8))",
                null
            ) as IReadOnlyDictionary<int, Rect>;

            Assert.AreEqual(1, converted.Count);
            Assert.AreEqual(123, converted.Keys.First());
            Assert.AreEqual(new Rect(5, 6, 7, 8), converted.Values.First());
        }

        [Test]
        public void ConvertsMultipleItemMap()
        {
            var converter = CreateInstance();
            var converted = converter.ConvertFromString(
                "((123, (1, 2, 3, 4)), (567, (-1, -2, -3, -4)))",
                null
            ) as IReadOnlyDictionary<int, Rect>;

            Assert.AreEqual(2, converted.Count);
            Assert.AreEqual(123, converted.Keys.First());
            Assert.AreEqual(new Rect(1, 2, 3, 4), converted.Values.First());

            Assert.AreEqual(567, converted.Keys.Skip(1).First());
            Assert.AreEqual(new Rect(-1, -2, -3, -4), converted.Values.Skip(1).First());
        }
        [Test]
        public void ConvertsMultipleItemMapWithNewLines()
        {
            var converter = CreateInstance();
            var converted = converter.ConvertFromString(
@"(
    (
        123, 
        (1, 2, 3, 4)
    ), 
    (
        567, 
        (-1, -2, -3, -4)
    )
)",
                null
            ) as IReadOnlyDictionary<int, Rect>;

            Assert.AreEqual(2, converted.Count);
            Assert.AreEqual(123, converted.Keys.First());
            Assert.AreEqual(new Rect(1, 2, 3, 4), converted.Values.First());

            Assert.AreEqual(567, converted.Keys.Skip(1).First());
            Assert.AreEqual(new Rect(-1, -2, -3, -4), converted.Values.Skip(1).First());
        }

        [Test]
        public void ConvertsMultipleItemMapToString()
        {
            string expected = "((123,(1,2,3,4)),(567,(-1,-2,-3,-4)))";
            var converter = CreateInstance();
            var converted = converter.ConvertToString(
                new KeyValuePair<int, Rect>[]
                {
                    new KeyValuePair<int, Rect>(123, new Rect(1, 2, 3, 4)),
                    new KeyValuePair<int, Rect>(567, new Rect(-1, -2, -3, -4)),

                }.AsEnumerable()
            );

            Assert.AreEqual(expected, converted);
        }

        private IntToRectMapTypeConverter CreateInstance()
        {
            return new IntToRectMapTypeConverter();
        }
    }
}
