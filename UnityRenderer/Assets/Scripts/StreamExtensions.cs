using System;
using System.IO;
using System.Text;
using UnityEngine;

namespace Holoportation
{
    public static class StreamExtensions
    {


        /// <summary>
        /// Tries to send an array to the given stream
        /// </summary>
        /// <param name="array">The output array</param>
        public static bool TryWriteString(this Stream stream, string value)
        {
            byte[] valueAsBytes = Encoding.UTF8.GetBytes(value);
            return stream.TryWriteArrayWithLength(valueAsBytes);
        }

        /// <summary>
        /// Tries to send an array to the given stream
        /// </summary>
        /// <param name="array">The output array</param>
        public static bool TryWriteArrayWithLength(this Stream stream, byte[] array)
        {
            return stream.TryWriteArrayWithLength(array, array.Length);
        }

        /// <summary>
        /// Tries to write an int array to the given stream
        /// </summary>
        /// <param name="stream"></param>
        /// <param name="array"></param>
        /// <returns></returns>
        public static bool TryWriteArrayWithLength(this Stream stream, int[] array)
        {
            return stream.TryWriteArrayWithLength(array, array.Length, 0);
        }

        /// <summary>
        /// Tries to write an int array to the given stream
        /// </summary>
        /// <param name="stream"></param>
        /// <param name="array"></param>
        /// <returns></returns>
        public static bool TryWriteArrayWithLength(this Stream stream, int[] array, int length, int offset = 0)
        {
            int indexByteCount = length * sizeof(int);
            byte[] valueAsBytes = new byte[indexByteCount];
            Buffer.BlockCopy(array, 0, valueAsBytes, offset * sizeof(int), indexByteCount);

            return stream.TryWriteArrayWithLength(valueAsBytes);
        }


        /// <summary>
        /// Tries to write an int array to the given stream
        /// </summary>
        /// <param name="stream"></param>
        /// <param name="array"></param>
        /// <returns></returns>
        public static bool TryWriteArrayWithLength(this Stream stream, float[] array)
        {
            return stream.TryWriteArrayWithLength(array, array.Length, 0);
        }

        /// <summary>
        /// Tries to write an int array to the given stream
        /// </summary>
        /// <param name="stream"></param>
        /// <param name="array"></param>
        /// <returns></returns>
        public static bool TryWriteArrayWithLength(this Stream stream, float[] array, int length, int offset = 0)
        {
            int indexByteCount = length * sizeof(float);
            byte[] valueAsBytes = new byte[indexByteCount];
            Buffer.BlockCopy(array, 0, valueAsBytes, offset * sizeof(float), indexByteCount);

            return stream.TryWriteArrayWithLength(valueAsBytes);
        }

        /// <summary>
        /// Tries to send an array to the given stream
        /// </summary>
        /// <param name="array">The output array</param>
        public static bool TryWriteArrayWithLength(this Stream stream, byte[] array, int length, int offset = 0)
        {
            // Write the length of the byte array
            stream.TryWriteInt(length);

            // Write the data
            stream.Write(array, offset, length);

            return true;
        }

        /// <summary>
        /// Tries to send an array to the given stream
        /// </summary>
        /// <param name="array">The output array</param>
        public static bool TryWriteInt(this Stream stream, int value)
        {
            byte[] valueAsBytes = BitConverter.GetBytes(value);

            // Write the length of the byte array
            stream.Write(valueAsBytes, 0, valueAsBytes.Length);

            return true;
        }

        /// <summary>
        /// Tries to receive an array from the given stream
        /// </summary>
        /// <param name="array">The output array</param>
        public static bool TryReadArrayWithLength(this Stream stream, out byte[] array)
        {
            int length;
            return stream.TryReadArrayWithLength(out array, out length);
        }

        /// <summary>
        /// Tries to receive an array from the given stream
        /// </summary>
        /// <param name="array">The output array</param>
        public static bool TryReadArrayWithLength(this Stream stream, out int[] array)
        {
            int length;
            return stream.TryReadArrayWithLength(out array, out length);
        }

        /// <summary>
        /// Tries to receive an array from the given stream
        /// </summary>
        /// <param name="array">The output array</param>
        public static bool TryReadArrayWithLength(this Stream stream, out float[] array)
        {
            int length;
            return stream.TryReadArrayWithLength(out array, out length);
        }

        /// <summary>
        /// Tries to receive an array from the given stream
        /// </summary>
        /// <param name="array">The output array</param>
        public static bool TryReadArrayWithLength(this Stream stream, out byte[] array, out int length)
        {
            array = null;
            if (stream.TryReadInt(out length, "Length"))
            {
                array = stream.ReadNumBytes(length);


                // Is there stuff after this>
                //if (stream.DataAvailable)
                //{
                // Debug.LogWarning("more datas than specified in the length header");
                //}

                return true;
            }
            else
            {
                Debug.Log("Could not receive a length for the array");
                return false;
            }
        }

        /// <summary>
        /// Tries to receive an array from the given stream
        /// </summary>
        /// <param name="array">The output array</param>
        public static bool TryReadArrayWithLength(this Stream stream, out int[] array, out int length)
        {
            array = null;
            int byteLength;
            length = 0;
            if (stream.TryReadInt(out byteLength, "Length"))
            {
                length = byteLength / sizeof(int);

                byte[] byteArr = stream.ReadNumBytes(byteLength);
                array = new int[length];

                Buffer.BlockCopy(byteArr, 0, array, 0, byteLength);

                // Is there stuff after this>
                //if (stream.DataAvailable)
                //{
                // Debug.LogWarning("more datas than specified in the length header");
                //}

                return true;
            }
            else
            {
                Debug.Log("Could not receive a length for the array");
                return false;
            }
        }

        /// <summary>
        /// Tries to receive an array from the given stream
        /// </summary>
        /// <param name="array">The output array</param>
        public static bool TryReadArrayWithLength(this Stream stream, out float[] array, out int length)
        {
            array = null;
            int byteLength;
            length = 0;
            if (stream.TryReadInt(out byteLength, "Length"))
            {
                length = byteLength / sizeof(float);

                byte[] byteArr = stream.ReadNumBytes(byteLength);
                array = new float[length];

                Buffer.BlockCopy(byteArr, 0, array, 0, byteLength);

                // Is there stuff after this>
                //if (stream.DataAvailable)
                //{
                // Debug.LogWarning("more datas than specified in the length header");
                //}

                return true;
            }
            else
            {
                Debug.Log("Could not receive a length for the array");
                return false;
            }
        }

        /// <summary>
        /// Reads the given number of bytes from the stream
        /// </summary>
        /// <param name="stream"></param>
        /// <param name="length"></param>
        /// <returns></returns>
        public static byte[] ReadNumBytes(this Stream stream, int length)
        {
            byte[] array = new byte[length];
            int offset = 0;
            int numBytesRead;
            while ((numBytesRead = stream.Read(array, offset, length - offset)) > 0)
            {
                offset += numBytesRead;
                if (offset >= length)
                {
                    break;
                }
            }
            return array;
        }

        /// <summary>
        /// Tries to receive an array from the given stream
        /// </summary>
        /// <param name="array">The output array</param>
        public static bool TryReadInt(this Stream stream, out int value, string name = "value")
        {
            value = BitConverter.ToInt32(stream.ReadNumBytes(4), 0);
            return true;
        }

        /// <summary>
        /// Tries to receive an array from the given stream
        /// </summary>
        /// <param name="array">The output array</param>
        public static bool TryReadByte(this Stream stream, out byte value, string name = "value")
        {
            int readValue = stream.ReadByte();
            if (readValue < 0)
            {
                Debug.Log("Stream unexpectedly closed");
                value = 0;
                return false;
            }
            value = (byte)readValue;
            return true;
        }

        /// <summary>
        /// Tries to send an array to the given stream
        /// </summary>
        /// <param name="array">The output array</param>
        public static bool TryWriteByte(this Stream stream, byte value)
        {
            stream.WriteByte(value);

            return true;
        }

        public static bool TryReadString(this Stream stream, out string value)
        {
            byte[] strBytes;
            value = null;
            if (stream.TryReadArrayWithLength(out strBytes))
            {
                value = Encoding.UTF8.GetString(strBytes);
                return true;
            }
            return false;
        }
    }
}
