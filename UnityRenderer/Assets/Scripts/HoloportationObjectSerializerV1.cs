using System.IO;
using Holoportation;
using UnityEngine;
using System.Linq;
using System;
using System.IO.Compression;
using System.Runtime.InteropServices;

namespace Assets.Scripts
{
    public static class HoloportationObjectSerializerV1
    {

        const string jpegDecoderDLL = "SimpleFastJPEGDecoder";
        [DllImport(jpegDecoderDLL)]
        public unsafe static extern IntPtr JPEGDecoder_Create();
        [DllImport(jpegDecoderDLL)]
        public unsafe static extern void JPEGDecoder_Decompress(IntPtr pointer, char* encodedDataPointer, int jpegSize, char* outputBuffer);
        [DllImport(jpegDecoderDLL)]
        public unsafe static extern void JPEGDecoder_Destroy(IntPtr pointer);

        public static IntPtr jpegDecoder;

        static HoloportationObjectSerializerV1()
        {
            jpegDecoder = JPEGDecoder_Create();
        }

        public static byte[] Serialize(HoloportObjectStruct toSerialize)
        {
            MemoryStream ms = new MemoryStream();
            int version = 1;
            ms.TryWriteInt(version);
            ms.TryWriteInt(toSerialize.vertexCount);
            ms.TryWriteInt(toSerialize.indexCount);
            ms.TryWriteArrayWithLength(toSerialize.vertexData, toSerialize.vertexCount * 3);
            ms.TryWriteArrayWithLength(toSerialize.indexData, toSerialize.indexData.Length);

            ms.TryWriteArrayWithLength(toSerialize.MJPEGDataSize);
            int sizePerCam = toSerialize.rawMJPEGData.Length / toSerialize.MJPEGDataSize.Length;
            for (int i = 0; i < toSerialize.MJPEGDataSize.Length; i++)
            {
                ms.TryWriteArrayWithLength(toSerialize.rawMJPEGData, toSerialize.MJPEGDataSize[i], i * sizePerCam);
            }
            return ms.ToArray();
        }

        public static void Deserialize(HoloportObjectStruct into, byte[] data)
        {
            MemoryStream ms = new MemoryStream(data);
            int version;
            if (!ms.TryReadInt(out version, "version"))
            {
                Debug.LogError("Could not read the version property from the data");
            }
            if (version == 1)
            {
                if (!ms.TryReadInt(out into.vertexCount))
                {
                    Debug.LogError("Could not read vertexCount from serialized data");
                    return;
                }
                if (!ms.TryReadInt(out into.indexCount))
                {
                    Debug.LogError("Could not read indexCount from serialized data");
                    return;
                }
                byte[] newVertData;
                if (!ms.TryReadArrayWithLength(out newVertData))
                {
                    Debug.LogError("Could not read vertexData from serialized data");
                    return;
                }
                Buffer.BlockCopy(newVertData, 0, into.vertexData, 0, newVertData.Length);

                byte[] newIndexData;
                if (!ms.TryReadArrayWithLength(out newIndexData))
                {
                    Debug.LogError("Could not read indexData from serialized data");
                    return;
                }
                Buffer.BlockCopy(newIndexData, 0, into.indexData, 0, newIndexData.Length);

                if (!ms.TryReadArrayWithLength(out into.MJPEGDataSize))
                {
                    Debug.LogError("Could not read MJPEGDataSize from serialized data");
                    return;
                }

                int bytesPerCamera = into.rawMJPEGData.Length / into.MJPEGDataSize.Length;
                byte[] compressedBuffer = new byte[bytesPerCamera];
                byte[] decompressedDataBuffer = new byte[bytesPerCamera];
                for (int i = 0; i < into.MJPEGDataSize.Length; i++)
                {
                    // Read the jpeg encoded image
                    byte[] rawData;
                    if (!ms.TryReadArrayWithLength(out rawData))
                    {
                        Debug.Log($"Could not read cam {i} jpeg data");
                        return;
                    }

                    // The rawMJPEGData field is packed with all camera images
                    Buffer.BlockCopy(rawData, 0, into.rawMJPEGData, bytesPerCamera * i, into.MJPEGDataSize[i]);

                    // Decompress the image
                    int jpegSize = into.MJPEGDataSize[i];
                    Buffer.BlockCopy(into.rawMJPEGData, compressedBuffer.Length * i, compressedBuffer, 0, compressedBuffer.Length);
                    DecodeRawImageData(compressedBuffer, jpegSize, decompressedDataBuffer);

                    // Pack the decompressed image into the raw colorData field
                    Buffer.BlockCopy(decompressedDataBuffer, 0, into.colorData, decompressedDataBuffer.Length * i, decompressedDataBuffer.Length);
                }
            }
            else
            {
                Debug.LogError($"Incompatible version number in the data {version}");
                return;
            }
        }

        /// <summary>
        /// Decodes the raw image data from the color frame
        /// </summary>
        /// <param name="data"></param>
        /// <param name="index"></param>
        public static unsafe void DecodeRawImageData(byte[] compressed, int jpegSize, byte[] output)
        {
            fixed (void* compressedColorBufferPtr = compressed)
            fixed (void* outputBufferPtr = output)
            {
                JPEGDecoder_Decompress(jpegDecoder, ((char*)compressedColorBufferPtr), jpegSize, (char*)outputBufferPtr);
            }
        }
    }

}
