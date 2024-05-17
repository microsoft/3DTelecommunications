using Assets.Scripts;
using System;
using System.IO;
using UnityEngine;
using Holoportation;

public class HoloportObjectStruct
{
    public DateTimeOffset processStartTime;

    //geometry
    public int[] vertexData;
    public float[] uvData; //genMeshUVs// generated UV based on Soucy paper
    public int[] indexData;
    public int vertexCount;
    public int indexCount;

    //for debugging only
    public Vector3[] meshVertices;
    public Vector3[] meshNormals;
    public Vector2[] meshUVs;

    //texture
    public byte[] rawMJPEGData; // fetched MJPEG data for sending downstream without reencoding
    public int[] MJPEGDataSize; 
    //color 
    public byte[] colorData; // bayered data coming from fusion
    //audio
    public byte[] audioData;
    public int[] audioDataSize;

    public bool processed;


    // Timing debug
    public System.Diagnostics.Stopwatch currStopWatch;

    //buffers
    public byte[] preAllocatedRenderTextureFetchArray_BYTE;

    public System.Drawing.Bitmap destinationTextureBitmap;

    //for streaming
    public ushort[] positionXYZ_ushort;
    public ushort[] indices_ushort;

    //settings for hte objStruct
    public int textureDim;
    public int perTriangleTextureDim;


    //CONSTs 
    public const int cUVComponentsPerIndex = 2; // u,v per index
    public const int cIndicesPerTriangle = 3; // each triangle has 3 indices 
    public const int cColorChannelsPerPixel = 4; // ARGB so 4 
    public const int cXYZComponentsPerVertex = 3; // Each vertex has XYZ 

    public void CopyInfo(HoloportObjectStruct anotherObj)
    {
        vertexCount = anotherObj.vertexCount;
        indexCount = anotherObj.indexCount;
    }

    public static HoloportObjectStruct CreateEmpty()
    {
        return new HoloportObjectStruct();
    }

    /// <summary>
    /// Creates a holoport object struct using the given settings
    /// </summary>
    /// <param name="name">The name of the holoport object struct</param>
    /// <param name="settings">The settings to use</param>
    /// <returns></returns>
    public static HoloportObjectStruct CreateWithSettings(int id, SettingsManager settings)
    {
        return new HoloportObjectStruct(
            id,
            settings.NumPods,
            settings.ColorImageWidth,
            settings.ColorImageHeight,
            settings.ColorPixelBytes,
            settings.AudioSampleRate,
            settings.AudioSampleSize,
            settings.ExpectedIndexCountHQ,
            settings.ExpectedVertexCountHQ,
            settings.TextureDimensionHQ,
            settings.PerTriangleTextureDimensionHQ
        );
    }

    private HoloportObjectStruct()
    {
    }
 
    private HoloportObjectStruct(
        int id, 
        int numPods,
        int imageWidth,
        int imageHeight,
        int imageByteCount,
        int audioSampleRate,
        int audioSampleSize, 
        int expectedIndexCount, 
        int expectedVertexCount, 
        int mTextureDimension, 
        int mPerTriangleTextureDim
    )
    {
        indexCount = expectedIndexCount;
        vertexCount = expectedVertexCount;
        uvData = new float[expectedIndexCount * cUVComponentsPerIndex];
        vertexData = new int[cIndicesPerTriangle * expectedVertexCount];  // (3 components) * (vertex + normal) * (packed into half;0.5) = 3
        indexData = new int[expectedIndexCount];
        colorData = new byte[numPods * imageWidth * imageHeight * imageByteCount];
        rawMJPEGData = new byte[numPods * imageWidth * imageHeight * imageByteCount];
        MJPEGDataSize = new int[numPods];

        textureDim = mTextureDimension;
        perTriangleTextureDim = mPerTriangleTextureDim;

        HoloportObjectStructUtils.GenerateTriBlockMeshUVs(false, uvData, expectedIndexCount, textureDim, perTriangleTextureDim);

        preAllocatedRenderTextureFetchArray_BYTE = new byte[mTextureDimension * mTextureDimension * cColorChannelsPerPixel];
        destinationTextureBitmap = new System.Drawing.Bitmap(mTextureDimension, mTextureDimension, System.Drawing.Imaging.PixelFormat.Format32bppArgb);
        meshVertices = new Vector3[expectedVertexCount];
        meshNormals = new Vector3[expectedVertexCount];
        this.ID = id;

        positionXYZ_ushort = new ushort[expectedVertexCount * cXYZComponentsPerVertex];
        indices_ushort = new ushort[expectedIndexCount];

        audioData = new byte[audioSampleRate * audioSampleSize * numPods];// maximum size assuming we get 1 sec worth of samples, and one stream per pod (overkill but safe)
        audioDataSize = new int[numPods];

        for (int p = 0; p < numPods; ++p)
        {
            audioDataSize[p] = 0; //defaulting to 0 means no audio 
        }

        processed = true;
    }

    /// <summary>
    /// Gets the id of this struct
    /// </summary>
    public int ID
    {
        get;
        private set;
    }

    /// <summary>
    /// Converts the holoport object struct into an .OBJ string
    /// </summary>
    /// <param name="writeUVs">If true, UVs will be written</param>
    /// <returns>The OBJ string</returns>
    public string ToOBJString(bool writeUVs = true, string name = null)
    {
        return HoloportObjectStructUtils.HoloportModelToOBJString(this, writeUVs, name);
    }

    /// <summary>
    /// Converts the holoport object struct into an .OBJ string
    /// </summary>
    /// <param name="writeUVs">If true, UVs will be written</param>
    /// <param name="correctionQ">Rotation correction</param>
    /// <param name="correctionP">Position correction</param>
    /// <returns>The OBJ string</returns>
    public string ToOBJString(Quaternion correctionQ, Vector3 correctionP, bool writeUVs = true, string name = null)
    {
        return HoloportObjectStructUtils.HoloportModelToOBJString(this, correctionQ, correctionP, writeUVs, name);
    }

    /// <summary>
    /// Serializes this instance of the struct
    /// </summary>
    /// <returns></returns>
    public byte[] Serialize()
    {
        return HoloportationObjectSerializerV1.Serialize(this);
    }

    /// <summary>
    /// Deserializes the data into this instance
    /// </summary>
    /// <returns></returns>
    public void Deserialize(byte[] data)
    {
        HoloportationObjectSerializerV1.Deserialize(this, data);
    }
}
