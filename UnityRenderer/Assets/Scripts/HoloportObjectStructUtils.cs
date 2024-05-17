using MIConvexHull;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace Assets.Scripts
{
    public static class HoloportObjectStructUtils
    {

        /// <summary>
        /// Converts the holoport object struct into an .OBJ string
        /// </summary>
        /// <param name="objStruct">The struct to conver to an OBJ string</param>
        /// <param name="writeUVs">If true, UVs will be written</param>
        /// <returns>The OBJ string</returns>
        public static string HoloportModelToOBJString(HoloportObjectStruct objStruct, bool writeUVs = true, string name = null)
        {
            return HoloportModelToOBJString(objStruct, Quaternion.identity, Vector3.zero, writeUVs, name);
        }

        /// <summary>
        /// Converts the holoport object struct into an .OBJ string
        /// </summary>
        /// <param name="objStruct">The struct to conver to an OBJ string</param>
        /// <param name="correctionQ">Rotation correction</param>
        /// <param name="correctionP">Position correction</param>
        /// <param name="writeUVs">If true, UVs will be written</param>
        /// <returns>The OBJ string</returns>
        public static string HoloportModelToOBJString(HoloportObjectStruct objStruct, Quaternion correctionQ, Vector3 correctionP, bool writeUVs = true, string name = null)
        {
            StringBuilder meshString = new StringBuilder(8000000);
            string meshName = name != null ? name : "HoloportOutput";
            meshString.Append("#" + meshName + ".obj"
                                + "\n#" + System.DateTime.Now.ToLongDateString()
                                + "\n#" + System.DateTime.Now.ToLongTimeString()
                                + "\n#-------"
                                + "\n\n");
            meshString.Append("g ").Append(meshName).Append("\n");
            meshString.Append("#" + meshName
                           + "\n#-------"
                           + "\n");

            meshString.Append($"mtllib ./{meshName}.mtl\n");

            const float cmToMRatio = 0.1f;
            int numVertices = 0;
            //1. output vertices
            for (int i = 0; i < objStruct.vertexCount; ++i)
            {
                //int vertID = objStruct.indexData[i];
                ref Vector3 v = ref objStruct.meshVertices[i];
                v = (correctionP * 100 + correctionQ * v) * cmToMRatio;
                numVertices++;
                //meshString.Append(string.Format("v {0} {1} {2}\n", v.x, v.y, v.z));
                meshString.Append("v ");
                meshString.Append(v.x.ToString());
                meshString.Append(" ");
                meshString.Append(v.y.ToString());
                meshString.Append(" ");
                meshString.Append(v.z.ToString());
                meshString.Append("\n");
            }
            meshString.Append("\n");
            //2. output normals
            for (int i = 0; i < objStruct.vertexCount; ++i)
            {
                // int vertID = objStruct.indexData[i];
                ref Vector3 v = ref objStruct.meshNormals[i];
                //  meshString.Append(string.Format("vn {0} {1} {2}\n", -v.x, v.y, v.z));

                meshString.Append("vn ");
                meshString.Append((-v.x).ToString());
                meshString.Append(" ");
                meshString.Append(v.y.ToString());
                meshString.Append(" ");
                meshString.Append(v.z.ToString());
                meshString.Append("\n");
            }
            meshString.Append("\n");
            if (writeUVs)
            {
                //3. output UV
                for (int i = 0; i < objStruct.indexCount; ++i)
                {
                    //  meshString.Append(string.Format("vt {0} {1}\n", objStruct.genMeshUVOBJ[2 * i], objStruct.genMeshUVOBJ[2 * i + 1]));
                    meshString.Append("vt ");
                    meshString.Append(objStruct.uvData[2 * i].ToString());
                    meshString.Append(" ");
                    meshString.Append(objStruct.uvData[2 * i + 1]);
                    meshString.Append("\n");
                }
            }


            //4. output material + triangle indices
            // for (int material = 0; material < m.subMeshCount; material++)
            {
                meshString.Append("\n");
                meshString.Append("usemtl ").Append(name != null ? name : "Default-Material").Append("\n");
                meshString.Append("usemap ").Append(name != null ? name : "Default-Material").Append("\n");

                // int triangleCount = indexCount / 3;
                for (int i = 0; i < objStruct.indexCount; i += 3)
                {
                    if (writeUVs)
                    {
                        //meshString.Append(string.Format("f {0}/{3} {1}/{4} {2}/{5}\n",

                        //objStruct.indexData[i] + 1,
                        //objStruct.indexData[i + 1] + 1,
                        //objStruct.indexData[i + 2] + 1,

                        //i + 1,
                        //(i + 1) + 1,
                        //(i + 2) + 1));  // +1 to change from 0 index to 1 index 


                        meshString.Append("f ");
                        meshString.Append((objStruct.indexData[i] + 1).ToString());
                        meshString.Append("/");
                        meshString.Append((i + 1).ToString());
                        meshString.Append(" ");
                        meshString.Append((objStruct.indexData[i + 1] + 1).ToString());
                        meshString.Append("/");
                        meshString.Append((i + 2).ToString());
                        meshString.Append(" ");
                        meshString.Append((objStruct.indexData[i + 2] + 1).ToString());
                        meshString.Append("/");
                        meshString.Append((i + 3).ToString());
                        meshString.Append("\n");
                    }
                    else
                    {
                        meshString.Append(string.Format("f {0} {1} {2}\n",

                        objStruct.indexData[i] + 1,
                        objStruct.indexData[i + 1] + 1,
                        objStruct.indexData[i + 2] + 1));  // +1 to change from 0 index to 1 index 
                    }
                }
            }

            return meshString.ToString();
        }
        
        //generate either rasterize or sample UV. Different to avoid seams 
        public static void GenerateTriBlockMeshUVs(bool sampleUV, float[] uvArray, int indexCount, int mTextureDimension, int mPerTriangleTextureDimension)
        {
            //Generate UV for triangle block UV map
            int trianglesPerColumn = mTextureDimension / mPerTriangleTextureDimension;
            int triangleCount = indexCount / 3;

            bool oddTriangle = triangleCount % 2 == 1;
            int uvArraySize = triangleCount * HoloportObjectStruct.cIndicesPerTriangle * HoloportObjectStruct.cUVComponentsPerIndex;
            float[] offSets =
            {
                0.0f,0,
                0,0,
                0.0f,0,
                -0.0f, 1.0f,
                -1.0f, 0.0f,
                -0.0f, 0.0f,
            };

            if (!sampleUV)
            {
                for (int i = 0; i < 12; ++i)
                {
                    offSets[i] = 0;
                }
            }
            for (int i = 0; i < triangleCount; i += 2)
            {
                int rectID = i / 2;
                int yID = rectID / trianglesPerColumn;
                int xID = rectID % trianglesPerColumn;

                int yOffset = yID * (mPerTriangleTextureDimension);
                int xOffset = xID * (mPerTriangleTextureDimension);
                //triangle 0 
                uvArray[12 * rectID] = (float)(xOffset + 0 + offSets[0]) / (float)mTextureDimension;
                uvArray[12 * rectID + 1] = (float)(yOffset + 0 + offSets[1]) / (float)mTextureDimension;

                uvArray[12 * rectID + 2] = (float)(xOffset + mPerTriangleTextureDimension + offSets[2]) / (float)mTextureDimension;
                uvArray[12 * rectID + 3] = (float)(yOffset + 0 + offSets[3]) / (float)mTextureDimension;

                uvArray[12 * rectID + 4] = (float)(xOffset + mPerTriangleTextureDimension + offSets[4]) / (float)mTextureDimension;
                uvArray[12 * rectID + 5] = (float)(yOffset + mPerTriangleTextureDimension + offSets[5]) / (float)mTextureDimension;


                if (oddTriangle && 12 * rectID + 6 >= uvArraySize)
                {
                    break;
                }
                //triangle 1                   
                uvArray[12 * rectID + 6] = (float)(xOffset + 0 + offSets[6]) / (float)mTextureDimension;
                uvArray[12 * rectID + 7] = (float)(yOffset + 0 + offSets[7]) / (float)mTextureDimension;

                uvArray[12 * rectID + 8] = (float)(xOffset + mPerTriangleTextureDimension + offSets[8]) / (float)mTextureDimension;
                uvArray[12 * rectID + 9] = (float)(yOffset + mPerTriangleTextureDimension + offSets[9]) / (float)mTextureDimension;

                uvArray[12 * rectID + 10] = (float)(xOffset + 0 + offSets[10]) / (float)mTextureDimension;
                uvArray[12 * rectID + 11] = (float)(yOffset + mPerTriangleTextureDimension + offSets[11]) / (float)mTextureDimension;
            }
        }

        public static void DecodeHalfDataIntoFloatData_OnlyVertices(int[] packedData, Vector3[] vertices, int vCount)
        {
            //decode packedData into vertices and IGNORING normals
            for (int i = 0; i < vCount; ++i)
            {
                int newIndex = i * 3; // 3 ushorts contain the position + normal 
                int packedHalf = packedData[newIndex++];

                ushort currNum = (ushort)(packedHalf & 0xFFFF);
                vertices[i].x = System.HalfHelper.HalfToSingle(Half.ToHalf(currNum));
                currNum = (ushort)((packedHalf >> 16) & 0xFFFF);
                vertices[i].y = System.HalfHelper.HalfToSingle(Half.ToHalf(currNum));

                packedHalf = packedData[newIndex++];
                currNum = (ushort)(packedHalf & 0xFFFF);
                vertices[i].z = System.HalfHelper.HalfToSingle(Half.ToHalf(currNum));
            }
        }

        public static void DecodeHalfDataIntoFloatData(int[] packedData, Vector3[] vertices, Vector3[] normals, int vCount)
        {
            //decode packedData into vertices and normals
            for (int i = 0; i < vCount; ++i)
            {
                int newIndex = i * 3;
                int packedHalf = packedData[newIndex++];
                Vector3 position, normal;
                ushort currNum = (ushort)(packedHalf & 0xFFFF);
                position.x = System.HalfHelper.HalfToSingle(Half.ToHalf(currNum));
                currNum = (ushort)((packedHalf >> 16) & 0xFFFF);
                position.y = System.HalfHelper.HalfToSingle(Half.ToHalf(currNum));

                packedHalf = packedData[newIndex++];
                currNum = (ushort)(packedHalf & 0xFFFF);
                position.z = System.HalfHelper.HalfToSingle(Half.ToHalf(currNum));
                currNum = (ushort)((packedHalf >> 16) & 0xFFFF);
                normal.x = System.HalfHelper.HalfToSingle(Half.ToHalf(currNum));

                packedHalf = packedData[newIndex];
                currNum = (ushort)(packedHalf & 0xFFFF);
                normal.y = System.HalfHelper.HalfToSingle(Half.ToHalf(currNum));
                currNum = (ushort)((packedHalf >> 16) & 0xFFFF);
                normal.z = System.HalfHelper.HalfToSingle(Half.ToHalf(currNum));
                vertices[i] = position;
                normals[i] = normal;
            }
        }

        public static void DecodeUVDataIntoVector2Data(float[] uvData, Vector2[] meshUVs, int iCount)
        {
            for (int i = 0; i < iCount; i += 2)
            {
                meshUVs[i / 2].x = uvData[i];
                meshUVs[i / 2].x = uvData[i + 1];
            }
        }
    }
}
