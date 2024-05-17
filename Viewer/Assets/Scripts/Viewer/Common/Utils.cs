using System;
using System.IO;
using UnityEngine;
using UnityEngine.Events;
using System.Linq;
using System.Text;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Net;
using Assets.Scripts.Common.Extensions;
using UnityEngine.EventSystems;

namespace Assets.Scripts.Viewer.Common
{
    public static class Utils
    {
        public static string EnsureDirectory(string directory)
        {
            if (!Directory.Exists(directory))
            {
                Directory.CreateDirectory(directory);
            }
            return directory;
        }

        public static void Timeout(UnityAction callback, int delay = 2000)
        {
            System.Threading.Timer timer = null;

            timer = new System.Threading.Timer(
                (ignore) =>
                {
                    timer.Dispose();
                    callback();
                }, null, delay, 100000);
        }

        public static string EnsureStorageDirectory(string subDirectory)
        {
            string storageDir = Application.persistentDataPath;
            string finalDir = Path.Combine(storageDir, "./", subDirectory);
            Utils.EnsureDirectory(finalDir);
            return finalDir;
        }

        // Extension method?
        public static T AddGameObject<T>(GameObject parent) where T : Component
        {
            GameObject obj = new GameObject();
            T component = obj.AddComponent<T>();
            obj.transform.SetParent(parent.transform);
            return component;
        }

        public static Texture2D MakeTexture2DFromSprite(Sprite sprite)
        {
            Texture2D texture2D = new Texture2D((int)sprite.rect.width, (int)sprite.rect.height, TextureFormat.RGBA32, false);

            Debug.Log(sprite.rect);
#if UNITY_EDITOR
            texture2D.alphaIsTransparency = true;
#endif
            var pixels = sprite.texture.GetPixels((int)sprite.rect.x, (int)sprite.rect.y, (int)sprite.rect.width, (int)sprite.rect.height);
            texture2D.SetPixels(pixels);
            texture2D.Apply();

            return texture2D;
        }

        /// <summary>
        /// Returns the set of game objects under the point
        /// </summary>
        /// <returns></returns>
        public static IEnumerable<GameObject> GetGameObjectsUnderPoint(Vector2 point)
        {
            PointerEventData pointerData = new PointerEventData(EventSystem.current)
            {
                pointerId = -1,
            };

            pointerData.position = point;

            List<RaycastResult> results = new List<RaycastResult>();
            EventSystem.current.RaycastAll(pointerData, results);

            return results.Select(n => n.gameObject);
        }

        public static Vector2 GetHotspotFromSprite(Sprite sprite)
        {
            return sprite.pivot;
        }
        public static byte[] CaptureCameraImagePNG(Camera c)
        {
            if (c != null)
            {
                RenderTexture rt = RenderTexture.GetTemporary(c.pixelWidth, c.pixelHeight, 24);
                RenderTexture prev = c.targetTexture;

                RenderTexture.active = rt;

                c.targetTexture = rt;

                c.Render();

                Texture2D imageTex = new Texture2D(rt.width, rt.height);
                imageTex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
                imageTex.Apply();

                RenderTexture.active = null;

                byte[] result = imageTex.EncodeToPNG();

                c.targetTexture = prev;

                return result;
            }
            return null;
        }
        public static void RenderCameraIntoTexture(Camera c, Texture2D texture)
        {
            if (c != null && texture != null)
            {
                RenderTexture rt = RenderTexture.GetTemporary(c.pixelWidth, c.pixelHeight, 24);
                RenderTexture prev = c.targetTexture;

                RenderTexture.active = rt;

                c.targetTexture = rt;

                c.Render();

                texture.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
                texture.Apply();

                RenderTexture.active = null;

                c.targetTexture = prev;
            }
        }

        // TODO: Textension method?
        public static byte[] EncodeRenderTextureToPng(RenderTexture rt)
        {
            if (rt == null)
            {
                throw new ArgumentNullException("rt");
            }

            RenderTexture prev = RenderTexture.active;

            RenderTexture.active = rt;

            Texture2D imageTex = new Texture2D(rt.width, rt.height);
            imageTex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
            imageTex.Apply();

            byte[] bytes = imageTex.EncodeToPNG();

            RenderTexture.active = prev;
            return bytes;
        }

        public static byte[] CaptureMainCameraImagePNG()
        {
            return CaptureCameraImagePNG(Camera.main);
        }

        public static bool IsDeviceIndependentControlDown()
        {
            return Input.GetKeyDown(KeyCode.LeftControl) || Input.GetKeyDown(KeyCode.RightControl) || Input.GetKeyDown(KeyCode.LeftCommand) || Input.GetKeyDown(KeyCode.RightCommand);
        }

        public static bool IsDeviceIndependentControlUp()
        {
            return Input.GetKeyUp(KeyCode.LeftControl) || Input.GetKeyUp(KeyCode.RightControl) || Input.GetKeyUp(KeyCode.LeftCommand) || Input.GetKeyUp(KeyCode.RightCommand);
        }

        public static int Modulo(int number, int mod)
        {
            return number - mod * Mathf.FloorToInt((float)number / mod);
        }

        public static string Vec3ToString(Vector3 vec3)
        {
            return $"{vec3.x},{vec3.y},{vec3.z}";
        }
        public static string Vec4ToString(Vector4 vec4)
        {
            return $"({vec4.x},{vec4.y},{vec4.z},{vec4.w})";
        }

        public static string RectToString(Rect rect)
        {
            return $"({rect.x},{rect.y},{rect.width},{rect.height})";
        }

        public static Vector3 ParseVec3(string strVec)
        {
            if (strVec != null)
            {
                var components = strVec.Split(',').Select(n => float.Parse(n.Trim())).ToArray();
                if (components.Length == 3)
                {
                    return new Vector3(components[0], components[1], components[2]);
                }
            }
            return Vector3.zero;
        }

        public static Vector4 ParseVec4(string strVec)
        {
            if (strVec != null)
            {
                strVec = strVec.TrimWhiteSpace('(', ')');
                var components = strVec.Split(',').Select(n => float.Parse(n.Trim())).ToArray();
                if (components.Length == 4)
                {
                    return new Vector4(components[0], components[1], components[2], components[3]);
                }
            }
            return Vector3.zero;
        }

        public static void ExportModelOBJ(MeshFilter[] meshes, string folder, string name)
        {
            OutputHelper.OutputLog("Exporting OBJ");

            StringBuilder meshString = new StringBuilder();

            meshString.Append("#" + name + ".obj"
                                + "\n#" + DateTime.Now.ToLongDateString()
                                + "\n#" + DateTime.Now.ToLongTimeString()
                                + "\n#-------"
                                + "\n\n");

            meshString.Append("mtllib ./" + name + ".mtl\n");
            meshString.Append("g ").Append(name).Append("\n");
            meshString.Append("#" + name
                           + "\n#-------"
                           + "\n");
            int totalVertices = 0;
            Dictionary<string, Texture2D> materialsToWrite = new Dictionary<string, Texture2D>();

            foreach (MeshFilter filter in meshes)
            {
                Mesh mesh = filter.mesh;
                Material[] mats = filter.GetComponent<Renderer>().sharedMaterials;

                foreach (Vector3 v in mesh.vertices)
                {
                    meshString.Append(string.Format("v {0} {1} {2}\n", v.x, v.y, v.z));
                }
                meshString.Append("\n");
                foreach (Vector3 v in mesh.normals)
                {
                    meshString.Append(string.Format("vn {0} {1} {2}\n", v.x, v.y, v.z));
                }
                meshString.Append("\n");
                foreach (Vector3 v in mesh.uv)
                {
                    meshString.Append(string.Format("vt {0} {1}\n", v.x, v.y));
                }

                for (int material = 0; material < mesh.subMeshCount; material++)
                {
                    meshString.Append("\n");
                    meshString.Append("usemtl ").Append(mats[material].name).Append("\n");
                    meshString.Append("usemap ").Append(mats[material].name).Append("\n");
                    if (!materialsToWrite.ContainsKey(mats[material].name))
                    {
                        materialsToWrite.Add(mats[material].name, (Texture2D)mats[material].mainTexture);
                    }

                    int[] triangles = mesh.GetTriangles(material);
                    for (int i = 0; i < triangles.Length; i += 3)
                    {
                        meshString.Append(string.Format("f {0}/{0}/{0} {1}/{1}/{1} {2}/{2}/{2}\n",
                            triangles[i] + 1 + totalVertices, triangles[i + 1] + 1 + totalVertices, triangles[i + 2] + 1 + totalVertices));
                    }
                }

                totalVertices += mesh.vertexCount;
            }

            File.WriteAllText(Path.Combine(folder, $"{name}.obj"), meshString.ToString());

            using (StreamWriter sw = new StreamWriter(Path.Combine(folder, name + ".mtl")))
            {
                foreach (KeyValuePair<string, Texture2D> kvp in materialsToWrite)
                {
                    sw.Write("\n");
                    sw.Write("newmtl {0}\n", kvp.Key);
                    sw.Write("Ka  0.0 0.0 0.0\n");
                    sw.Write("Kd  0.0 0.0 0.0\n");
                    sw.Write("Ks  0.0 0.0 0.0\n");
                    sw.Write("d  1.0\n");
                    sw.Write("Ns  0.0\n");
                    sw.Write("illum 2\n");

                    if (kvp.Value != null)
                    {
                        string textureFileName = kvp.Key;


                        int stripIndex = textureFileName.LastIndexOf(Path.PathSeparator);

                        if (stripIndex >= 0)
                            textureFileName = textureFileName.Substring(stripIndex + 1).Trim();

                        textureFileName = textureFileName + ".png";

                        File.WriteAllBytes(Path.Combine(folder, textureFileName), kvp.Value.EncodeToPNG());

                        sw.Write("map_Kd {0}", textureFileName);
                    }

                    sw.Write("\n\n\n");
                }
            }

        }
        public static string GetFirstLocalIP()
        {
            string localIP = null;
            var host = Dns.GetHostEntry(Dns.GetHostName());
            foreach (var ip in host.AddressList)
            {
                if (ip.AddressFamily == AddressFamily.InterNetwork)
                {
                    localIP = ip.ToString();
                    break;
                }
            }

            return localIP;
        }
    }
}
