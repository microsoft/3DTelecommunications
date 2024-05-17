using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;
using Newtonsoft.Json;

namespace PeabodyNetworkingLibrary
{
	[System.Serializable]
	public struct SerializableVector3
	{
		/// <summary>
		/// x component
		/// </summary>
		public float x { get; set; }

		/// <summary>
		/// y component
		/// </summary>
		public float y { get; set; }

		/// <summary>
		/// z component
		/// </summary>
		public float z { get; set; }

		/// <summary>
		/// Constructor
		/// </summary>
		/// <param name="rX"></param>
		/// <param name="rY"></param>
		/// <param name="rZ"></param>
		public SerializableVector3(float rX, float rY, float rZ)
		{
			x = rX;
			y = rY;
			z = rZ;
		}

		/// <summary>
		/// Returns a string representation of the object
		/// </summary>
		/// <returns></returns>
		public override string ToString()
		{
			return String.Format("[{0}, {1}, {2}]", x, y, z);
		}

	}

	public class CalibrationContainer
	{
		public string Name { get; set; }
		public List<CameraCalibration> Calibrations { get; set; }
	}

	public class CameraCalibration
	{
		public string Name { get; set; }
		public int Width { get; set; }
		public int Height { get; set; }
		public float Fx { get; set; }
		public float Fy { get; set; }
		public float S { get; set; }
		public float Px { get; set; }
		public float Py { get; set; }
		public float[] Kappa { get; set; }
		public bool IsR6kt { get; set; }
		public float[] RotationMatrix { get; set; }
		public SerializableVector3 Rotation { get; set; } //rodrigues
		public SerializableVector3 Translation { get; set; }
		public SerializableVector3 ColorScale { get; set; }
		public SerializableVector3 ColorBias { get; set; }

		public CameraCalibration()
		{
			Name = "";
			Width = 0;
			Height = 0;
			Fx = 0;
			Fy = 0;
			S = 1;
			Px = 0;
			Py = 0;
			Kappa = new float[8];
			for (int i = 0; i < 8; ++i)
			{
				Kappa[i] = 0;
			}

			IsR6kt = false;
			Rotation = new SerializableVector3(0, 0, 0);
			Translation = new SerializableVector3(0, 0, 0);
			ColorScale = new SerializableVector3(1, 1, 1);
			ColorBias = new SerializableVector3(0, 0, 0);
			RotationMatrix = new float[9];
			//making an identtiy matrix
			for (int i = 0; i < 3; ++i)
			{
				for (int j = 0; j < 3; ++j)
				{
					int index = i * 3 + j;
					if (i == j)
					{
						RotationMatrix[index] = 1;
					}
					else
					{
						RotationMatrix[index] = 0;
					}
				}
			}
		}
	}


	public class CameraCalibrationHelper
	{
		public static void SaveToCalibBACamFiles(CalibrationContainer calibrationContainer, string folderPath)
		{
			// Create the folder path if it doesn't exist
			Directory.CreateDirectory(folderPath);
			foreach (CameraCalibration cc in calibrationContainer.Calibrations)
			{
				int camNum = -1;
				try
				{
					camNum = Convert.ToInt32(cc.Name.Substring(3, 1));
				}
				catch (FormatException)
				{
					Console.WriteLine($"Couldn't parse a camera number from {cc.Name}");
					return;
				}
				string fileName = System.IO.Path.Combine(folderPath, $"calib_Depth{camNum}_ba.txt");
				using (StreamWriter outputFile = new StreamWriter(fileName, false))
				{
					outputFile.WriteLine("{0:D} {1:D}", cc.Width, cc.Height);
					outputFile.WriteLine("{0:F15} 0.00 {1:F15}", cc.Fx, cc.Px);
					outputFile.WriteLine("0.00 {0:F15} {1:F15}", cc.Fy, cc.Py);
					outputFile.WriteLine("0.00 0.00 1.00");
					outputFile.WriteLine("{0:F15} {1:F15} {2:F15}", cc.RotationMatrix[0], cc.RotationMatrix[1], cc.RotationMatrix[2]);
					outputFile.WriteLine("{0:F15} {1:F15} {2:F15}", cc.RotationMatrix[3], cc.RotationMatrix[4], cc.RotationMatrix[5]);
					outputFile.WriteLine("{0:F15} {1:F15} {2:F15}", cc.RotationMatrix[6], cc.RotationMatrix[7], cc.RotationMatrix[8]);
					outputFile.WriteLine("{0:F15}", cc.Translation.x);
					outputFile.WriteLine("{0:F15}", cc.Translation.y);
					outputFile.WriteLine("{0:F15}", cc.Translation.z);
					int numKt = cc.IsR6kt ? 8 : 5;
					for (int i = 0; i < numKt; i++)
					{
						outputFile.WriteLine("{0:F15}", cc.Kappa[i]);
					}
				}
			}
		}
		public static List<CameraCalibration> LoadFromCalibBACamFiles(string folderPath, int podCount)
		{

			List<CameraCalibration> calibs = new List<CameraCalibration>();

			for (int fileNum = 0; fileNum < podCount; fileNum++)
			{
				string filePath = string.Format("{0}\\calib_Depth{1}_ba.txt", folderPath, fileNum.ToString());


				if (!File.Exists(filePath))
				{
					Console.WriteLine("Error: Couldn't load BA Calib file.");
					return calibs;
				}
				string[] allLines = File.ReadAllLines(filePath);

				string[] lineSplit;

				CameraCalibration cc;
				cc = new CameraCalibration();
				cc.Name = "cam" + fileNum;

				//load w h
				lineSplit = allLines[0].Split(' ');
				cc.Width = int.Parse(lineSplit[0]);
				cc.Height = int.Parse(lineSplit[1]);

				// load 
				// fx 0 px
				// 0 fy py
				// 0 0 0
				lineSplit = allLines[1].Split(' ');
				cc.Fx = float.Parse(lineSplit[0]);
				cc.Px = float.Parse(lineSplit[2]);
				lineSplit = allLines[2].Split(' ');
				cc.Fy = float.Parse(lineSplit[1]);
				cc.Py = float.Parse(lineSplit[2]);

				//load rotation matrix
				const int cRotationStartLine = 4;
				for (int i = 0; i < 3; ++i)
				{
					lineSplit = allLines[cRotationStartLine + i].Split(' ');
					for (int j = 0; j < 3; ++j)
					{
						cc.RotationMatrix[i * 3 + j] = float.Parse(lineSplit[j]);
					}
				}
				// load translation
				cc.Translation = new SerializableVector3(float.Parse(allLines[7]),
											float.Parse(allLines[8]),
											float.Parse(allLines[9]));

				// K = 5: default r3kt (3 radial, and 2 tangential distortion parameters)
				int numK = 5;
				cc.IsR6kt = false;
				if (allLines.Length >= 18)
				{
					// K = 8: r6kt (6 radial, and 2 tangential distortion parameters)
					numK = 8;
					cc.IsR6kt = true;
				}
				const int cIntrinsicsStartLine = 10;
				for (int i = 0; i < numK; ++i)
				{
					cc.Kappa[i] = float.Parse(allLines[cIntrinsicsStartLine + i]);
				}

				calibs.Add(cc);

			}

			return calibs;
		}

		public static void SaveToCalibColorCamsFile(CalibrationContainer calibrationContainer, String filePath)
		{
			Directory.CreateDirectory(filePath);
			string fileName = System.IO.Path.Combine(filePath, "calibColorCams.txt");
			using (StreamWriter outputFile = new StreamWriter(fileName, false))
			{
				outputFile.WriteLine(calibrationContainer.Calibrations.Count());
				foreach (CameraCalibration cameraCalibration in calibrationContainer.Calibrations)
				{
					outputFile.WriteLine("{0} {1:D} {2:D}", cameraCalibration.Name, cameraCalibration.Width, cameraCalibration.Height);
					outputFile.Write("{0:F3} {1:F3} {2:F3} {3:F3} {4:F3} ", cameraCalibration.Fx, cameraCalibration.Fy, cameraCalibration.S, cameraCalibration.Px, cameraCalibration.Py);
					outputFile.Write("{0:F3} {1:F3} {2:F3} {3:F3} {4:F3} ", cameraCalibration.Kappa[0], cameraCalibration.Kappa[1], cameraCalibration.Kappa[2], cameraCalibration.Kappa[3], cameraCalibration.Kappa[4]);
					if (cameraCalibration.IsR6kt)
					{
						outputFile.Write("{0:F3} {1:F3} {2:F3}", cameraCalibration.Kappa[5], cameraCalibration.Kappa[6], cameraCalibration.Kappa[7]);
					}
					outputFile.WriteLine();
					outputFile.Write("{0:F3} {1:F3} {2:F3} ", cameraCalibration.Rotation.x, cameraCalibration.Rotation.y, cameraCalibration.Rotation.z);
					outputFile.WriteLine("{0:F3} {1:F3} {2:F3}", cameraCalibration.Translation.x, -1 * cameraCalibration.Translation.y, cameraCalibration.Translation.z);
					outputFile.Write("{0:F3} {1:F3} {2:F3} ", cameraCalibration.ColorScale.x, cameraCalibration.ColorScale.y, cameraCalibration.ColorScale.z);
					outputFile.WriteLine("{0:F3} {1:F3} {2:F3}", cameraCalibration.ColorBias.x, cameraCalibration.ColorBias.y, cameraCalibration.ColorBias.z);
				}
			}
		}

		public static List<CameraCalibration> LoadFromCalibColorCamsFile(string filePath, List<CameraCalibration> existingList = null)
		{
			List<CameraCalibration> calibs = existingList;
			if (existingList == null)
			{
				calibs = new List<CameraCalibration>();
			}

			string[] allLines = File.ReadAllLines(filePath);

			int currLineIndex = 1;
			//quick sanity checks
			if (allLines.Length <= 0)
			{
				Console.WriteLine("Warning: calib file found but it's empty!");
				return calibs;
			}
			int camNumFromFile = Int32.Parse(allLines[0]);
			for (int i = 0; i < camNumFromFile; ++i)
			{
				bool addCCtoList = false;
				CameraCalibration cc;
				if (calibs.Count > i)
				{
					cc = calibs[i];
				}
				else
				{
					cc = new CameraCalibration();
					addCCtoList = true;
				}

				string[] lineOne = allLines[currLineIndex].Split(' ');
				cc.Name = lineOne[0];
				cc.Width = int.Parse(lineOne[1]);
				cc.Height = int.Parse(lineOne[2]);

				string[] lineTwo = allLines[currLineIndex + 1].Split(' ');
				cc.Fx = float.Parse(lineTwo[0]);
				cc.Fy = float.Parse(lineTwo[1]);
				cc.S = float.Parse(lineTwo[2]);
				cc.Px = float.Parse(lineTwo[3]);
				cc.Py = float.Parse(lineTwo[4]);

				cc.Kappa[0] = float.Parse(lineTwo[5]);
				cc.Kappa[1] = float.Parse(lineTwo[6]);
				cc.Kappa[2] = float.Parse(lineTwo[7]);
				cc.Kappa[3] = float.Parse(lineTwo[8]);
				cc.Kappa[4] = float.Parse(lineTwo[9]);

				if (lineTwo.Length > 10)
				{
					cc.IsR6kt = true;
					cc.Kappa[5] = float.Parse(lineTwo[10]);
					cc.Kappa[6] = float.Parse(lineTwo[11]);
					cc.Kappa[7] = float.Parse(lineTwo[12]);
				}
				else
				{
					cc.IsR6kt = false;
				}

				string[] lineThree = allLines[currLineIndex + 2].Split(' ');
				cc.Rotation = new SerializableVector3(float.Parse(lineThree[0]), float.Parse(lineThree[1]), float.Parse(lineThree[2]));
				cc.Translation = new SerializableVector3(float.Parse(lineThree[3]), -float.Parse(lineThree[4]), float.Parse(lineThree[5]));

				string[] lineFour = allLines[currLineIndex + 3].Split(' ');
				cc.ColorScale = new SerializableVector3(float.Parse(lineFour[0]), float.Parse(lineFour[1]), float.Parse(lineFour[2]));
				cc.ColorBias = new SerializableVector3(float.Parse(lineFour[3]), float.Parse(lineFour[4]), float.Parse(lineFour[5]));

				currLineIndex += 4;

				if (addCCtoList)
				{
					calibs.Add(cc);
				}
			}
			return calibs;
		}

		public static void SaveTo3DTMCalibrationFormat(string json, string calibration_directory)
		{
			Directory.CreateDirectory(calibration_directory);
			string filename = System.IO.Path.Combine(calibration_directory, "calibCameras3DTM.json");

			WriteJSONToFile(json, filename);
		}


		//STATIC serialize /Desrialize methods
		public static CalibrationContainer Deserialize(string JSONText)
		{
			return JsonConvert.DeserializeObject<CalibrationContainer>(JSONText);
		}

		public static string Serialize(CalibrationContainer overallCalib)
		{
			return JsonConvert.SerializeObject(overallCalib);
		}

		public static void JSONStringToBytePacket(string json, out byte[] packetBytes)
		{
			// calibration data format: [int jsonData_size] [char[] jsonData]
			int jsonLength = json.Length;
			packetBytes = new byte[jsonLength + sizeof(int)];
			byte[] lengthInBytes = BitConverter.GetBytes(jsonLength);
			byte[] jsonInBytes = Encoding.Default.GetBytes(json);
			Buffer.BlockCopy(lengthInBytes, 0, packetBytes, 0, sizeof(int));
			Buffer.BlockCopy(jsonInBytes, 0, packetBytes, sizeof(int), jsonLength);
		}

		public static void BytePacketToJSONString(byte[] packetBytes, out string json_str)
		{
			// Skipping event type (first byte) and json length in bytes (int value)
			int jsonLengthInBytes = System.BitConverter.ToInt32(packetBytes, 1);
			json_str = System.Text.Encoding.UTF8.GetString(packetBytes, 1 + sizeof(int), jsonLengthInBytes);
		}


		public static void BytePacketToJSON(byte[] packetBytes, out Newtonsoft.Json.Linq.JObject json)
		{
			string json_str;
			BytePacketToJSONString(packetBytes, out json_str);
			json = Newtonsoft.Json.Linq.JObject.Parse(json_str);
		}

		public static void WriteJSONToFile(string json_str, string filename)
		{
			Newtonsoft.Json.Linq.JObject json = Newtonsoft.Json.Linq.JObject.Parse(json_str);
			// write JSON directly to a file
			using (StreamWriter file = File.CreateText(filename))
			using (JsonTextWriter writer = new JsonTextWriter(file))
			{
				json.WriteTo(writer);
			}
		}
	}
}
