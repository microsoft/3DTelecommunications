using System.Collections;
using System.Collections.Generic;
using UnityEngine;
#if !UNITY_EDITOR && UNITY_WSA
using System.Threading.Tasks;
using Windows.Storage;
using Windows.Storage.Streams;
using System;
#endif
public class FileTest : MonoBehaviour {

	// Use this for initialization
	void Start ()
    {
#if !UNITY_EDITOR && UNITY_WSA
         Task<Task> task = Task<Task>.Factory.StartNew(
                            async () =>
                            {
                                   Windows.Storage.StorageFile sampleFile = await Windows.Storage.StorageFile.GetFileFromPathAsync("//research/root/peabody/Calibrations/_sf/calib.txt");
                                   string text = await Windows.Storage.FileIO.ReadTextAsync(sampleFile);
        Debug.Log(text);
                                   //allLines = text.Split(separators, StringSplitOptions.None);
                            });
            task.Wait();
            task.Result.Wait();
        //Create the file
      

        //string[] allLines = File.ReadAllLines(generalCalibrationPath);
        //Windows.Storage.StorageFolder storageFolder = Windows.Storage.ApplicationData.Current.LocalFolder;
        //Windows.Storage.StorageFile sampleFile = await storageFolder.GetFileAsync(generalCalibrationPath);

#endif
    }

    // Update is called once per frame
    void Update () {
		
	}
}
