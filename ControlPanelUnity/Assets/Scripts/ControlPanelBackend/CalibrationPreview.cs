using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using PeabodyNetworkingLibrary;

public class CalibrationPreview : MonoBehaviour
{
    public float calibToUnityScaleChange = 100.0f;
    public GameObject cameraModelPrefab;
    public GameObject originPrefab;

    public Transform cam0OriginPose;
    public List<GameObject> cameraRepresentations;
    public GameObject placeHolderGO;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void CreateCamRepresentations(List<CameraCalibration> allCalibs)
    {
        //turn off the preview 
        if (placeHolderGO != null)
        {
            placeHolderGO.SetActive(false);
        }

        if (cameraRepresentations != null)
        {
            for(int i = 0; i < cameraRepresentations.Count; ++i)
            {
                Destroy(cameraRepresentations[i]);
            }
            cameraRepresentations.Clear();
        }
        else
        {
            cameraRepresentations = new List<GameObject>();
        }
        
        for(int i = 0; i < allCalibs.Count; ++i)
        {
            GameObject currCamRep = CreateCameraBasedOnCalib(allCalibs[i]);
            currCamRep.transform.parent = transform;
            cameraRepresentations.Add(currCamRep);
        }
        
        Quaternion rotationToCam0AsOrigin = Quaternion.FromToRotation(cameraRepresentations[0].transform.forward, cam0OriginPose.transform.forward);      
        transform.rotation = rotationToCam0AsOrigin * transform.rotation;

        Vector3 translationToCam0AsOrigin = cam0OriginPose.transform.position - cameraRepresentations[0].transform.position;
        transform.position = translationToCam0AsOrigin + transform.position;
        
    }
    public GameObject CreateCameraBasedOnCalib(CameraCalibration cc)
    {
        Vector3 pos = new Vector3(cc.Translation.x, cc.Translation.y, cc.Translation.z);
        pos /= calibToUnityScaleChange;
        //rodrigues.x = -rodrigues.x;
        Quaternion rot = RodriguesToQuaternion(new Vector3(cc.Rotation.x, cc.Rotation.y, cc.Rotation.z));
        //pos = -(rot * pos);
        GameObject currCamRep = Instantiate(cameraModelPrefab);
        currCamRep.name = "Cam_" + cc.Name;
        currCamRep.transform.position = pos;
        currCamRep.transform.rotation = rot;
        currCamRep.transform.localScale = cameraModelPrefab.transform.lossyScale;
        return currCamRep;
    }

    //also implicitly does lhs->rhs conversion
    public static Quaternion RodriguesToQuaternion(Vector3 inputRod)
    {

        Vector3 rodrigues = new Vector3(inputRod.x, -inputRod.y, -inputRod.z);
        float magnitude = rodrigues.magnitude;
        if(magnitude > 0)
        {
            //rotate into cam space, apply rot, then rotate back
            return Quaternion.AngleAxis(180, Vector3.forward) * Quaternion.AngleAxis(magnitude * Mathf.Rad2Deg, rodrigues.normalized) * Quaternion.AngleAxis(180, Vector3.forward);
        }
        else
        {
            //identity
            return new Quaternion(0, 0, 0, 1);
        }
    }
}
