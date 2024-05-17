using UnityEngine;

namespace Assets.Scripts.Common.Extensions
{
    public static class CameraExtensions
    {
        public static float INCHES_PER_METER = 39.3701f;

        public static void FitIntoView(this Camera camera, GameObject obj)
        {
            Renderer renderer = obj.GetComponent<Renderer>();
            if (renderer != null)
            {
                camera.FitIntoView(renderer.bounds);
            }
            else
            {
                Bounds? bounds = obj.ComputeBounds();
                if (bounds != null)
                {
                    camera.FitIntoView(bounds.Value);
                }
                else
                {
                    camera.transform.position = obj.transform.position - camera.transform.forward;
                }
            }
        }

        public static void FitIntoView(this Camera camera, Bounds bounds, float fudgeFactor = 1.1f)
        {
            var frustumHeight = Mathf.Max(bounds.size.x, bounds.size.y, bounds.size.z);
            var distance = frustumHeight * 0.5f / Mathf.Tan(camera.fieldOfView * 0.5f * Mathf.Deg2Rad);

            // Wiggle room
            distance *= fudgeFactor;

            // Look at it
            camera.transform.LookAt(bounds.center);

            // Line up the camera with the object we're fitting
            // Assume UP is Y
            camera.transform.position = new Vector3(camera.transform.position.x, bounds.center.y, camera.transform.position.z);

            // Move it far enough away to fit it
            camera.transform.position = bounds.center - distance * camera.transform.forward;
        }

        public static void FitToLifesizeFOV(this Camera camera, GameObject obj)
        {
            Renderer renderer = obj.GetComponent<Renderer>();
            if (renderer != null)
            {
                camera.FitToLifesizeFOV(renderer.bounds);
            }
            else
            {
                Bounds? bounds = obj.ComputeBounds();
                if (bounds != null)
                {
                    camera.FitToLifesizeFOV(bounds.Value);
                }
                else
                {
                    camera.transform.position = obj.transform.position;
                }
            }
        }
        public static void FitToLifesizeFOV(this Camera camera, Bounds bounds)
        {
            // Assume UP is Y
            camera.transform.position = new Vector3(camera.transform.position.x, bounds.center.y, camera.transform.position.z);
            camera.transform.LookAt(bounds.center);

            // Place camera in center of bounds, but move back a meter
            camera.transform.position = bounds.center - camera.transform.forward;

            float screenHeightInch = Screen.height / Screen.dpi;
            float screenHeightMeters = screenHeightInch / INCHES_PER_METER;

            // FOV adjustment
            var distance = (camera.transform.position - bounds.center).magnitude;
            camera.fieldOfView = (Mathf.Atan((screenHeightMeters / 2.0f) / distance) * 2.0f) * Mathf.Rad2Deg;
        }
        public static void FitToLifesize(this Camera camera, Bounds bounds)
        {
            // Assume UP is Y
            camera.transform.position = new Vector3(camera.transform.position.x, bounds.center.y, camera.transform.position.z);
            camera.transform.LookAt(bounds.center);

            float screenHeightInch = Screen.height / Screen.dpi;
            float screenHeightMeters = screenHeightInch / INCHES_PER_METER;
            
            var frustumHeight = screenHeightMeters;
            var distance = frustumHeight * 0.5f / Mathf.Tan(camera.fieldOfView * 0.5f * Mathf.Deg2Rad);

            camera.transform.position = bounds.center - distance * camera.transform.forward;
            camera.transform.LookAt(bounds.center);
        }
    }
}
