using UnityEngine;

public class CameraController : MonoBehaviour
{
    [Range(0, 1)]
    public float smoothFactor = 0.1f;

    private Vector3 smoothedPosition = new Vector3();
    private bool isInitialized = false;

    const float predictedScaleFactor = 1.0f;

    const float halfVerticalDistanceInMeters = (0.27f / 2.0f) * predictedScaleFactor;
    const float halfHorizontalDistanceInMeters = (0.475f / 2.0f) * predictedScaleFactor;

    Vector3 rootToPlaneCentre;

    [SerializeField]
    Camera cam;

    [SerializeField]
    VitraPoseReader poseReader;

    [SerializeField]
    Transform physicalCam;

    private void Awake()
    {
        CalculateRootValues();
    }

    private void Update()
    {
        poseReader.UpdatePhysCamPos(predictedScaleFactor);
        CalculateNewCamPlanes();
    }

    private void CalculateRootValues()
    {
        rootToPlaneCentre = transform.forward * cam.nearClipPlane;
    }

    private void CalculateNewCamPlanes()
    {
        if (!isInitialized)
        {
            smoothedPosition = physicalCam.localPosition;
            isInitialized = true;
        }
        else
        {
            smoothedPosition = (smoothFactor * physicalCam.localPosition) + ((1.0f - smoothFactor) * smoothedPosition);
        }

        cam.transform.localPosition = smoothedPosition;

        //cam.transform.localPosition = planeCentreToVirtualCam;
        //cam.transform.localPosition = physicalCam.localPosition;

        Vector3 camToRoot = -cam.transform.localPosition;

        cam.nearClipPlane = rootToPlaneCentre.z + camToRoot.z;

        float left = -halfHorizontalDistanceInMeters + camToRoot.x;
        float right = halfHorizontalDistanceInMeters + camToRoot.x;
        float bottom = -halfVerticalDistanceInMeters + camToRoot.y;
        float top = halfVerticalDistanceInMeters + camToRoot.y;

        cam.fieldOfView = 2f * Mathf.Atan(halfVerticalDistanceInMeters / cam.nearClipPlane) * Mathf.Rad2Deg;
        cam.projectionMatrix = PerspectiveOffCenter(left, right, bottom, top, cam.nearClipPlane, cam.farClipPlane);
    }

    Matrix4x4 PerspectiveOffCenter(float left, float right, float bottom, float top, float near, float far)
    {
        Matrix4x4 m = new Matrix4x4();
        m[0, 0] = 2.0f * near / (right - left);
        m[0, 2] = (right + left) / (right - left);
        m[1, 1] = 2.0f * near / (top - bottom);
        m[1, 2] = (top + bottom) / (top - bottom);
        m[2, 2] = -(far + near) / (far - near);
        m[2, 3] = -(2.0f * far * near) / (far - near);
        m[3, 2] = -1.0f;
        return m;
    }

    private Vector3 GetScaledCamPosition(Vector3 nearPlaneToPhysicalCam, Vector3 nearPlaneOrigin)
    {
        float curDistance = Vector3.Magnitude(nearPlaneToPhysicalCam);
        if (curDistance > 1.0f)
        {
            return nearPlaneToPhysicalCam;
        }

        float t = Mathf.InverseLerp(0f, 0.5f, curDistance);  // 0 → 1
        float targetFOV = Mathf.Lerp(180f, 80f, t);              // Smooth transition

        // Clamp just in case
        targetFOV = Mathf.Clamp(targetFOV, 1f, 179f); // Avoid tan(90°) blowup

        float desiredDistance = halfVerticalDistanceInMeters / Mathf.Tan(targetFOV * 0.5f * Mathf.Deg2Rad);

        return nearPlaneOrigin + Vector3.Normalize(nearPlaneToPhysicalCam) * desiredDistance;
    }
}
