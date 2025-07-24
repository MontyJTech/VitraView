using System.Runtime.InteropServices;
using UnityEngine;

public class VitraPoseReader : MonoBehaviour
{
    [DllImport("VitraView")]
    private static extern void StartPollingPose();

    [DllImport("VitraView")]
    private static extern void StopPollingPose();

    [DllImport("VitraView")]
    private static extern void StartRecordingCam();

    [DllImport("VitraView")]
    private static extern void StopRecordingCam();

    [DllImport("VitraView")]
    private static extern float GetLatestValue();

    [DllImport("VitraView")]
    private static extern float GetCamPosition([Out]float[] outArray);


    [DllImport("VitraViewIris")]
    private static extern void StartEyeTracking();

    [DllImport("VitraViewIris")]
    private static extern void StopEyeTracking();

    [DllImport("VitraViewIris")]
    private static extern float GetEyePosition([Out] float[] outArray);

    private float[] camPos = new float[3];
    private Vector3 curPos = new Vector3();

    private bool isRecording = false;

    void Start()
    {
        StartEyeTracking();
        //StartPollingPose();
    }

    public void Update()
    {
        //if (Input.GetKeyDown(KeyCode.V))
        //{
        //    OnRecordKeyPressed();
        //}
    }

    public void OnRecordKeyPressed()
    {
        if (isRecording)
        {
            StopRecordingCam();
            isRecording = false;
            Debug.Log("Stopped recording");
        }
        else
        {
            StartRecordingCam();
            isRecording = true;
            Debug.Log("Started recording");
        }
    }

    public void UpdatePhysCamPos(float scalar = 1.0f)
    {
        GetEyePosition(camPos);
        curPos = new Vector3(camPos[0] * scalar, camPos[1] * scalar, camPos[2] * scalar);
        gameObject.transform.localPosition = curPos;
    }

    void OnApplicationQuit()
    {
        StopEyeTracking();
    }
}
