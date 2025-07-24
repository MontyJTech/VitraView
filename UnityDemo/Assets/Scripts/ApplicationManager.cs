using UnityEngine;

public class ApplicationManager : MonoBehaviour
{
    public string fileName = "screenshot.png";

    [SerializeField]
    GameObject arucoMarkersScreen;

    [SerializeField]
    GameObject chessBoardScreen;

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Escape))
        {
            Application.Quit();
        }
        if (Input.GetKeyDown(KeyCode.P))
        {
            TakeScreenshot();
        }
        if (Input.GetKeyDown(KeyCode.C))
        {
            EnterCameraCalibrationMode();
        }
        if (Input.GetKeyDown(KeyCode.D))
        {
            EnterDemoMode();
        }
    }

    private void TakeScreenshot()
    {
        ScreenCapture.CaptureScreenshot(fileName);
        Debug.Log("Screenshot saved to: " + fileName);
    }

    private void EnterCameraCalibrationMode()
    {
        arucoMarkersScreen.SetActive(false);
        chessBoardScreen.SetActive(true);
    }

    private void EnterDemoMode()
    {
        arucoMarkersScreen.SetActive(true);
        chessBoardScreen.SetActive(false);
    }
}
