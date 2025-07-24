#pragma once

#ifndef VITRAVIEW_EXPORTS
#define VITRAVIEW_API __declspec(dllexport)
#else
#define VIRTAVIEW_API __declspec(dllimport)
#endif 

extern "C" {
	VITRAVIEW_API void CalculateExtrinsics();
	VITRAVIEW_API void StartArucoBoardTracking();
	VITRAVIEW_API void StopArucoBoardTracking();
	VITRAVIEW_API void StartRecordingCam();
	VITRAVIEW_API void StopRecordingCam();
	VITRAVIEW_API float GetLatestValue();
	VITRAVIEW_API void GetCamPosition(float* outArray);
}