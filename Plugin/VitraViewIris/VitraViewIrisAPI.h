#pragma once

#ifndef VITRAVIEW_EXPORTS
#define VITRAVIEW_API __declspec(dllexport)
#else
#define VIRTAVIEW_API __declspec(dllimport)
#endif 

extern "C" {

	VITRAVIEW_API void StartEyeTracking();
	VITRAVIEW_API void StopEyeTracking();
	VITRAVIEW_API void GetEyePosition(float* outArray);
}