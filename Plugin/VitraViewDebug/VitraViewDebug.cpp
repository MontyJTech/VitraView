#include <iostream>
#include <windows.h>

typedef void (*CalculateExtrinsicsFunc)();
typedef void (*StartPollingPoseFunc)();
typedef void (*StopPollingPoseFunc)();
typedef void (*GetCamPositionFunc)(float*);

typedef void (*StartPollingEyeFunc)();
typedef void (*StopPollingEyeFunc)();
typedef void (*GetEyePositionFunc)(float*);

CalculateExtrinsicsFunc CalculateExtrinsics;

StartPollingPoseFunc StartPollingPose;
StopPollingPoseFunc StopPollingPose;
GetCamPositionFunc GetCamPosition;

StartPollingEyeFunc StartPollingEye;
StopPollingEyeFunc StopPollingEye;
GetEyePositionFunc GetEyePosition;

bool InitArucoDynamicFunctions(HMODULE* arucoDll);
bool InitIrisDynamicFunctions(HMODULE* irisDll);

void PollScreenTrackingTest();
void PollEyeTrackingTest();
void CamExtrinsicsTest();

int main() {
    HMODULE arucoDll = LoadLibraryA("VitraView.dll");
    HMODULE irisDll = LoadLibraryA("VitraViewIris.dll");

    if (!arucoDll) {
        std::cerr << "Failed to load VitraView.dll\n";
        return 1;
    }

    if (!irisDll) {
        std::cerr << "Failed to load VitraViewIris.dll\n";
        return 1;
    }

    if (!InitArucoDynamicFunctions(&arucoDll)) {
        std::cerr << "One or more functions not found in aruco DLL.\n";
        FreeLibrary(arucoDll);
        return 1;
    }

    if (!InitIrisDynamicFunctions(&irisDll)) {
        std::cerr << "One or more functions not found in iris DLL.\n";
        FreeLibrary(irisDll);
        return 1;
    }

    //PollScreenTrackingTest();
    //CamExtrinsicsTest();
    PollEyeTrackingTest();

    FreeLibrary(arucoDll);
    FreeLibrary(irisDll);
    return 0;
}


bool InitArucoDynamicFunctions(HMODULE* arucoDll) {
    CalculateExtrinsics = (CalculateExtrinsicsFunc)GetProcAddress(*arucoDll, "CalculateExtrinsics");

    StartPollingPose = (StartPollingPoseFunc)GetProcAddress(*arucoDll, "StartArucoBoardTracking");
    StopPollingPose = (StopPollingPoseFunc)GetProcAddress(*arucoDll, "StopArucoBoardTracking");
    GetCamPosition = (GetCamPositionFunc)GetProcAddress(*arucoDll, "GetCamPosition");

    return StartPollingPose && StopPollingPose && GetCamPosition && CalculateExtrinsics;
}

bool InitIrisDynamicFunctions(HMODULE* irisDll) {
    StartPollingEye = (StartPollingEyeFunc)GetProcAddress(*irisDll, "StartEyeTracking");
    StopPollingEye = (StopPollingEyeFunc)GetProcAddress(*irisDll, "StopEyeTracking");
    GetEyePosition = (GetEyePositionFunc)GetProcAddress(*irisDll, "GetEyePosition");

    return StartPollingEye && StopPollingEye && GetEyePosition;
}

void PollScreenTrackingTest() {
    std::cout << "Starting polling...\n";
    StartPollingPose();

    float pos[3] = { 0 };

    while (true) {
        GetCamPosition(pos);
        std::cout << "\rPredicted: x=" << pos[0] << ", y=" << pos[1] << ", z=" << pos[2] << "                                                    ";
    }

    std::cout << "Stopping polling...\n";
    StopPollingPose();
}

void PollEyeTrackingTest() {
    std::cout << "Starting polling...\n";
    StartPollingEye();

    float pos[3] = { 0 };

    while (true) {
        GetEyePosition(pos);
        //std::cout << "\rPredicted: x=" << pos[0] << ", y=" << pos[1] << ", z=" << pos[2] << "                                                    ";
    }

    std::cout << "Stopping polling...\n";
    StopPollingEye();
}

void CamExtrinsicsTest() {
    std::cout << "Starting extrinsics...\n";

    CalculateExtrinsics();

    std::cout << "Stopping extrinsics...\n";
}