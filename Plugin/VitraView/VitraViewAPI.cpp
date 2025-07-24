#include "VitraViewAPI.h"

#include <thread>
#include <atomic>
#include <chrono>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

std::vector<int> screenTrackingMarkerIds = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };

std::vector<int> mirrorMarkerIds = { 0, 1, 2, 3 };
std::vector<int> screenMarkerIds = { 4, 5, 6, 7, 8, 9 };

float hScreenWidth = 0.475f / 2.0f; //half screen width
float hScreenHeight = 0.27f / 2.0f; //half screen height

float hMirrorWidth = 0.34f / 2.0f; //half mirror width
float hMirrorHeight = 0.24f / 2.0f; //half mirror height

float hMarkerSize = 0.03f;
float hArucoSize = 0.02f;

std::vector<std::vector<cv::Point3f>> screenArucoBoard;
std::vector<std::vector<cv::Point3f>> mirrorArucoBoard;

cv::KalmanFilter kalman(6, 3);
std::chrono::steady_clock::time_point lastUpdateTime;
std::mutex kalmanMutex;

cv::VideoCapture cap;

static std::atomic<float> value(0.0f);
static std::atomic<float> camPosition[3]{0.f, 0.f, 0.f};

static std::thread worker;

bool pollingPose = false;
bool screenTrackingInitialized = false;

static bool isRecordingVideo = false;
cv::VideoWriter writer = cv::VideoWriter();

static cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 589.622580, 0.000000, 310.524711, 0.000000, 589.912712, 217.313359, 0.000000, 0.000000, 1.000000);
static cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -0.369050, 0.160037, 0.000265, 0.000991, -0.072916);

cv::Ptr<cv::aruco::Dictionary> markerDictionary;

cv::Ptr<cv::aruco::Board> screenBoard;
cv::Ptr<cv::aruco::Board> mirrorBoard;

cv::aruco::DetectorParameters params;
cv::Ptr<cv::aruco::DetectorParameters> paramsPtr = cv::makePtr<cv::aruco::DetectorParameters>(params);
cv::Mat frame;

void PrintMatrix(cv::Mat* mat);
cv::Mat BuildTransformationMatrix(cv::Mat rotation, cv::Vec3d translation);

float GetXByColumn(int colNum) {
    const int totalCols = 4; // 4 markers along top/bottom
    if (colNum < 0 || colNum >= totalCols)
        return 0.f;

    float markerSize = hMarkerSize * 2.0f;
    float monitorWidth = hScreenWidth * 2.0f;
    float spacing = ((monitorWidth - (totalCols * markerSize)) / (totalCols - 1)) + markerSize;

    return -hScreenWidth + hMarkerSize + (spacing * colNum);
}

float GetYByRow(int rowNum) {
    const int totalRows = 3; // top, middle, bottom
    if (rowNum < 0 || rowNum >= totalRows)
        return 0.f;

    float markerSize = hMarkerSize * 2.0f;
    float monitorHeight = hScreenHeight * 2.0f;
    float spacing = ((monitorHeight - (totalRows * markerSize)) / (totalRows - 1)) + markerSize;

    return -hScreenHeight + hMarkerSize + (spacing * rowNum);
}

std::vector<cv::Point3f> GetMarkerCornerArrays(float xCentre, float yCentre) {
    return {
        {xCentre + hArucoSize, yCentre + hArucoSize, 0}, //br
        {xCentre - hArucoSize, yCentre + hArucoSize, 0},  //bl
        {xCentre - hArucoSize, yCentre - hArucoSize, 0}, //tl
        {xCentre + hArucoSize, yCentre - hArucoSize, 0} //tr
    };
}

void InitializeScreenTrackingArucoBoard() {
    // 0: 0 - 0
    float xByCol = GetXByColumn(0);
    float yByRow = GetYByRow(0);
    screenArucoBoard.push_back(GetMarkerCornerArrays(xByCol, yByRow));

    // 1: 1 - 0
    xByCol = GetXByColumn(1);
    yByRow = GetYByRow(0);
    screenArucoBoard.push_back(GetMarkerCornerArrays(xByCol, yByRow));

    // 2: 2 - 0
    xByCol = GetXByColumn(2);
    yByRow = GetYByRow(0);
    screenArucoBoard.push_back(GetMarkerCornerArrays(xByCol, yByRow));

    // 3: 3 - 0
    xByCol = GetXByColumn(3);
    yByRow = GetYByRow(0);
    screenArucoBoard.push_back(GetMarkerCornerArrays(xByCol, yByRow));

    // 4: 3 - 1
    xByCol = GetXByColumn(3);
    yByRow = GetYByRow(1);
    screenArucoBoard.push_back(GetMarkerCornerArrays(xByCol, yByRow));

    // 5: 3 - 2
    xByCol = GetXByColumn(3);
    yByRow = GetYByRow(2);
    screenArucoBoard.push_back(GetMarkerCornerArrays(xByCol, yByRow));

    // 6: 2 - 2
    xByCol = GetXByColumn(2);
    yByRow = GetYByRow(2);
    screenArucoBoard.push_back(GetMarkerCornerArrays(xByCol, yByRow));

    // 7: 1 - 2
    xByCol = GetXByColumn(1);
    yByRow = GetYByRow(2);
    screenArucoBoard.push_back(GetMarkerCornerArrays(xByCol, yByRow));

    // 8: 0 - 2
    xByCol = GetXByColumn(0);
    yByRow = GetYByRow(2);
    screenArucoBoard.push_back(GetMarkerCornerArrays(xByCol, yByRow));

    // 9: 0 - 1
    xByCol = GetXByColumn(0);
    yByRow = GetYByRow(1);
    screenArucoBoard.push_back(GetMarkerCornerArrays(xByCol, yByRow));

    screenBoard = cv::aruco::Board::create(screenArucoBoard, markerDictionary, screenTrackingMarkerIds);
}

void ScreenTrackingOnAwake() {
    kalman.transitionMatrix = (cv::Mat_<float>(6, 6) <<
        1, 0, 0, 1, 0, 0,
        0, 1, 0, 0, 1, 0,
        0, 0, 1, 0, 0, 1,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1);

    cv::setIdentity(kalman.measurementMatrix);
    cv::setIdentity(kalman.processNoiseCov, cv::Scalar::all(1e-4));
    cv::setIdentity(kalman.measurementNoiseCov, cv::Scalar::all(1e-2));
    cv::setIdentity(kalman.errorCovPost, cv::Scalar::all(1));
    kalman.statePost = (cv::Mat_<float>(6, 1) << 0, 0, 0, 0, 0, 0);

    lastUpdateTime = std::chrono::steady_clock::now();

    markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);

    InitializeScreenTrackingArucoBoard();

    screenTrackingInitialized = true;
}

void ScreenTrackingUpdate() {
    int frameIndex = 0;

    while (cap.read(frame)) {
        std::vector<int> detectedIds;
        std::vector<std::vector<cv::Point2f>> detectedCorners, rejectedCandidates;

        cv::aruco::detectMarkers(frame, markerDictionary, detectedCorners, detectedIds, paramsPtr, rejectedCandidates);
        cv::aruco::refineDetectedMarkers(frame, screenBoard, detectedCorners, detectedIds, rejectedCandidates);

        cv::Vec3d rvec, tvec;
        int valid = cv::aruco::estimatePoseBoard(
            detectedCorners, detectedIds, screenBoard, cameraMatrix, distCoeffs, rvec, tvec
        );
        if (valid > 0) {
            cv::Mat R;
            cv::Rodrigues(rvec, R);
            
            cv::Mat cameraPosition = -R.t() * cv::Mat(tvec);
            cameraPosition.at<double>(1, 0) *= -1;

            //Kalman on update

            std::lock_guard<std::mutex> lock(kalmanMutex);

            auto now = std::chrono::steady_clock::now();
            float dt = std::chrono::duration<float>(now - lastUpdateTime).count();

            // Update transition matrix with actual delta time
            kalman.transitionMatrix.at<float>(0, 3) = dt;
            kalman.transitionMatrix.at<float>(1, 4) = dt;
            kalman.transitionMatrix.at<float>(2, 5) = dt;

            cv::Mat prediction = kalman.predict();

            cv::Mat measurement;
            cameraPosition.convertTo(measurement, CV_32F);
            cv::Mat estimate = kalman.correct(measurement);

            camPosition[0] = estimate.at<float>(0);
            camPosition[1] = estimate.at<float>(1);
            camPosition[2] = estimate.at<float>(2);

            lastUpdateTime = now;

            //kalman end on update
        }

        if (isRecordingVideo) {
            if (!writer.isOpened()) {
                double fps = cap.get(cv::CAP_PROP_FPS);
                if (fps == 0) fps = 30;

                int frame_width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
                int frame_height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));

                writer = cv::VideoWriter("output.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, cv::Size(frame_width, frame_height));
            }
            else {
                if (frame.empty()) {
                    continue;
                }
                writer.write(frame);
            }
        }
        else {
            if (writer.isOpened()) {
                writer.release();
            }
        }
    }
    cap.release();
}

void InitializeExtrinsicsArucoBoards() {
    //markers 0-3 are initialized as the mirror board, the rest are intialized as the screenboard
    float xByCol = 0;
    float yByRow = 0;

    // 0: 0 - 0
    mirrorArucoBoard.push_back(GetMarkerCornerArrays(-hMirrorWidth, -hMirrorHeight));

    // 1
    mirrorArucoBoard.push_back(GetMarkerCornerArrays(hMirrorWidth, -hMirrorHeight));

    // 2
    mirrorArucoBoard.push_back(GetMarkerCornerArrays(hMirrorWidth, hMirrorHeight));

    // 3: 3 - 0
    mirrorArucoBoard.push_back(GetMarkerCornerArrays(-hMirrorWidth, hMirrorHeight));



    // 4: 3 - 1
    xByCol = GetXByColumn(1);
    yByRow = GetYByRow(0);
    screenArucoBoard.push_back(GetMarkerCornerArrays(xByCol, yByRow));

    // 5: 3 - 2
    xByCol = GetXByColumn(2);
    yByRow = GetYByRow(0);
    screenArucoBoard.push_back(GetMarkerCornerArrays(xByCol, yByRow));

    // 6: 2 - 2
    xByCol = GetXByColumn(3);
    yByRow = GetYByRow(1);
    screenArucoBoard.push_back(GetMarkerCornerArrays(xByCol, yByRow));

    // 7: 1 - 2
    xByCol = GetXByColumn(2);
    yByRow = GetYByRow(2);
    screenArucoBoard.push_back(GetMarkerCornerArrays(xByCol, yByRow));

    // 8: 0 - 2
    screenArucoBoard.push_back(GetMarkerCornerArrays(GetXByColumn(1), GetYByRow(2)));

    // 9: 0 - 1
    xByCol = GetXByColumn(0);
    yByRow = GetYByRow(1);
    screenArucoBoard.push_back(GetMarkerCornerArrays(xByCol, yByRow));

    screenBoard = cv::aruco::Board::create(screenArucoBoard, markerDictionary, screenMarkerIds);
    mirrorBoard = cv::aruco::Board::create(mirrorArucoBoard, markerDictionary, mirrorMarkerIds);
}

cv::Mat ComputeReflectionMatrix(const cv::Vec3d& mirrorNorm) {
    cv::Mat M = cv::Mat::eye(3, 3, CV_64F);
    cv::Vec3d n = cv::normalize(mirrorNorm);

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            M.at<double>(i, j) -= 2 * n[i] * n[j];  // M = I - 2 * n * n^T

    return M;
}

cv::Mat ReflectRotationMatrix(const cv::Mat& R_obs, const cv::Vec3d& mirrorNormal) {
    cv::Mat M = ComputeReflectionMatrix(mirrorNormal);
    return M * R_obs * M;
}

void WaitForExtrinsicsConvergence() {
    markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    InitializeExtrinsicsArucoBoards();

    bool extrinsicsCalculated = false;
    int maxCountTimeout = 1000000;
    int counter = 0;

    cv::Mat frame;
    std::cout << "Calculating...\n";

    cv::Vec3d screenOriginRollingAverage = cv::Vec3d();
    cv::Vec3d reflectedNormalRollingAverage = cv::Vec3d();
    cv::Mat rotationMatrixRollingAverage = cv::Mat::eye(3, 3, CV_64F);

    int iteration = 0;

    while (cap.read(frame)) {
        std::vector<int> detectedIds;
        std::vector<std::vector<cv::Point2f>> detectedCorners, rejectedCandidates;

        cv::aruco::detectMarkers(frame, markerDictionary, detectedCorners, detectedIds, paramsPtr, rejectedCandidates);
        cv::aruco::refineDetectedMarkers(frame, screenBoard, detectedCorners, detectedIds, rejectedCandidates);
        cv::aruco::refineDetectedMarkers(frame, mirrorBoard, detectedCorners, detectedIds, rejectedCandidates); 

        cv::Vec3d srvec, stvec, mrvec, mtvec;
        int screenValid = cv::aruco::estimatePoseBoard(
            detectedCorners, detectedIds, screenBoard, cameraMatrix, distCoeffs, srvec, stvec
        );

        int mirrorValid = cv::aruco::estimatePoseBoard(
            detectedCorners, detectedIds, mirrorBoard, cameraMatrix, distCoeffs, mrvec, mtvec
        );

        //DEBUG ONLY
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(detectedCorners, 0.04f, cameraMatrix, distCoeffs, rvecs, tvecs);

        for (size_t i = 0; i < rvecs.size(); i++) {
            cv::drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.04f);
        }

        if (screenValid > 0) {
            cv::drawFrameAxes(frame, cameraMatrix, distCoeffs, srvec, stvec, 0.05f);
            cv::drawFrameAxes(frame, cameraMatrix, distCoeffs, mrvec, mtvec, 0.05f);
        }

        //DEBUG ONLY

        if (screenValid > 0) {
            if (iteration < 1000) {
                iteration++;
            }
            cv::Mat mRotation, sRotation;

            cv::Rodrigues(mrvec, mRotation);
            cv::Rodrigues(srvec, sRotation); //screen rotation relative to camera. (not reflected)

            sRotation.at<double>(0, 2) *= -1;
            sRotation.at<double>(1, 2) *= -1;
            sRotation.at<double>(2, 2) *= -1;

            mRotation.at<double>(0, 2) *= -1;
            mRotation.at<double>(1, 2) *= -1;
            mRotation.at<double>(2, 2) *= -1;

            cv::Vec3d mZAxis(
                mRotation.at<double>(0, 2),
                mRotation.at<double>(1, 2),
                mRotation.at<double>(2, 2)
            );

            cv::Vec3d sZAxis(
                sRotation.at<double>(0, 2),
                sRotation.at<double>(1, 2),
                sRotation.at<double>(2, 2)
            );

            //Screen position relative to camera.
            cv::Vec3d mirrorToScreen = stvec - mtvec;
            double reprojectedDir = mirrorToScreen.dot(mZAxis);
            cv::Vec3d screenOrigin = stvec - (2.0f * reprojectedDir * mZAxis);

            //Get tip of z axis 
            cv::Vec3d screenZPoint = stvec + (cv::Vec3d(sRotation.at<double>(0, 2), sRotation.at<double>(1, 2), sRotation.at<double>(2, 2)));

            cv::Vec3d mirrorToScreenZPoint = screenZPoint - mtvec;
            double zReprojected = mirrorToScreenZPoint.dot(mZAxis);
            cv::Vec3d camSpaceSZAxis = screenZPoint - (2.0f * zReprojected * mZAxis);

            cv::Vec3d forward = camSpaceSZAxis - screenOrigin;


            //Get tip of y axis 
            cv::Vec3d screenYPoint = stvec + (cv::Vec3d(sRotation.at<double>(0, 1), sRotation.at<double>(1, 1), sRotation.at<double>(2, 1)));

            cv::Vec3d mirrorToScreenYPoint = screenYPoint - mtvec;
            double yReprojected = mirrorToScreenYPoint.dot(mZAxis);
            cv::Vec3d camSpaceSYAxis = screenYPoint - (2.0f * yReprojected * mZAxis);

            cv::Vec3d up = camSpaceSYAxis - screenOrigin;

            cv::normalize(forward, forward);
            cv::normalize(up, up);

            cv::Vec3d right = cv::normalize(up.cross(forward));
            cv::Vec3d newUp = forward.cross(right);

            cv::Mat R = (cv::Mat_<double>(3, 3) <<
                right[0], newUp[0], forward[0],
                right[1], newUp[1], forward[1],
                right[2], newUp[2], forward[2]);

            PrintMatrix(&R);

            //std::cout << "\rInit result: " << camSpaceSZAxis - screenOrigin;
            screenOriginRollingAverage -= (screenOriginRollingAverage / iteration);
            screenOriginRollingAverage += (screenOrigin / iteration);

            //if (iteration == 1000) {
            //    cv::Mat transformation = BuildTransformationMatrix(rotationMatrixRollingAverage, screenOriginRollingAverage);
            //    PrintMatrix(&transformation);
            //    std::cout << "\niter limit reached.";
            //}
        }

        cv::imshow("TestWindow", frame);
        cv::waitKey(1);

        counter++;
        if (extrinsicsCalculated || counter >= maxCountTimeout) {
            break;
        }
    }
}

extern "C" {
    void CalculateExtrinsics() {
        cameraMatrix = (cv::Mat_<double>(3, 3) << 589.622580, 0.000000, 310.524711, 0.000000, 589.912712, 217.313359, 0.000000, 0.000000, 1.000000);
        distCoeffs = (cv::Mat_<double>(1, 5) << -0.369050, 0.160037, 0.000265, 0.000991, -0.072916);

        if (pollingPose) {
            StopArucoBoardTracking();
        }

        cap = cv::VideoCapture(0, cv::CAP_DSHOW);
        if (!cap.isOpened()) return;

        WaitForExtrinsicsConvergence();

        cap.release();
    }


	void StartArucoBoardTracking() {
		if (!pollingPose) {
            cap = cv::VideoCapture(0, cv::CAP_DSHOW);
            if (!cap.isOpened()) return;
            if (!screenTrackingInitialized) {
                ScreenTrackingOnAwake();
            }

            pollingPose = true;
			worker = std::thread(ScreenTrackingUpdate);
		}
	}

	void StopArucoBoardTracking() {
        pollingPose = false;
        cap.release();
		if(worker.joinable()) {
			worker.join();
		}
	}

    void StartRecordingCam() {
        isRecordingVideo = true;
    }

    void StopRecordingCam() {
        isRecordingVideo = false;
    }

	float GetLatestValue() {
		return value.load();
	}

    void GetCamPosition(float* outArray) {
        std::lock_guard<std::mutex> lock(kalmanMutex);

        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - lastUpdateTime).count();

        // Build temp transition matrix
        cv::Mat A = (cv::Mat_<float>(6, 6) <<
            1, 0, 0, dt, 0, 0,
            0, 1, 0, 0, dt, 0,
            0, 0, 1, 0, 0, dt,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1);

        cv::Mat predicted = A * kalman.statePost;

        outArray[0] = predicted.at<float>(0); // x
        outArray[1] = predicted.at<float>(1); // y
        outArray[2] = predicted.at<float>(2); // z
    }
}

cv::Mat BuildTransformationMatrix(cv::Mat rotation, cv::Vec3d translation) {
    cv::Mat transform = cv::Mat::eye(4, 4, CV_64F); // Initialize to identity

    // Copy rotation matrix
    rotation.copyTo(transform(cv::Rect(0, 0, 3, 3)));

    // Copy translation vector
    transform.at<double>(0, 3) = translation[0];
    transform.at<double>(1, 3) = translation[1];
    transform.at<double>(2, 3) = translation[2];

    return transform;
}

void PrintMatrix(cv::Mat* mat) {
    std::cout << "\n";
    for (int row = 0; row < mat->rows; row++) {
        for (int col = 0; col < mat->cols; col++) {
            std::cout << mat->at<double>(row, col) << " ";
        }
        std::cout << "\n";
    }
}