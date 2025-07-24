#include "VitraViewIrisAPI.h"

#include <thread>
#include <atomic>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

static std::thread worker;

bool pollingEyePos = false;
bool eyeTrackingInitialized = false;

int frameWidth, frameHeight;
cv::VideoCapture cap = cv::VideoCapture();

cv::String fd_modelPath = "D:\\CodingProjects\\VitraView\\VitraViewPlugin\\VitraView\\VitraViewIris\\x64\\face_detection_yunet_2023mar.onnx";

cv::Point2f leftEye;
cv::Point2f rightEye;

const float scoreThreshold = 0.9f;
const float nmsThreshold = 0.3f;
const int topK = 5000;

const float scale = 1.0f;

const double cosine_similar_thresh = 0.363;
const double l2norm_similar_thresh = 1.128;

cv::TickMeter tm;
cv::Ptr<cv::FaceDetectorYN> detector;

//Add code to update this from file.
const float userIpd = 0.07f;

cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 589.622580, 0.000000, 310.524711, 0.000000, 589.912712, 217.313359, 0.000000, 0.000000, 1.000000);
cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -0.369050, 0.160037, 0.000265, 0.000991, -0.072916);

cv::Mat camRotExtrinsics = (cv::Mat_<float>(3, 3) <<
    0.999892, 0.014597, 0.00119158,
    -0.0136876, 0.960336, -0.278507,
    -0.00520969, 0.278461, 0.960433);

cv::Mat camPosExtrinsics = (cv::Mat_<float>(3, 1) << 
    0.0114517, -0.164061, -0.0334594);

// -- End read from file logic

cv::KalmanFilter kalman(6, 3);
std::chrono::steady_clock::time_point lastUpdateTime;
std::mutex kalmanMutex;

static std::atomic<float> eyePosition[3]{ 0.f, 0.f, 0.f };

void UpdateEyePixelCoords(cv::Mat* faces) {
    leftEye = cv::Point2f(faces->at<float>(0, 4), faces->at<float>(0, 5));
    rightEye = cv::Point2f(faces->at<float>(0, 6), faces->at<float>(0, 7));
}

void OnAwake() {
    if (cap.isOpened())
    {
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 1080);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1920);

        frameWidth = int(cap.get(cv::CAP_PROP_FRAME_WIDTH) * scale);
        frameHeight = int(cap.get(cv::CAP_PROP_FRAME_HEIGHT) * scale);
    }
    detector = cv::FaceDetectorYN::create(fd_modelPath, "", cv::Size(frameWidth, frameHeight), scoreThreshold, nmsThreshold, topK);

    kalman.transitionMatrix = (cv::Mat_<float>(6, 6) <<
        1, 0, 0, 1, 0, 0,
        0, 1, 0, 0, 1, 0,
        0, 0, 1, 0, 0, 1,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1);

    cv::setIdentity(kalman.measurementMatrix);
    cv::setIdentity(kalman.processNoiseCov, cv::Scalar::all(1e-4));
    cv::setIdentity(kalman.measurementNoiseCov, cv::Scalar::all(1e-3));
    cv::setIdentity(kalman.errorCovPost, cv::Scalar::all(1));
    kalman.statePost = (cv::Mat_<float>(6, 1) << 0, 0, 0, 0, 0, 0);

    lastUpdateTime = std::chrono::steady_clock::now();

    eyeTrackingInitialized = true;
}

void OnUpdate() {
    cv::Mat frame;
    float rollingAvgFrameRate = 0;
    int maxFrames = 300;
    int framecounter = 0;
    float curFramerate = 0;
    while (cap.read(frame)) {
        cv::Mat faces;

        tm.start();
        detector->detect(frame, faces);
        tm.stop();

        if (faces.rows > 0) {
            UpdateEyePixelCoords(&faces);

            std::vector<cv::Point2f> distorted = { leftEye, rightEye };
            std::vector<cv::Point2f> undistorted;

            cv::undistortPoints(distorted, undistorted, cameraMatrix, distCoeffs);

            cv::Vec3f dirL(undistorted[0].x, undistorted[0].y, 1.0);
            cv::Vec3f dirR(undistorted[1].x, undistorted[1].y, 1.0);

            cv::Point3f dL(cv::normalize(cv::Vec3f(undistorted[0].x, undistorted[0].y, 1.0)));
            cv::Point3f dR(cv::normalize(cv::Vec3f(undistorted[1].x, undistorted[1].y, 1.0)));

            dL.y *= -1;
            dR.y *= -1;

            cv::Point3f center_ray = (dL + dR) * 0.5f;
            cv::Mat pMat = (cv::Mat_<float>(3, 1) << center_ray.x, center_ray.y, center_ray.z);
            cv::Mat rotatedPointAsMat = camRotExtrinsics * pMat;

            float norm_eye_distance = cv::norm(dL - dR);

            float scale = userIpd / norm_eye_distance;

            cv::Mat eye_center_3d = (rotatedPointAsMat * scale) - camPosExtrinsics;

            std::lock_guard<std::mutex> lock(kalmanMutex);
            auto now = std::chrono::steady_clock::now();
            float dt = std::chrono::duration<float>(now - lastUpdateTime).count();
            
            curFramerate = 1.0f / dt;
            
            if (dt <= 0.0001f) {
                continue;
            }

            // Update transition matrix with actual delta time
            kalman.transitionMatrix.at<float>(0, 3) = dt;
            kalman.transitionMatrix.at<float>(1, 4) = dt;
            kalman.transitionMatrix.at<float>(2, 5) = dt;

            cv::Mat prediction = kalman.predict();

            cv::Mat measurement;
            eye_center_3d.convertTo(measurement, CV_32F);
            cv::Mat estimate = kalman.correct(measurement);

            eyePosition[0] = estimate.at<float>(0);
            eyePosition[1] = estimate.at<float>(1);
            eyePosition[2] = estimate.at<float>(2);

            lastUpdateTime = now;
        }

        if (framecounter < maxFrames) {
            framecounter++;
        }
        rollingAvgFrameRate -= (rollingAvgFrameRate / framecounter);
        rollingAvgFrameRate += (curFramerate / framecounter);
        std::cout << "\r fps:" << rollingAvgFrameRate;
        cv::waitKey(1);
    }
}

extern "C" {
    void StartEyeTracking() {
        if (!pollingEyePos) {
            cap = cv::VideoCapture(0, cv::CAP_DSHOW);
            if (!cap.isOpened()) return;
            if (!eyeTrackingInitialized) {
                OnAwake();
            }

            pollingEyePos = true;
            worker = std::thread(OnUpdate);
        }
    }

    void StopEyeTracking() {
        pollingEyePos = false;
        cap.release();
        if (worker.joinable()) {
            worker.join();
        }
    }

    void GetEyePosition(float* outArray) {
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

        outArray[0] = -predicted.at<float>(0); // x
        outArray[1] = predicted.at<float>(1); // y
        outArray[2] = -predicted.at<float>(2); // z
    }
}