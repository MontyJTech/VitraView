//#include <opencv2/dnn.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/objdetect.hpp>
//#include <opencv2/calib3d.hpp>
//
//#include <iostream>
//
//using namespace cv;
//using namespace std;
//
//const float ipd = 0.07f; //in meters
//const float ipd_mm = 70.0f; //in meters
//
//cv::Point2f leftEye;
//cv::Point2f rightEye;
//
//cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 589.622580, 0.000000, 310.524711, 0.000000, 589.912712, 217.313359, 0.000000, 0.000000, 1.000000);
//cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -0.369050, 0.160037, 0.000265, 0.000991, -0.072916);
//
//void UpdateEyePixelCoords(Mat* faces);
//
//cv::Mat camRotExtrinsics = (cv::Mat_<float>(3, 3) <<
//    0.999892, 0.014597, 0.00119158,
//    -0.0136876, 0.960336, -0.278507,
//    -0.00520969, 0.278461, 0.960433);
//
//cv::Vec3d camPosExtrinsics = cv::Vec3d(0.0114517, -0.164061, -0.0334594);
//
//static void visualize(Mat& input, int frame, Mat& faces, double fps, int thickness = 2)
//{
//    std::string fpsString = cv::format("FPS : %.2f", (float)fps);
//    if (frame >= 0)
//        cout << "Frame " << frame << ", ";
//    cout << "FPS: " << fpsString << endl;
//    for (int i = 0; i < faces.rows; i++)
//    {
//        //// Print results
//        //cout << "Face " << i
//        //    << ", top-left coordinates: (" << faces.at<float>(i, 0) << ", " << faces.at<float>(i, 1) << "), "
//        //    << "box width: " << faces.at<float>(i, 2) << ", box height: " << faces.at<float>(i, 3) << ", "
//        //    << "score: " << cv::format("%.2f", faces.at<float>(i, 14))
//        //    << endl;
//
//        // Draw bounding box
//        rectangle(input, Rect2i(int(faces.at<float>(i, 0)), int(faces.at<float>(i, 1)), int(faces.at<float>(i, 2)), int(faces.at<float>(i, 3))), Scalar(0, 255, 0), thickness);
//        // Draw landmarks
//        circle(input, Point2i(int(faces.at<float>(i, 4)), int(faces.at<float>(i, 5))), 2, Scalar(255, 0, 0), thickness);
//        circle(input, Point2i(int(faces.at<float>(i, 6)), int(faces.at<float>(i, 7))), 2, Scalar(0, 0, 255), thickness);
//        //circle(input, Point2i(int(faces.at<float>(i, 8)), int(faces.at<float>(i, 9))), 2, Scalar(0, 255, 0), thickness);
//        //circle(input, Point2i(int(faces.at<float>(i, 10)), int(faces.at<float>(i, 11))), 2, Scalar(255, 0, 255), thickness);
//        //circle(input, Point2i(int(faces.at<float>(i, 12)), int(faces.at<float>(i, 13))), 2, Scalar(0, 255, 255), thickness);
//    }
//    putText(input, fpsString, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);
//}
//
//void UpdateEyePixelCoords(Mat* faces) {
//    leftEye = Point2f(faces->at<float>(0, 4), faces->at<float>(0, 5));
//    rightEye = Point2f(faces->at<float>(0, 6), faces->at<float>(0, 7));
//}
//
////Later improvements: adjust the ipd calculation from a relative calculated avg. width of the head to account for angled head
////Experiment with/ learn about the parameters to the dnn module to optimise. scoreThreshold, nmsThreshold, topK, scale, cosine_similar_thresh, l2norm_similar_thresh
////Add kalman filter to returned values.
//
//int main()
//{
//    //Load in extrinsics offset from file (Camera -> Screen centre)
//    //After detecting camera position, return Eye centre -> screen centre position.
//
//    String fd_modelPath = "D:\\CodingProjects\\VitraView\\VitraViewPlugin\\VitraView\\VitraViewIris\\x64\\face_detection_yunet_2023mar.onnx";
//    String fr_modelPath = "D:\\CodingProjects\\VitraView\\VitraViewPlugin\\VitraView\\VitraViewIris\\x64\\face_recognition_sface_2021dec.onnx";
//
//    float scoreThreshold = 0.9f;
//    float nmsThreshold = 0.3f;
//    int topK = 5000;
//
//    float scale = 1.0f;
//
//    double cosine_similar_thresh = 0.363;
//    double l2norm_similar_thresh = 1.128;
//
//    TickMeter tm;
//
//    int frameWidth, frameHeight;
//    VideoCapture capture(0);
//
//    if (!capture.isOpened())
//    {
//        cout << "Could not open webcam." << "\n";
//        return -1;
//    }
//
//    frameWidth = int(capture.get(CAP_PROP_FRAME_WIDTH) * scale);
//    frameHeight = int(capture.get(CAP_PROP_FRAME_HEIGHT) * scale);
//
//    // Initialize FaceDetectorYN
//    Ptr<FaceDetectorYN> detector = FaceDetectorYN::create(fd_modelPath, "", Size(frameWidth, frameHeight), scoreThreshold, nmsThreshold, topK);
//
//    Mat frame;
//
//    while(capture.read(frame))
//    {
//        // Inference
//        Mat faces;
//        tm.start();
//        detector->detect(frame, faces);
//        tm.stop();
//
//        Mat result = frame.clone();
//
//        if (faces.rows > 0) {
//            UpdateEyePixelCoords(&faces);
//
//            std::vector<cv::Point2f> distorted = { leftEye, rightEye };
//            std::vector<cv::Point2f> undistorted;
//
//            cv::undistortPoints(distorted, undistorted, cameraMatrix, distCoeffs);
//
//            cv::Vec3f dirL(undistorted[0].x, undistorted[0].y, 1.0);
//            cv::Vec3f dirR(undistorted[1].x, undistorted[1].y, 1.0);
//
//            cv::Point3f dL(cv::normalize(Vec3f(undistorted[0].x, undistorted[0].y, 1.0)));
//            cv::Point3f dR(cv::normalize(Vec3f(undistorted[1].x, undistorted[1].y, 1.0)));
//
//            dL.y *= -1;
//            dR.y *= -1;
//
//            cv::Point3f center_ray = (dL + dR) * 0.5f;
//            cv::Mat pMat = (cv::Mat_<float>(3, 1) << center_ray.x, center_ray.y, center_ray.z);
//            cv::Mat rotatedPointAsMat = camRotExtrinsics * pMat;
//            cv::Vec3d rotatedPointAsVec = cv::Vec3d(rotatedPointAsMat);
//
//            float norm_eye_distance = cv::norm(dL - dR);
//
//            float scale = ipd / norm_eye_distance;
//
//            cv::Vec3d eye_center_3d = (rotatedPointAsVec * scale) - camPosExtrinsics;
//            std::cout << "\r point: " << eye_center_3d;
//
//            rectangle(result, Rect2i(int(faces.at<float>(0, 0)), int(faces.at<float>(0, 1)), int(faces.at<float>(0, 2)), int(faces.at<float>(0, 3))), Scalar(0, 255, 0), 2);
//            circle(result, Point2i(int(faces.at<float>(0, 4)), int(faces.at<float>(0, 5))), 2, Scalar(255, 0, 0), 2);
//            circle(result, Point2i(int(faces.at<float>(0, 6)), int(faces.at<float>(0, 7))), 2, Scalar(0, 0, 255), 2);
//        }
//
//        // Visualize results
//        imshow("Live", result);
//
//        int key = waitKey(1);
//        if (key == ' ')
//        {
//            key = 0;  // handled
//        }
//
//        if (key > 0)
//            break;
//
//    }
//    cout << "Done." << endl;
//    return 0;
//}