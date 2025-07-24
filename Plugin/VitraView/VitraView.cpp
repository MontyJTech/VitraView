#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>  // For CharucoBoard
#include <opencv2/aruco.hpp>          // For GridBoard and Dictionary
#include <opencv2/calib3d.hpp>

std::vector<int> ids = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };

float halfWidth = 0.475f / 2.0f; //from the centre screen to centre of the marker
float halfHeight = 0.27f / 2.0f; //from the centre screen to centre of the marker
float halfMarkerSize = 0.03f;
float halfArucoSize = 0.02f;

//non-wide webcam:
//cameraMatrix = (cv::Mat_<double>(3, 3) << 545.459118, 0.000000, 314.770126, 0.000000, 546.914933, 233.265792, 0.000000, 0.000000, 1.000000);
//distCoeffs = (cv::Mat_<double>(1, 5) << 0.011073, -0.106461, -0.000920, -0.000030, 0.055461, CV_64F);

//wide webcam:
//cameraMatrix = (cv::Mat_<double>(3, 3) << 589.622580, 0.000000, 310.524711, 0.000000, 589.912712, 217.313359, 0.000000, 0.000000, 1.000000);
//distCoeffs = (cv::Mat_<double>(1, 5) << -0.369050, 0.160037, 0.000265, 0.000991, -0.072916);

std::vector<std::vector<cv::Point3f>> objPoints;

float XByColumn(int colNum);
float YByRow(int rowNum);
std::vector<cv::Point3f> MarkerCornerArrays(float xCentre, float yCentre);

void PrintMat(cv::Mat* mat);
int MainArucoBoardAttempt();


int main() {
    return MainArucoBoardAttempt();
}

int MainArucoBoardAttempt() {
    cv::aruco::DetectorParameters params;
    //params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    cv::Ptr<cv::aruco::DetectorParameters> paramsPtr = &params;

    // 0: 0 - 0 - top left
    float xByCol = XByColumn(0);
    float yByRow = YByRow(0);
    objPoints.push_back(MarkerCornerArrays(xByCol, yByRow));

    // 1: 1 - 0
    xByCol = XByColumn(1);
    yByRow = YByRow(0);
    objPoints.push_back(MarkerCornerArrays(xByCol, yByRow));

    // 2: 2 - 0
    xByCol = XByColumn(2);
    yByRow = YByRow(0);
    objPoints.push_back(MarkerCornerArrays(xByCol, yByRow));

    // 3: 3 - 0
    xByCol = XByColumn(3);
    yByRow = YByRow(0);
    objPoints.push_back(MarkerCornerArrays(xByCol, yByRow));

    // 4: 3 - 1
    xByCol = XByColumn(3);
    yByRow = YByRow(1);
    objPoints.push_back(MarkerCornerArrays(xByCol, yByRow));

    // 5: 3 - 2
    xByCol = XByColumn(3);
    yByRow = YByRow(2);
    objPoints.push_back(MarkerCornerArrays(xByCol, yByRow));

    // 6: 2 - 2
    xByCol = XByColumn(2);
    yByRow = YByRow(2);
    objPoints.push_back(MarkerCornerArrays(xByCol, yByRow));

    // 7: 1 - 2
    xByCol = XByColumn(1);
    yByRow = YByRow(2);
    objPoints.push_back(MarkerCornerArrays(xByCol, yByRow));

    // 8: 0 - 2
    xByCol = XByColumn(0);
    yByRow = YByRow(2);
    objPoints.push_back(MarkerCornerArrays(xByCol, yByRow));

    //// 9: 0 - 1
    xByCol = XByColumn(0);
    yByRow = YByRow(1);
    objPoints.push_back(MarkerCornerArrays(xByCol, yByRow));

    cv::Mat frame;
    cv::VideoCapture cap(0, cv::CAP_DSHOW);
    if (!cap.isOpened()) return -1;

    cv::Mat cameraMatrix, distCoeffs;

    cameraMatrix = (cv::Mat_<double>(3, 3) << 589.622580, 0.000000, 310.524711, 0.000000, 589.912712, 217.313359, 0.000000, 0.000000, 1.000000);
    distCoeffs = (cv::Mat_<double>(1, 5) << -0.369050, 0.160037, 0.000265, 0.000991, -0.072916);

    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    cv::Ptr<cv::aruco::Board> board = cv::aruco::Board::create(objPoints, dictionary, ids);
    cv::Vec3d smoothedTvec, smoothedRvec;

    while (cap.read(frame)) {
        std::vector<int> detectedIds;
        std::vector<std::vector<cv::Point2f>> detectedCorners, rejectedCandidates;
        cv::Ptr<cv::aruco::Dictionary> ptr = cv::makePtr<cv::aruco::Dictionary>(dictionary);
        cv::aruco::detectMarkers(frame, ptr, detectedCorners, detectedIds, paramsPtr, rejectedCandidates);
        //cv::aruco::refineDetectedMarkers(frame, board, detectedCorners, detectedIds, rejectedCandidates);

        cv::Vec3d rvec, tvec;
        int valid = cv::aruco::estimatePoseBoard(
            detectedCorners, detectedIds, board, cameraMatrix, distCoeffs, rvec, tvec
        );

        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(detectedCorners, 0.04f, cameraMatrix, distCoeffs, rvecs, tvecs);

        for (size_t i = 0; i < rvecs.size(); i++) {
            cv::drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.04f);
        }

        if (valid > 0) {
            cv::drawFrameAxes(frame, cameraMatrix, distCoeffs, rvec, tvec, 0.05f);
            cv::Mat R;
            cv::Rodrigues(rvec, R);

            cv::Mat cameraPosition = -R.t() * cv::Mat(tvec);

            PrintMat(&cameraPosition);
        }
        cv::imshow("Aruco Pose", frame);

        if (cv::waitKey(1) == 27) break;
    }

    return 0;
}

float XByColumn(int colNum) {
    const int totalCols = 4; // 4 markers along top/bottom
    if (colNum < 0 || colNum >= totalCols)
        return 0.f;

    float markerSize = halfMarkerSize * 2.0f;
    float monitorWidth = halfWidth * 2.0f;
    float spacing = ((monitorWidth - (totalCols * markerSize)) / (totalCols - 1)) + markerSize;

    return -halfWidth + halfMarkerSize + (spacing * colNum);
}

float YByRow(int rowNum) {
    const int totalRows = 3; // top, middle, bottom
    if (rowNum < 0 || rowNum >= totalRows)
        return 0.f;

    float markerSize = halfMarkerSize * 2.0f;
    float monitorHeight = halfHeight * 2.0f;
    float spacing = ((monitorHeight - (totalRows * markerSize)) / (totalRows - 1)) + markerSize;

    return -halfHeight + halfMarkerSize + (spacing * rowNum);
}

std::vector<cv::Point3f> MarkerCornerArrays(float xCentre, float yCentre) {
    return {
        {xCentre + halfArucoSize, yCentre + halfArucoSize, 0}, //br
        {xCentre - halfArucoSize, yCentre + halfArucoSize, 0},  //bl
        {xCentre - halfArucoSize, yCentre - halfArucoSize, 0}, //tl
        {xCentre + halfArucoSize, yCentre - halfArucoSize, 0} //tr
    };
}

void PrintMat(cv::Mat* mat) {
    std::cout << "\r";
    for (int i = 0; i < mat->cols; i++) {
        for (int j = 0; j < mat->rows; j++) {
            std::cout << "\t\t" << mat->at<double>(j, i) << " ";
        }
    }
}
