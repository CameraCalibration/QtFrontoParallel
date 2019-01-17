#include "cameracalibrator.h"
#include "mainwindow.h"
#include "constants.h"
#include "image.h"
#include <time.h>
#include <iterator>
#include <set>
#include <fstream>

#define path "D:/opt/windows/Microsoft/VisualStudio/repos/CameraCalibration/img/test/"

int patternType = RINGS_GRID;
int noImages = 30; // Numero de imagenes para la Calibración
int noIterations = 30;
float squareSize = 0.044;//0.04540;//meters
//cv::Size imgPixelSize = Size(640,480); // Tamaño de la imagen
cv::Size patternSize = cv::Size(5,4);


CameraCalibrator::CameraCalibrator(QObject *parent) : QObject(parent)
{
    actived = true;
}

///
/// \brief CameraCalibrator::setVisualizer asigna el visualizador que se usara para el procesamiento
/// \param vis  MainWindow donde se visualizara los videos
///
void CameraCalibrator::setVisualizer(MainWindow *vis)
{
    visualizer = vis;
}

///
/// \brief CameraCalibrator::setActived setea el estado del procesamiento
/// \param act  Nuevo estado del procesamiento
///
void CameraCalibrator::setActived(bool act)
{
    actived = act;
}

void CameraCalibrator::setCurrentCalibrator(unsigned int calib)
{
    currCalib = calib;
}

///
/// \brief CameraCalibrator::setSizePattern setea el tamaño del patrón
/// \param nRows    Numero de filas que tiene el patron
/// \param nCols    Numero de columnas que tiene el patron
///
void CameraCalibrator::setSizePattern(int nRows, int nCols)
{
    numRows = nRows;
    numCols = nCols;
}

///
/// \brief CameraCalibrator::loadVideo se carga el video a partir de la ruta asignada
/// \param path     Ruta donde se encuentra el video
/// \return   Bool que indica si el video se cargo correctamente o no
///
bool CameraCalibrator::loadVideo(std::string path_video)
{
    pathVideo = path_video;
    video.open(path_video);
    if (!video.isOpened())
        return false;
    return true;
}

///
/// \brief CameraCalibrator::processingPattern realiza el procesamiento para la calibracion del patrón
///
void CameraCalibrator::processingPattern()
{
    cv::Mat frame;
    std::vector< std::vector<cv::Point3f> > objPoints; // Puntos de nuestro objeto(Patron de calibracion)
    // Suponemos que el patron se encuentra de forma paralela a la camara, y a una misma altura
    std::vector< std::vector<cv::Point2f> > imgPoints; // 2D Points en la Imagen(Pixels)

    // Inital Calibration
    objPoints.resize(1);
    calcBoardCornerPositions(cv::Size(5,4),squareSize,objPoints[0],patternType);
    objPoints.resize(noImages,objPoints[0]);

    bool isTracking; // Variable para ayudar a la función FindRingGridPattern
    std::vector<cv::Point2f> oldPoints; // Punto usados para el Tracking en RingGrid

    //Variables para guardar los Valores de Correccion
    double rms;
    cv::Mat cameraMatrix = cv::Mat::eye(3,3,CV_64F); // Matriz para guardar la camera Intrinsics
    cv::Mat distCoeffs = cv::Mat::zeros(8, 1,CV_64F); // Aqui guardamos los coeficientes de Distorsion
    std::vector<cv::Mat> rvecs,tvecs; //Vectores de rotacion y de traslacion para cada frame

    //Capturamos las matrices
    FOR(i,noImages){

        string filename = path + std::to_string(i)  +  ".jpg";

        frame = cv::imread(filename,CV_LOAD_IMAGE_COLOR);
        visualizer->visualizeImage(PROCFIN, ImageHelper::convertMatToQimage(frame), windowName);

        std::vector<cv::Point2f> PointBuffer;

        isTracking = false; // Para que busque en todas las imagenes
        cv::Mat frame_grid;
        bool found = findRingsGridPattern(frame, frame_grid, patternSize, PointBuffer, isTracking,oldPoints);
        visualizer->visualizeImage(PROC6, ImageHelper::convertMatToQimage(frame_grid), "Patter Detected");

        if(found){
            imgPoints.push_back(PointBuffer);
        }
        else{
            cout << "Patron no encontrado: " << i << endl;;
        }

        int key = cv::waitKey(10);
        if(key == 27){
            cv::destroyAllWindows();
            return;
        }
    }

    // Calibracion Iterativa
    vector<float> rms_set;
    std::vector<cv::Point2f> fronto_corners = getFrontoParallelCorners(frame.size(),patternSize);

    FOR(it,noIterations)
    {
        // cout << "=================================\n";
        // cout << "           Iteracion " << it << endl;
        // cout << "=================================\n";
        // Limpiamosc variables
        rvecs.clear(); tvecs.clear();
        // cout << imgPoints.size() << endl;

        // Comenzamos la Calibracion
        rms = cv::calibrateCamera(objPoints,imgPoints, frame.size(),cameraMatrix,distCoeffs,rvecs,tvecs);
        cout << it << " " << cameraMatrix.at<double>(0,0) << " " << cameraMatrix.at<double>(1,1) <<
        " " << cameraMatrix.at<double>(0,2) << " " << cameraMatrix.at<double>(1,2) << " " << rms << " ";

        rms_set.push_back(rms);

        std::vector< std::vector<cv::Point2f> > imgPoints2;

        vector<float> v; // Para sacar un promedio de las colinealidades del vector

        // Mostrar imagens sin Distorsion
        FOR(i,noImages){
            string filename = path + std::to_string(i)  +  ".jpg";
            frame = cv::imread(filename,CV_LOAD_IMAGE_COLOR);
            visualizer->visualizeImage(PROC1, ImageHelper::convertMatToQimage(frame), "Input");
            visualizer->visualizeImage(PROCFIN, ImageHelper::convertMatToQimage(frame), "Input");

            cv::Mat temp = frame.clone();
            cv::Mat OptimalMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, frame.size(), 1.0);
            cv::undistort(temp,frame,cameraMatrix,distCoeffs,OptimalMatrix);
            visualizer->visualizeImage(PROC2, ImageHelper::convertMatToQimage(frame), "Undistort");

            std::vector<cv::Point2f> PointBuffer;
            cv::undistortPoints(imgPoints[i], PointBuffer, cameraMatrix, distCoeffs, cv::noArray(),OptimalMatrix);

            float m = getAvgColinearityFromVector( PointBuffer, patternSize );
            v.push_back(m);

            // Almacenamos solo cuatro esquinas
            std::vector<cv::Point2f> corners1 = extractCorners(PointBuffer,patternSize);

            cv::Mat H = cv::findHomography(corners1,fronto_corners);

            //Transformacion Fronto Parallel
            cv::Mat imgWarp;
            cv::warpPerspective(frame,imgWarp,H,Size(300,240));
            visualizer->visualizeImage(PROC3, ImageHelper::convertMatToQimage(imgWarp), "FrontoParallel");

            PointBuffer.clear();
            isTracking = false; // Para que busque en todas las imagenes
            cv::Mat frame_grid;
            bool found2 = findRingsGridPattern(imgWarp, frame_grid, patternSize, PointBuffer, isTracking,oldPoints);
            visualizer->visualizeImage(PROC4, ImageHelper::convertMatToQimage(frame_grid), "FP Grid");

            if(!found2){
                cout << "no se pudo encontrar el patron en la proyeccion FrontoParallel: " << i << endl;
                cv::destroyAllWindows();
                return;
            }

            //Transformacion Fronto Parallel Inversa
            cv::Mat imgWarp_inv;
            cv::warpPerspective(imgWarp,imgWarp_inv,H.inv(),frame.size());

            vector<Point2f> points_buffer2;
            cv::perspectiveTransform( PointBuffer, points_buffer2, H.inv() );

            std::vector<cv::Point2f> corrected_points;
            // Distorsión Inversa
            cv::Mat temp_inv = imgWarp_inv.clone();
            cv::undistort(temp_inv,imgWarp_inv,OptimalMatrix,-distCoeffs,cameraMatrix);
            cv::undistortPoints(points_buffer2,corrected_points,OptimalMatrix,-distCoeffs,cv::noArray(),cameraMatrix);

            cv::drawChessboardCorners(imgWarp_inv, patternSize, corrected_points, true);
            cv::drawChessboardCorners(imgWarp_inv, patternSize, imgPoints[i], true);

            imgPoints2.push_back( corrected_points );

            visualizer->visualizeImage(PROC5, ImageHelper::convertMatToQimage(imgWarp_inv), "Reprojection");
            visualizer->visualizeImage(PROC6, ImageHelper::convertMatToQimage(imgWarp_inv), "Reprojection");

            int key = cv::waitKey(10);
            if(key == 27){
                cv::destroyAllWindows();
                return;
            }

        }

        FOR(i,noImages)
            FOR(j,patternSize.width * patternSize.height){
                imgPoints[i][j].x = (imgPoints[i][j].x +  imgPoints2[i][j].x) / 2.0;
                imgPoints[i][j].y = (imgPoints[i][j].y +  imgPoints2[i][j].y) / 2.0;

                /*imgPoints[i][j].x = imgPoints2[i][j].x;
                imgPoints[i][j].y = imgPoints2[i][j].y;*/
            }

        cout << printAvgColinearity(v) << endl;
    }

    std::sort( rms_set.begin(), rms_set.end() );

    //terminando el programa
    cv::destroyAllWindows();
}

///
/// \brief CameraCalibrator::initProcessing inicia el proceso de calibración de la cámara
///
void CameraCalibrator::initProcessing(unsigned int pattSelected)
{
    visualizer->cleanImage(PROC1);
    visualizer->cleanImage(PROC2);
    visualizer->cleanImage(PROC3);
    visualizer->cleanImage(PROC4);
    visualizer->cleanImage(PROCFIN);

    processingPattern();
}
