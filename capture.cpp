/**
Capturar Imagenes del video
g++ -std=c++11 capture.cpp `pkg-config opencv --cflags --libs` -o capture && ./capture
**/
#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#define windowName "video"

#define video_path "D:/opt/windows/Microsoft/VisualStudio/repos/CameraCalibration/cam1/anillos.mp4"
#define image_path "D:/opt/windows/Microsoft/VisualStudio/repos/CameraCalibration/QtFrontoParallel/imgs/cam1/"

int main(){
	cv::VideoCapture cap(video_path);

	cv::namedWindow(windowName,0);
	cv::resizeWindow(windowName,1000,1000);

	cv::Mat frame;

    int count = 30;
	std::cout << "==================================\n";
	std::cout << "Opciones:\n" << "\tx : Capturar Imagen\n" << "\tc : Pasar al Siguiente\n" << "\tESC : Terminar\n";
	std::cout << "==================================\n";
	while(true){
		cap >> frame;
		if(frame.empty()) break;

		cv::imshow(windowName,frame);

		int key = cv::waitKey(10000);

		bool c = true; // Check key for continue

		switch(key){
			// Capture Frame
			case 'x':{
                std::string str = image_path + std::to_string(count) + ".jpg";
				bool captured = cv::imwrite(str, frame);
				if(captured){
					std::cout << "Imagen Capturada\n" << "escrito en: " << str << std::endl;	
					count++;
				} 
				else std::cout << "Problema al Capturar Imagen\n";
				break;
			}
			case 27:{
				c = false;
				break;
			}
			case 'c': //Pasar al siguiente frame
				break;
			default:
				break;
		}

		if(c) continue;
		else break;
	}

	//terminando el programa
    cap.release();
    cv::destroyAllWindows();
}
