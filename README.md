# QtFrontoParallel
Implementation of Fronto Parallel algorithm in C++ and Qt.

# Requirements

**OS:** Windows 10 <br/>
**Compiler:** VisualStudio 2017 <br/>
**IDE:** QtCreator 4.8 <br/>
**Framework**: Qt 5.12.0 <br/>
**OpenCV:** 3.4.4

## How to Build

 *  Clone the repository inside the directory solution:
 
 ``
 git clone https://github.com/CameraCalibration/QtFrontoParallel.git
 ``
 *  Open QtCreator and open the file **QtFrontoParallel.pro**.
 *  Change the dataset directory in the file **cameracalibrator.cpp** using the flag (at line 11):
 ``
 #define path "(directory solution)/imgs/cam1"
 ``
 *  Change the **ASPECT** of the images to use in the file **cameracalibrator.cpp** (at line 12):
 ``
 #define ASPECT 16.0/9.0
 `` 
 *  Change the **number of frames** to use in the file **cameracalibrator.cpp** (at line 13):
 ``
 #define noImages 70
 ``
 *  Change the **number of iterations** to use in the file **cameracalibrator.cpp** (at line 13):
 ``
 #define noIterations 14
 ``
 *  Finally, build using the architecture of home.
 
 ## Run the Application
 
 Just press the start button:
 
 ![alt text](https://raw.githubusercontent.com/CameraCalibration/QtFrontoParallel/master/static/start_button.png "Start Button")
 
 ## Results
 
 *  Using **LiveCam Microsoft** (cam01) with 14 frames: 
 
![alt text](https://raw.githubusercontent.com/CameraCalibration/QtFrontoParallel/master/static/cam1_rms.png "RMS Cam01")

![alt text](https://raw.githubusercontent.com/CameraCalibration/QtFrontoParallel/master/static/cam1_centers.png "Centers Cam01")
 
 *  Using **PlayStation 2 Cam** (cam02) with 15 frames: 
 
![alt text](https://raw.githubusercontent.com/CameraCalibration/QtFrontoParallel/master/static/cam2_rms.png "RMS Cam02")

![alt text](https://raw.githubusercontent.com/CameraCalibration/QtFrontoParallel/master/static/cam2_centers.png "Centers Cam02")
 
 
## Contact
If you need any help you can contact me to:
 
**Author:** Daniel Palomino <br/>
**Mail:**   dapalominop@gmail.com
