Project RCJ 2014
================
by Arne Baeyens

This repository contains the source code I wrote for my image-processing robot for the RoboCup Junior competition in may 2014. It recognizes a line and does object tracking at the same time and sends the line's position and angle over the serial port for further processing.
About my robot: http://www.raspberrypi.org/an-image-processing-robot-for-robocup-junior/


##How to compile the program:

1. Install the openCV library
----------------------------------------
    sudo apt-get install libopencv-dev

2. Install the raspicam library
----------------------------------------
Follow Emil Valkov's instructions in his README
https://github.com/robidouille/robidouille/blob/master/raspicam_cv/README but replace in step 3 before running the 'make' command the following files with those in my repository:
* `RaspiCamCV.c`
* `Makefile`

3. Compile the C++ program
----------------------------------------
Copy the C++ file 'RCJ_2014' to a directory somewhere on your Pi
and compile with 

    g++ "%f" -lopencv_highgui -lopencv_core -lopencv_legacy -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_imgproc -lpthread -lm -L/home/pi/git/raspberrypi/userland/build/lib -lmmal_core -lmmal -l mmal_util -lvcos -lbcm_host -lX11 -lXext -lrt -lstdc++ -L/home/pi/git/robidouille/raspicam_cv -lraspicamcv -I/usr/include/opencv -o "%e" -L/usr/local/lib -lwiringPi

You can use the above command with the geany editor, but if compiling in a terminal you have to change "%f" to the name of the output executable and "%e" to the name of the input c++ file.
For any questions, please refer to the forum thread:
http://www.raspberrypi.org/forums/viewtopic.php?f=37&t=82151

4. Run the program
----------------------------------------
It is preferable to run the program in terminal as it gives verbose output during running.

    ./RCJ_2014

Credits
----------------------------------------
RaspiCam library: Emil Valkov and Pierre Raufast

Object tracking: Kyle Hounslow
 - https://www.youtube.com/watch?v=bSeFrPrqZ2A
 - or the complete source code: https://dl.dropboxusercontent.com/u/28096936/tuts/objectTrackingTut.cpp.

 
More info:

Forum thread: http://www.raspberrypi.org/forums/viewtopic.php?f=37&t=82151

Dwengo site: http://www.dwengo.org/node/46
