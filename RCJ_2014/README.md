Project RCJ 2014
================
by Arne Baeyens

This repository contains the source code I wrote for my image-processing robot for the RoboCup Junior competition in may 2014. It uses a Raspberry Pi and Pi camera for the image recognition and a Dwengo microcontroller board to control the robot. The robot recognizes a line and does object tracking at the same time and sends the line's position and angle over the serial port for further processing. [Here](http://youtu.be/AsoLF6NsqBI) is a short video of the 'thinking' of the robot but the program also gives other output like graphs and threshold images.
About my robot: [raspberrypi.org/an-image-processing-robot-for-robocup-junior](http://www.raspberrypi.org/an-image-processing-robot-for-robocup-junior/).

Please note that the RaspiCamCV.c file and the Makefile are made by Emil Valkov and Pierre Raufast. I only edited them slightly.

##How to compile the program:

1. Install the openCV library
----------------------------------------
    sudo apt-get install libopencv-dev

2. Install the raspicam library
----------------------------------------
Follow Emil Valkov's instructions in his [README](https://github.com/robidouille/robidouille/blob/master/raspicam_cv/README) but replace in step 3 before running the 'make' command the `RaspiCamCV.c` file by that in my repository.

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
    
To exit the program, select one of the windows and press 'q'


Credits
----------------------------------------
RaspiCam library: Emil Valkov and Pierre Raufast
* [robidouille.wordpress.com](http://www.robidouille.wordpress.com)
* [thinkrpi.wordpress.com](http://www.thinkrpi.wordpress.com).

Object tracking: Kyle Hounslow
* [Tutorial: Real-Time Object Tracking Using OpenCV](https://youtube.com/watch?v=bSeFrPrqZ2A)
* or the complete source code: [objectTrackingTut.cpp](https://dl.dropboxusercontent.com/u/28096936/tuts/objectTrackingTut.cpp).

Other libraries:
* wiringPi library: [wiringpi.com](http://wiringpi.com/) by Gordon Henderson
* the openCV library: [opencv.org](http://opencv.org).
 
More info:

Forum thread: [raspberrypi.org/forums/viewtopic.php?f=37&t=82151](http://raspberrypi.org/forums/viewtopic.php?f=37&t=82151)

Dwengo site: [dwengo.org/node/46](http://dwengo.org/node/46)



TODO?
-----
* code line following algo using mmal libraries so it runs on the GPU - preferably at 90fps?
* community efforts are greatly appreciated.
