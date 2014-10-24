/*
 * Line follower algorithm
 * 
 * Version: 1.0
 * Date: 2014-10-24
 * 
 * 
 * Copyright 2014 Arne Baeyens
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * The object tracking part is mainly made by Kyle Hounslow. Check his excellent tutorial about object tracking: youtube.com/watch?v=bSeFrPrqZ2A
 * or the complete source code: dl.dropboxusercontent.com/u/28096936/tuts/objectTrackingTut.cpp.
 * Without the work of Pierre Raufast (thinkrpi.wordpress.com) and Emil Valvov (robidouille.wordpress.com) this project would have been much more difficult.
 * A big thanks to them.
 *
 * 2arne.baeyens@gmail.com
 *
 * 
 * For compiling, please refer to the README on github or the forum thread.
*/

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <math.h>

#include "RaspiCamCV.h"		// Emil Valcov`s library.

#include <wiringPi.h>
#include <wiringSerial.h>

#define PI 3.14159265


using namespace cv;
using namespace std;

// Debug settings variables.
bool debug = 1;			// Debugging on or off.
bool show_images = 1;	// Show images or not.
bool debug_mode;		// Variable to regulate debugging inside program.
//bool camera_show = 0;	// Show camera feed or not.

// Settings for object tracking and drawing.
const int FRAME_WIDTH = 320;
const int FRAME_HEIGHT = 240;

/* FUNCTIONS */
// For scanning and searching the line.
void scancircle( Mat& I, Point Mp, int radius, int look_angle, int width );
void scanline( Mat& I, Point Mp, int line_radius );
Point findLine( char scan_mode );

// For object tracking.
void drawObject(int x, int y,Mat &frame, Scalar color );
void trackObject(int *x, int *y, Mat &threshold);
  // Variables for green dot.
  int green_x = 160, green_y = 120;
  bool objectFound = false;
  //bool greenField = false;
  int green_area = 1;
  
// For external communication over UART.
void sendPositionData( void );
void sendCommand( char command_1, char command_2 );
void initSerialPort( void );
  int fd;	// file descriptor for serial port.
  char data_received = 0;	// Character to store answer from microcontroller.

bool test_inimage( Mat& I, short x_cor, short y_cor );

void loadTime( float *t_load_pointer );

void dataDo( char rx_data );

int lineangle( void );	// Returns the angle of the line (0 - 360 degrees), using global variables.

int first_angle=0;


// Data about scans.
uchar scandata[640];		// Array to contain scan values.
short scan_w;				// Number of (active) values in scandata array.
Point scanpoint;			// Last scanpoint.
Point line_point;			// Last x coordinates of found line.
vector<Point> line_points(7);	// vector om de laatste gevonden punten bij te houden.
Point first_scanpoint;		// Eerst gevonden punt.

int scan_counter = 0;		// Variable to count how many scans are done.

short scan_radius;
short sc_strt, sc_end;

short new_scan_radius1 = 140;

// Variables for trackbars.
int scan_height_reg = 220;
int scan_posx_reg = 160;
int scan_radius1_reg = 55;
int scan_radius2_reg = 18;
int look_angle_reg = 180;
int look_width_reg = 160;

int H_min_reg = 50;
int H_max_reg = 87;
int S_min_reg = 71;
int S_max_reg = 256;	// 133
int V_min_reg = 80;		// 90
int V_max_reg = 256;	// 197


// The calculated line error for transmittment
short line_error;
short P_Error, I_Error;

// Some variables for time measurement.
float time_e;
double time_running = (double)getTickCount()/getTickFrequency();
double time_old = 0;

// Matrices.
  Mat image;			// image from camera.
  Mat gray_image;		// gray camera image.
  Mat HSV_image;		// HSV converted image - for object tracking.
  Mat thresh_image;
  Mat eroded_image;
  Mat scan_image( 256, 640, CV_8UC3, Scalar( 255, 255, 255));		// to graph greyscale values on.
  Mat D_scan_image( 256, 640, CV_8UC3, Scalar( 255, 255, 255));		// to graph the differences in greyscale values.
  //Mat exp_image( 240, 320, CV_8U, Scalar( 255 ));					// for experimentation purposes.
  
  Mat erodeElement = getStructuringElement( MORPH_RECT, Size(4,4));	// matrix element to erode image - eroding removes small spots.
  
  vector<vector<Point> > contours;		// Variables for object tracking.
  vector<Vec4i> hierarchy;


int main(int argc, const char** argv){
  RaspiCamCvCapture * capture = raspiCamCvCreateCameraCapture(0); // Index doesn't really matter

  // Some strings for the trackbars.
  const string track_1 = "trackbar window 1";
  const string track_2 = "trackbar window 2";
  
  namedWindow( "camera image", CV_WINDOW_AUTOSIZE );
  namedWindow( "scan image", CV_WINDOW_AUTOSIZE );
  //namedWindow( "gray image", CV_WINDOW_AUTOSIZE );
  namedWindow( "derivative scan", CV_WINDOW_AUTOSIZE );
  namedWindow( "HSV image", CV_WINDOW_AUTOSIZE );
  namedWindow( "thresh image", CV_WINDOW_AUTOSIZE );
  //namedWindow( "experiment", CV_WINDOW_NORMAL );
  //cvResizeWindow( "experiment", 640, 480 );


  // Create some trackbars.
  namedWindow( track_1, CV_WINDOW_NORMAL );
  cvResizeWindow( "trackbar window 1", 320, 10 );
  createTrackbar( "scan height", track_1, &scan_height_reg, 239 );
  createTrackbar( "scan pos x", track_1, &scan_posx_reg, 319 );
  createTrackbar( "scan radius 1", track_1, &scan_radius1_reg, 320 );
  createTrackbar( "scan radius 2", track_1, &scan_radius2_reg, 160 );
  createTrackbar( "look angle", track_1, &look_angle_reg, 360 );
  createTrackbar( "look width", track_1, &look_width_reg, 360 );
  
  namedWindow( track_2, CV_WINDOW_NORMAL );
  cvResizeWindow( "trackbar window 2", 320, 10 );
  createTrackbar( "H_MIN", track_2, &H_min_reg, 256 );
  createTrackbar( "H_MAX", track_2, &H_max_reg, 256 );
  createTrackbar( "S_MIN", track_2, &S_min_reg, 256 );
  createTrackbar( "S_MAX", track_2, &S_max_reg, 256 );
  createTrackbar( "V_MIN", track_2, &V_min_reg, 256 );
  createTrackbar( "V_MAX", track_2, &V_max_reg, 256 );

  initSerialPort();	// Initiate the serial port.

  first_scanpoint = Point(160, scan_height_reg);

  // Debug settings.
  //debug=1;
  //show_images=1;
  
  do {
	//cout << "getting new image..." << endl;
    image = raspiCamCvQueryFrame(capture);	// Load image from camera.
    //cout << "  new image acquired" << endl;

	// Convert colors.
    cvtColor( image, HSV_image, COLOR_RGB2HSV );
    cvtColor( image, gray_image, CV_RGB2GRAY );	// Convert it to a gray image.

	// Threshold and erode image.
    inRange( HSV_image, Scalar( H_min_reg, S_min_reg, V_min_reg ), Scalar( H_max_reg, S_max_reg, V_max_reg ), thresh_image );
    erode( thresh_image, eroded_image, erodeElement );
    
    if(show_images==1){
		imshow( "HSV image", HSV_image );
		imshow( "thresh image", thresh_image );
	}
    trackObject( &green_x, &green_y, eroded_image );

	if(objectFound==true && show_images==1) drawObject( green_x, green_y, image, Scalar( 0, 255, 0 ));


    /* ZERO SCAN */
    scan_counter=0;
    if(debug==1) debug_mode=1;
    scanline( gray_image, Point(first_scanpoint.x, scan_height_reg), new_scan_radius1 );
    first_scanpoint = findLine( 0 );			// Keep point in memory for next iteration.
    debug_mode=0;
    new_scan_radius1 = scan_radius1_reg;
    
    /* FIRST SCAN */
    scancircle( gray_image, line_points[0], scan_radius2_reg, first_angle, look_width_reg );
    findLine( 1 );
    first_angle=lineangle();					// Keep angle in memory for next iteration.
    if(first_angle<-45) first_angle = -45;
    else if(first_angle>45) first_angle = 45;
    
    /* SECOND SCAN */
    scancircle( gray_image, line_points[0], scan_radius2_reg, first_angle, 180 );
    findLine( 1 );

	/* THIRD SCAN */
    scancircle( gray_image, line_points[0], scan_radius2_reg, lineangle(), 180 );
    findLine( 1 );

    P_Error=first_scanpoint.x-image.cols/2;

	/* FOURTH SCAN */
    scancircle( gray_image, line_points[0], scan_radius2_reg, lineangle(), 180 );
    findLine( 1 );

    if(show_images==1) imshow( "camera image", image );		// Show camera view after editing.
    
    
  } while (waitKey(10) != 'q');

  //cvDestroyWindow("RaspiCamTest");
  raspiCamCvReleaseCapture(&capture);
  return 0;
}

void scancircle( Mat& I, Point Mp, int radius, int look_angle, int width ){
    int i,j;
    int n = 0;
    Point dot_pos, dot2_pos;

    scanpoint = Mp;

    scan_radius = radius;
    sc_strt = Mp.x - radius;
    sc_end = Mp.x + radius;

    Point end_point_left = Point( scanpoint.x - sin(PI*(look_angle+180-width/2)/180)*scan_radius, scanpoint.y - cos(PI*(look_angle+180-width/2+180)/180)*scan_radius );
    Point end_point_right = Point( scanpoint.x - sin(PI*(look_angle+180+width/2)/180)*scan_radius, scanpoint.y + cos(PI*(look_angle+180+width/2)/180)*scan_radius );
    if(show_images==1) {
	circle( image, end_point_left, 5, Scalar( 255, 0, 0 ), -1, 8, 0 );
    	circle( image, end_point_right, 5, Scalar( 0, 255, 0 ), -1, 8, 0 );
    }

    for( i=0; i<radius*2; i++ ){
      dot_pos.x = round(Mp.x-radius*sqrt(1-pow(((float)i/radius)-1,2)));
      dot2_pos.x = round(Mp.x-radius*sqrt(1-pow((((float)i+1)/radius)-1,2)));

      dot_pos.y = Mp.y-i+radius;
      if( test_inimage( I, dot_pos.x, dot_pos.y)) scandata[n]=I.at<uchar>(dot_pos);
      else scandata[n]=scandata[n-1];
      n++;
      /*I.at<uchar>(dot_pos) = 0;
      waitKey(10);
      imshow( "experiment", I );*/

      for( j=1; j<dot2_pos.x-dot_pos.x; j++){
	if(test_inimage( I, dot_pos.x, dot_pos.y+j)) scandata[n]=I.at<uchar>(dot_pos.y, dot_pos.x+j);
        else scandata[n] = scandata[n-1];
	n++;
	/*I.at<uchar>(dot_pos.y, dot_pos.x+j)=120 ;
        waitKey(10);
        imshow( "experiment", I );*/
      }
      for( j=1; j<dot_pos.x-dot2_pos.x; j++){
        if( test_inimage( I, dot_pos.x, dot_pos.y-j)) scandata[n]=I.at<uchar>(dot_pos.y, dot_pos.x-j);
        else scandata[n] = scandata[n-1];
	n++;
	/*I.at<uchar>(dot_pos.y, dot_pos.x-j)=120 ;
        waitKey(10);
        imshow( "experiment", I );*/
      }
    }

    for( i=radius*2-1; i>=0; i-- ){
      dot_pos.x = round(Mp.x+radius*sqrt(1-pow(((float)i/radius)-1,2)));
      dot2_pos.x = round(Mp.x+radius*sqrt(1-pow((((float)i+1)/radius)-1,2)));

      dot_pos.y = Mp.y-i+radius;

      for( j=dot2_pos.x-dot_pos.x-1; j>0; j--){
        if(test_inimage( I, dot_pos.x, dot_pos.y+j)) scandata[n]=I.at<uchar>(dot_pos.y, dot_pos.x+j);
        else scandata[n] = scandata[n-1];
        n++;
        /*I.at<uchar>(dot_pos.y, dot_pos.x+j)=120 ;
        waitKey(100);
        imshow( "experiment", I );*/
      }
      for( j=dot_pos.x-dot2_pos.x-1; j>0; j--){
        if(test_inimage( I, dot_pos.x, dot_pos.y-j)) scandata[n]=I.at<uchar>(dot_pos.y, dot_pos.x-j);
        else scandata[n] = scandata[n-1];
        n++;
        /*I.at<uchar>(dot_pos.y, dot_pos.x-j)=120 ;
        waitKey(100);
        imshow( "experiment", I );*/
      }

      if(test_inimage( I, dot_pos.x, dot_pos.y)) scandata[n]=I.at<uchar>(dot_pos);
      else scandata[n]=scandata[n-1];

      n++;
      /*I.at<uchar>(dot_pos.y, dot_pos.x) = 0;
      waitKey(100);
      imshow( "experiment", I );*/
    }

    scan_w = n-1;
    //cout << scan_w << endl;

    int look_strt = round((float)scan_w*(look_angle+180-width/2)/360);
    int look_end = round((float)scan_w*(look_angle+180+width/2)/360);

    for(i=0; i<look_strt; i++) scandata[i]=scandata[look_strt+1];
    for(i=look_end; i<scan_w; i++) scandata[i]=scandata[look_end-1];

    if(debug_mode==1){
      for(i=0; i<scan_w; i++) line( scan_image, Point(i,scan_image.rows), Point(i,scandata[i]), Scalar( 0, 0, 0 ), 1, 8 );
      //circle( I, Mp, 5, Scalar( 255 ), -1, 8, 0  );
      //imshow( "experiment", I );
    }

    //I = Mat( 240, 320, CV_8U, Scalar( 255 ));
}

void scanline( Mat& I, Point Mp, int line_radius ){
    // accept only char type matrices
    CV_Assert(I.depth() != sizeof(uchar));

    int channels = I.channels();

    int nRows = I.rows;
    int nCols = I.cols * channels;

    int i;

    scanpoint = Mp;

    scan_radius = line_radius;
    scan_w = line_radius*2;
    sc_strt = Mp.x - line_radius;
    sc_end = Mp.x + line_radius;

    uchar* p;
    p = I.ptr<uchar>(Mp.y);

    for( i=0; i<scan_w; i++){
      //cout << i << " " ;
      if(i+sc_strt<0) scandata[i]=p[0];             // Als er buiten het beeld gekeken wordt...
      else if(i+sc_strt>=I.cols) scandata[i]=p[I.cols-2];
      else scandata[i] = p[i+sc_strt];
      if(debug_mode==1) line( scan_image, Point(i,scan_image.rows), Point(i,scandata[i]), Scalar( 0, 0, 0 ), 1, 8 );
      //cout << int(scanline[i]) << " ";
      //cout << i << " ";
    }
    if( show_images == 1 ){
      line( image, Point(0, Mp.y), Point(image.cols, Mp.y), Scalar( 0, 0, 255 ), 1, 8 );
      if(Mp.x-line_radius>=0) circle( image, Point(Mp.x-line_radius, Mp.y), 5, Scalar( 255, 0, 0 ), -1, 8, 0 );
      if(Mp.x+line_radius<I.cols) circle( image, Point(Mp.x+line_radius, Mp.y), 5, Scalar( 255, 0, 0 ), -1, 8, 0 );
	}
}

Point findLine( char scan_mode )
{
    Point left_side;
    Point right_side;

    int i,j;

    int der_scan[639];
	der_scan[0]=0;
	der_scan[639]=0;

    for( i=1; i<scan_w-1; i++){
      der_scan[i]=int(scandata[i-1])-int(scandata[i+1]);
      if(debug_mode == 1) line( D_scan_image, Point(i,D_scan_image.rows/2), Point(i,D_scan_image.rows/2-der_scan[i]), Scalar( 255, 0, 0 ), 1, 8 );
      //cout << der_scan[i] << " ";
    }
    //cout << endl << endl;

    for( i=1; i<scan_w-1; i++ ) {
      if( der_scan[i] > left_side.y ){
        left_side.y = der_scan[i];
		left_side.x = i;
      }
      if( der_scan[i] < right_side.y ){
		right_side.y = der_scan[i];
		right_side.x = i;
      }
    }


    float line_pos = (right_side.x+left_side.x)/2;
    if ( scan_mode == 0 ) line_point = Point( line_pos+sc_strt, scanpoint.y );
    else if ( scan_mode == 1 ){
      line_pos=line_pos/scan_w;
      line_point = Point( scanpoint.x + cos(PI*((float)line_pos*2+0.5))*scan_radius, scanpoint.y + sin(PI*((float)line_pos*2+0.5))*scan_radius );
      if(show_images==1) circle( image, scanpoint, scan_radius, Scalar( 255, 0, 0 ), 1, 8, 0 );
    }

    if(line_point.x>=image.cols) line_point.x=image.cols-1;	// Vermijden dat het gevonden punt buiten het beeld ligt.
    else if(line_point.x<0) line_point.x = 0;

    if(show_images==1) circle( image, Point(line_point.x, line_point.y), 5, Scalar( 0, 0, 255 ), -1, 8, 0 );

      //circle( D_scan_image, Point( left_side.x+sc_strt, D_scan_image.rows/2-left_side.y ), 5, Scalar( 255, 0, 255 ), 2, 8, 0 );
      //circle( D_scan_image, Point( right_side.x+sc_strt, D_scan_image.rows/2-right_side.y ), 5, Scalar( 0, 255, 255 ), 2, 8, 0 );

    if(debug_mode==1){
		imshow( "scan image", scan_image );
        imshow( "derivative scan", D_scan_image );
       	scan_image = Mat( 256, 640, CV_8UC3, Scalar( 255, 255, 255));
      	D_scan_image = Mat( 256, 640, CV_8UC3, Scalar( 255, 255, 255));
    }

    //cout << "line_error: " << line_error << endl;
    //cout << endl;
    for(i=2;i>=0;i--) line_points[i+1]=line_points[i];	// Waarden doorschuiven.
    line_points[0]=line_point;				// Laatst gevonden punt opslaan in de lijst.

    return line_point;
}

void drawObject(int x, int y,Mat &frame, Scalar color){
	/* by Kyle Hounslow */
	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!

    //UPDATE:JUNE 18TH, 2013
    //added 'if' and 'else' statements to prevent
    //memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

	circle(frame,Point(x,y),10,color,1);
    if(y-15>0)
    line(frame,Point(x,y),Point(x,y-15),color,1);
    else line(frame,Point(x,y),Point(x,0),color,1);
    if(y+15<FRAME_HEIGHT)
    line(frame,Point(x,y),Point(x,y+15),color,1);
    else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),color,1);
    if(x-15>0)
    line(frame,Point(x,y),Point(x-15,y),color,1);
    else line(frame,Point(x,y),Point(0,y),color,1);
    if(x+15<FRAME_WIDTH)
    line(frame,Point(x,y),Point(x+15,y),color,1);
    else line(frame,Point(x,y),Point(FRAME_WIDTH,y),color,1);

	//putText(frame,i(x)+","+intToString(y),Point(x,y+30),1,1,Scalar(0,255,0),2);
}

void trackObject(int *x, int *y, Mat &workImage){
	/* by Kyle Hounslow */
	//cout << "/* Start tracking */" << endl;
    findContours(workImage, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
    //cout << "  hierarchy:		" << Mat(hierarchy) << endl;
    
    double refArea = 0;
    objectFound=false;
    
    if(hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		//cout << "  numObjects:		" << numObjects << endl;
		if(numObjects<10) {
			for(int index = 0; index >=0; index = hierarchy[index][1]) {		// Changed 0 to 1 to return position of closest intead of farthest object.
				//cout << "    index:		" << index << endl;
				Moments moment = moments((cv::Mat)contours[index]);
				green_area = moment.m00;
				cout<< "    area:		" << green_area << endl;
				
				if(green_area>50 && green_area>refArea){
					//cout << "area: " << area << endl;
					*x = moment.m10/green_area;
					*y = moment.m01/green_area;
					objectFound = true;
				}
				else{
					objectFound = false;
					*x=workImage.cols/2;
					*y=15;
				}
				//cout <<  "    *x:			" << *x << endl << "    *y:			" << *y << endl << endl;
			}
			//if(objectFound == true);
			//putText(image, "Object", Point(0,50), 2, 1, Scalar(0,255,0), 2);
			//drawObject(*x,*y,image);
		}
		else cout << "Too much noise!" << endl;
	}
	//cout << "/* End tracking   */" << endl << endl;
}

bool test_inimage( Mat& I, short x_cor, short y_cor ){
  if( x_cor >= 0 && y_cor >=0 && x_cor < I.cols && y_cor < I.rows ) return 1;
  else return 0;
}

int lineangle( void ){
  return round(atan2( (line_points[0].x-scanpoint.x), -(line_points[0].y-scanpoint.y) )*180/PI);
}

void loadTime( float *t_load_pointer ){
  time_running = (float)getTickCount()/getTickFrequency();
  *t_load_pointer=time_running-time_old;
  time_old=time_running;
}

void initSerialPort( void ){
  if((fd = serialOpen("/dev/ttyAMA0", 115200)) < 0) {
    cout << "Unable to open serial device: " << fd << endl;
    //return 1;
  }
}

void sendPositionData( void ){
  //short new_data;
  /*char msg[2];
  msg[0] = (line_error & 0xff00) >> 8;
  msg[1] = line_error & 0x00ff;*/
  //cout << "msg_data: " << int(msg[0]) << " and " << int(msg[1]) << " endline"<< endl;

  /*new_data = msg[0];
  new_data = (new_data<<8)+msg[1];
  cout << "send_data: " << new_data << endl;*/

  // send syntax: $RPIE<high_bits><low_bits>,*
  // example: $RPIE5k*
  serialPrintf( fd, "$PIR" );
  /*serialPutchar( fd, msg[0] );
  serialPutchar( fd, msg[1] );*/
  serialPutchar( fd, char(P_Error/2) );
  serialPutchar( fd, char(I_Error) );
  serialPutchar( fd, 0x2A );	// Send * to terminate the message.
  // Receive answer.
  if(serialDataAvail( fd ) == 1){
	data_received = serialGetchar( fd );
	cout << "data char: " << data_received << endl;
	dataDo( data_received );		// Do something with the received data.
  }
  serialFlush( fd );			// Clean the serial port.
}

void sendCommand( char command_1, char command_2 ){
  serialPrintf( fd, "$PIC" );
  serialPutchar( fd, command_1 );
  serialPutchar( fd, command_2 );
  serialPutchar( fd, 0x2A );	// Send * to terminate the message.
}

void dataDo( char rx_data ){
	if( rx_data == 'R' ){	
		first_scanpoint.x = 160;
		new_scan_radius1 = 140;
		first_angle=0;
	}
}
