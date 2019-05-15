#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <math.h>
#include <bitset>
#include "address_map_arm_brl4.h"

/****** FPGA SETUP ******/
#define HW_REGS_BASE          0xff200000
#define HW_REGS_SPAN          0x00005000

#define FPGA_ONCHIP_BASE      0xC8000000
#define FPGA_ONCHIP_SPAN      0x00080000

// video display
#define SDRAM_BASE            0xC0000000
#define SDRAM_END             0xC3FFFFFF


/* function prototypes */
void VGA_text (int, int, char *);
void VGA_text_clear();
void write_feedback(int, int); 
void setup_feedback_text(); 

// HPS onchip memory (HPS side!)
volatile unsigned int * hps_onchip_ptr = NULL;
void *hps_onchip_virtual_base;

// the light weight buss base
void *h2p_lw_virtual_base;

// RAM FPGA command buffer
volatile unsigned int * sram_ptr = NULL;
void *sram_virtual_base;

// image buffer
unsigned int * sdram_image_ptr = NULL; //int
//volatile unsigned int * right_image_ptr = NULL; //int
void *image_virtual_base;
// /dev/mem file id
int fd;

// character buffer
volatile unsigned int * vga_char_ptr = NULL ;
void *vga_char_virtual_base;

/***PIO SetUp***/

#define FPGA_PIO_MARKER1 0x00
#define FPGA_PIO_FEEDBACK 0x10
#define FPGA_PIO_START 0x20
#define FPGA_PIO_THRESHOLD 0x30
#define FPGA_PIO_MARKER2 0x40
#define FPGA_PIO_MARKER3 0x50
#define FPGA_PIO_MARKER_NO 0x60

volatile unsigned int *feedback_pio_ptr = NULL;
volatile unsigned int *marker1_coord_pio_ptr = NULL;
volatile unsigned int *marker2_coord_pio_ptr = NULL;
volatile unsigned int *marker3_coord_pio_ptr = NULL;
volatile unsigned int *start_signal_pio_ptr = NULL;
volatile unsigned int *threshold_pio_ptr = NULL;
volatile unsigned int *marker_number = NULL; 
volatile int x1_start; 
volatile int x2_start; 
volatile int x3_start; 

volatile int y1_start; 
volatile int y2_start; 
volatile int y3_start; 

volatile int z1_start; 
volatile int z2_start; 
volatile int z3_start; 


/****** Aruco SETUP ******/
using namespace cv;
using namespace std;

void printBinary(unsigned long myNumber)
{
    int numberOfBits = sizeof(unsigned long)*16;
    for (int i=numberOfBits-1; i>=0; i--) {
        bool isBitSet = (myNumber & (1<<i));
        if (isBitSet) {
            cout << "1";
        } else {
            cout << "0";
        }
    }
}
unsigned char reverse(unsigned char b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}

int main (int argc, char** argv){

    // ============= FPGA SetUp ==================
    // Open /dev/mem
    if ((fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1) {
        printf("ERROR: could not open \"/dev/mem\"...\n");
        return(1);
    }

    // get virtual addr that maps to physical
    // for light weight bus
    h2p_lw_virtual_base = mmap(NULL, HW_REGS_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, HW_REGS_BASE);
    if (h2p_lw_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap1() failed...\n");
        close(fd);
        return(1);
    }
    marker1_coord_pio_ptr = (unsigned int*)( ((uint8_t*)h2p_lw_virtual_base) + FPGA_PIO_MARKER1);
    marker2_coord_pio_ptr = (unsigned int*)( ((uint8_t*)h2p_lw_virtual_base) + FPGA_PIO_MARKER2);
    marker3_coord_pio_ptr = (unsigned int*)( ((uint8_t*)h2p_lw_virtual_base) + FPGA_PIO_MARKER3);
    feedback_pio_ptr = (unsigned int *)( ((uint8_t*)h2p_lw_virtual_base) + FPGA_PIO_FEEDBACK);
    start_signal_pio_ptr = (unsigned int*)( ((uint8_t*)h2p_lw_virtual_base) + FPGA_PIO_START);
    threshold_pio_ptr = (unsigned int*)( ((uint8_t*)h2p_lw_virtual_base) + FPGA_PIO_THRESHOLD);
    marker_number = (unsigned int*)( ((uint8_t*)h2p_lw_virtual_base) + FPGA_PIO_MARKER_NO); 
    // FPGA SRAM SetUp
    sram_virtual_base = mmap(NULL, FPGA_ONCHIP_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, FPGA_ONCHIP_BASE); //fp

    if (sram_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap3() failed...\n");
        close(fd);
        return(1);
    }
    // Get the address that maps to the RAM buffer
    sram_ptr = (unsigned int *)(sram_virtual_base);


    // SDRAM MemMap: get virtual addr that maps to physical
    image_virtual_base = mmap(NULL, SDRAM_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, SDRAM_BASE); //SDRAM_BASE

    if (image_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap3() failed...\n");
        close(fd);
        return(1);
    }
    // Get the address that maps to the FPGA pixel buffer
    sdram_image_ptr = (unsigned int *)(image_virtual_base);

    // === get VGA char addr =====================
    // get virtual addr that maps to physical
    vga_char_virtual_base = mmap( NULL, FPGA_CHAR_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, FPGA_CHAR_BASE );
    if( vga_char_virtual_base == MAP_FAILED ) {
        printf( "ERROR: mmap2() failed...\n" );
        close( fd );
        return(1);
    }

    // Get the address that maps to the character
    vga_char_ptr =(unsigned int *)(vga_char_virtual_base);

    // ============= Aruco Main ==================
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(cv::aruco::DICT_4X4_50));
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    readDetectorParameters("detector_params.yml", detectorParams);
    
    VideoCapture inputVideo;
    inputVideo.open(0);

    //setup system control
    int start_signal = (*start_signal_pio_ptr); 
    (*threshold_pio_ptr) = 10;
    char callibrate = 0; 
    setup_feedback_text(); 
    	
    while(inputVideo.grab()) {

        Mat image; //(480, 640, CV_8UC3);
        inputVideo.retrieve(image);

        // split image
        Mat left, right;  //(480, 640, CV_8UC3), right(480, 640, CV_8UC3);
        image.copyTo(left);
        image.copyTo(right);
        left = left(cv::Rect(0, 0, left.cols/2, left.rows));
        right = right(cv::Rect(right.cols/2, 0, right.cols/2, right.rows));
        //printf("feedback: %d start: %d\n", *feedback_pio_ptr, *start_signal_pio_ptr);
        //printf("%i, %i\n", left.size().height, right.size().width);
        // write image to memory
        unsigned short one_img[76800];
        int j = 0;
        for(int y = 0; y < 240; y++) {
            for(int x = 0; x < 320; x++) {
                Vec3b bgrPixel_l = left.at<Vec3b>(y,x);
                unsigned short b_l = bgrPixel_l.val[0];
                unsigned short g_l = bgrPixel_l.val[1];
                unsigned short r_l = bgrPixel_l.val[2];

                unsigned short bgr_l = (((b_l*3)/ 255) << 6) + (((g_l*7)/ 255) << 3) + (((r_l*7)/ 255)) ;

                Vec3b bgrPixel_r = right.at<Vec3b>(y,x);
                unsigned short b_r = bgrPixel_r.val[0];
                unsigned short g_r = bgrPixel_r.val[1];
                unsigned short r_r = bgrPixel_r.val[2];

                unsigned short bgr_r = (((b_r*3)/ 255) << 6) + (((g_r*7)/ 255) << 3) + (((r_r*7)/ 255)) ;
                one_img[j] = bgr_l + (bgr_r << 8);

                j+=1;

            }
        }
        
        memcpy((sdram_image_ptr), (one_img), sizeof(one_img));
	
      	//check if start
	      start_signal = (*start_signal_pio_ptr);  
        
        if(start_signal) {
          
          // detect on left
          vector< int > ids_left;
          vector< vector< Point2f > > corners_left, rejected_left;
          aruco::detectMarkers(left, dictionary, corners_left, ids_left, detectorParams, rejected_left);
          /*if (ids_left.size() > 0){
              cv::aruco::drawDetectedMarkers(left, corners_left, ids_left);
          }*/
  
          // detect on right
          vector< int > ids_right;
          vector< vector< Point2f > > corners_right, rejected_right;
          aruco::detectMarkers(right, dictionary, corners_right, ids_right, detectorParams, rejected_right);
          /*if (ids_right.size() > 0){
              cv::aruco::drawDetectedMarkers(right, corners_right, ids_right);
          }*/
          
          //printf("detected: %i %i\n", ids_left.size(), ids_right.size());
  	//char feedback_buffer[50]; 
          int ids[3] = {43,42,38};
  	      int feedback = 0; 
          for (int i = 0; i < 3; i++){
              int found = 0;
              int id = ids[i];
              for (int r = 0; r < ids_right.size(); r++){
                  if(ids_right[r] == id){
                      for (int l = 0; l < ids_left.size(); l++) {
                          if(ids_left[l] == id) {
                              int x = corners_right[r][0].x;
                              int y = corners_right[r][0].y;
                              int z = 1000/abs((corners_left[l][0].x-corners_right[r][0].x));
                              unsigned int pt = (x << 16) + (y << 8) + (z);
			      printf("id: %d, x = %d, y = %d, z = %d\n", id, x, y, z); 
                              // clear the text
                              VGA_text_clear();
                              setup_feedback_text();
                              char text_top_row[40] = "Detected marker\0";
                              VGA_text (1, 56, text_top_row);
                              //printf("found %d\n", id);
                              if(i == 0){ //43
				  *(marker_number) = i; 
                                  *(marker1_coord_pio_ptr) = pt;
  			      	                  feedback = *(feedback_pio_ptr);
                        				  //write_feedback(feedback, i);  
                        				  //printf("id: %d\n", id);  
			                        }
                              else if(i == 1){ //42
				  *(marker_number) = i; 
                                  *(marker2_coord_pio_ptr) = pt;
  			      	                  feedback = *(feedback_pio_ptr);
                        				  //write_feedback(feedback, i); 
                        				  //printf("id: %d\n", id);  
                              }
                              else{ //38
				  *(marker_number) = i; 
                                  *(marker3_coord_pio_ptr) = pt;
  			      	                  feedback = *(feedback_pio_ptr);
				                          //write_feedback(feedback, i); 
				                          //printf("id: %d\n", id);  
                              }
                              write_feedback(feedback, i);  
                              printf("id: %d feedback: %d start: %d\n", id, feedback, *start_signal_pio_ptr);
                              //write_feedback(feedback); 
  			   
  			                      //printf("%d, %d\n", ids_right[r], corners_right[r].size());
                              //printf("points right: %i: %f, %f, %i\n", id, corners_right[r][0].x, corners_right[r][0].y, z);
                              found = 1;
                              break;
                          }
                      }
                  }
                  if(found == 1){
                      //printf("found %i", i);
                      break;
                  }
              }
  
          }
    }

    char key = (char)waitKey(10);
    if(key == 27) break;
    }

    return 0;
}

/****************************************************************************************
 * Subroutine to send a string of text to the VGA monitor
****************************************************************************************/
void VGA_text(int x, int y, char * text_ptr)
{
    volatile char * character_buffer = (char *) vga_char_ptr ;  // VGA character buffer
    int offset;
    /* assume that the text string fits on one line */
    offset = (y << 7) + x;
    while ( *(text_ptr) )
    {
        // write to the character buffer
        *(character_buffer + offset) = *(text_ptr);
        ++text_ptr;
        ++offset;
    }
}

/****************************************************************************************
 * Subroutine to clear text to the VGA monitor
****************************************************************************************/
void VGA_text_clear()
{
    volatile char * character_buffer = (char *) vga_char_ptr ;  // VGA character buffer
    int offset, x, y;
    for (x=0; x<79; x++){
        for (y=0; y<59; y++){
    /* assume that the text string fits on one line */
            offset = (y << 7) + x;
            // write to the character buffer
            *(character_buffer + offset) = ' ';
        }
    }
}

/****************************************************************************************
 * Write Feedback
****************************************************************************************/

void setup_feedback_text() {
  volatile char * character_buffer = (char *) vga_char_ptr ;
  //VGA_text_clear(); 
  int offset1 = (35 << 7) + 15;
  *(character_buffer + offset1) = ' ';
  char feedback_buffer[40] = "Marker 1:\0"; 
  VGA_text (15, 35, feedback_buffer);
  
  int offset2 = (40 << 7) + 15;
  *(character_buffer + offset2) = ' ';
  char feedback_buffer2[40] = "Marker 2:\0"; 
  VGA_text (15, 40, feedback_buffer2);
  
  int offset3 = (45 << 7) + 15;
  *(character_buffer + offset3) = ' ';
  char feedback_buffer3[40] = "Marker 3:\0"; 
  VGA_text (15, 45, feedback_buffer3);
  
}

void write_feedback(int feedback, int i) {
      int y = 0; 
      if(feedback == 3) {
           y = 50; 
           char feedback_buffer1[40] = "Callibrating\0"; 
           VGA_text (50, y , feedback_buffer1);
      } else {
        if(i == 0) y = 35; 
        else if (i == 1) y = 40; 
        else y = 45; 	
        
        setup_feedback_text(); 
        if(feedback == 0) {
           	 char feedback_buffer[40] = "Stay still\0"; 
             VGA_text (50, y , feedback_buffer);
        } 
        else if(feedback == 1) {
    	      char feedback_buffer[40] = "Move down\0"; 
            VGA_text (50, y , feedback_buffer);
        } 
        else if(feedback == 2) {
  	        char feedback_buffer[40] = "Move up\0"; 
            VGA_text (50, y, feedback_buffer);
        } else if(feedback == 3) {
    	      char feedback_buffer[40] = "callibrating\0"; 
            VGA_text (50, 56 , feedback_buffer);
        }
      }

}
