/*
 * Copyright (C) 2015 The Paparazzi Community
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/computer_vision/opticflow/opticflow_thread.c
 *
 */

// Sockets
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>

#include "opticflow_thread.h"


/////////////////////////////////////////////////////////////////////////
// COMPUTER VISION THREAD

// Video
#include "v4l/v4l2.h"
#include "resize.h"

// Payload Code
#include "visual_estimator.h"

// Downlink Video
//#define DOWNLINK_VIDEO 1

#ifdef DOWNLINK_VIDEO
#include "encoding/jpeg.h"
#include "encoding/rtp.h"
#endif

#include <stdio.h>
#define DEBUG_INFO(X, ...) ;


//------------------------ added by Peng Lu ---------------------------//

float yaw_ref;
//float yaw_ref_get;
//------------------------ added by Peng Lu ---------------------------//



static volatile enum{RUN,EXIT} computer_vision_thread_command = RUN;  /** request to close: set to 1 */

void computervision_thread_request_exit(void) {
  computer_vision_thread_command = EXIT;
}

void *computervision_thread_main(void *args)
{
  int thread_socket = *(int *) args;

  // Local data in/out
  struct CVresults vision_results;
  struct PPRZinfo autopilot_data;

  // Status
  computer_vision_thread_command = RUN;


  /* On ARDrone2:
   * video1 = front camera; video2 = bottom camera
   */
  // Create a V4L2 device
  //struct v4l2_device *dev = v4l2_init("/dev/video2", 320, 240, 10);
  
  //--------------------------------- changed by Peng LU-----------------------------//
  struct v4l2_device *dev = v4l2_init("/dev/video1", 1280, 720, 10);
  // image taken may be downsized 320 * 176, but not in this file
  //--------------------------------- changed by Peng LU-----------------------------//
  
  if (dev == NULL) {
    printf("Error initialising video\n");
    return 0;
  }

  // Start the streaming on the V4L2 device
  if(!v4l2_start_capture(dev)) {
    printf("Could not start capture\n");
    return 0;
  }

#ifdef DOWNLINK_VIDEO
  // Video Compression
  uint8_t *jpegbuf = (uint8_t *)malloc(dev->w * dev->h * 2);

  // Network Transmit
  struct UdpSocket *vsock;
  //#define FMS_UNICAST 0
  //#define FMS_BROADCAST 1
  vsock = udp_socket("192.168.1.255", 5000, 5001, FMS_BROADCAST);
#endif

  // First Apply Settings before init
  opticflow_plugin_init(dev->w, dev->h, &vision_results);

  while (computer_vision_thread_command == RUN) {

    // Wait for a new frame
    struct v4l2_img_buf *img = v4l2_image_get(dev);

    // Get most recent State information
    int bytes_read = sizeof(autopilot_data);
    while (bytes_read == sizeof(autopilot_data))
    {
      bytes_read = recv(thread_socket, &autopilot_data, sizeof(autopilot_data), MSG_DONTWAIT);
      if (bytes_read != sizeof(autopilot_data)) {
        if (bytes_read != -1) {
          printf("[thread] Failed to read %d bytes PPRZ info from socket.\n",bytes_read);
        }
      }
    }
    DEBUG_INFO("[thread] Read # %d\n",autopilot_data.cnt);

    // Run Image Processing with image and data and get results
    
//--------------------------------- changed by Peng LU-----------------------------//
       
    //opticflow_plugin_run(img->buf, &autopilot_data, &vision_results);
    
    
    
//struct v4l2_img_buf *img = v4l2_image_get(viewvideo.dev);
struct img_struct img_data_s;
img_data_s.buf = img->buf;
//uint8_t img_data[] = img_data_s.buf;

uint8_t img_data = img->buf;
 
//uint8_t img_data = viewvideo_thread();
 
//int im_w = viewvideo.dev->w;
//int im_h = viewvideo.dev->h;
//int row_chosen = im_h/2;


//uint8_t im_UYVY = img_data[0] + img_data[1];

//printf("%d\n", im_UYVY);


//------------------------ Perform ---------------------------
/*
        #define  row_chosen  300 
        #define  im_w     640/2
        #define  TH_U_U   0.7
        #define  TH_U_L   0.3
        #define  TH_V_U   0.5
        #define  TH_V_L   0.3
        #define  threshold 15
        //#define  downsize_factor 1
        
        //FILE *fp_Y, *fp_U, *fp_V;
        int   i, j, p;
        float im_Y[im_w], im_U[im_w],im_V[im_w];
        int   index_f[im_w], n, m, initial =0 ;
	int   Th_lr, N_1, pixel_sum;
	int   index_sum = 0, index_mean;
	float yaw_ref;
        int   flag_U[im_w] = {0};
        int   flag_V[im_w] = {0};
        
        for (j=0; j<im_w; j++)
        {
        
                //im_U[j] = *(img_data + row_chosen*im_w*2 + j*2 + 0);
                //im_Y[j] = *(img_data + row_chosen*im_w*2 + j*2 + 1);
                //im_V[j] = *(img_data + row_chosen*im_w*2 + j*2 + 2);
                
                //im_U[j] = *(img_data + row_chosen*im_w*2 + 0);
                //im_Y[j] = *(img_data + row_chosen*im_w*2 + 1);
                //im_V[j] = *(img_data + row_chosen*im_w*2 + 2);
                
                // no downsize
                //im_U[j] = img_data_s.buf[row_chosen*im_w*2*2 + j*2+ 0];
                //im_Y[j] = img_data_s.buf[row_chosen*im_w*2*2 + j*2+ 1];
                //im_V[j] = img_data_s.buf[row_chosen*im_w*2*2 + j*2+ 2];
                
                //if (downsize_factor == 2)
                //{
                        im_U[j] = img_data_s.buf[row_chosen*im_w*2*2 + j*4+ 0];
                        im_Y[j] = img_data_s.buf[row_chosen*im_w*2*2 + j*4+ 1];
                        im_V[j] = img_data_s.buf[row_chosen*im_w*2*2 + j*4+ 2];
               // }
                
                // normalize                             
                //im_Y[j] = im_Y[j]/255;
                //im_U[j] = im_U[j]/255;
                //im_V[j] = im_V[j]/255;                            
                // check the upper and lower thresholds for U
                flag_U[j] = 0;
                if ( im_U[j] > TH_U_U*255 || im_U[j] < TH_U_L*255)
                {
                        flag_U[j] = 1;        
                }
                // check the upper and lower thresholds for V
                flag_V[j] = 0;
                if (im_V[j] > TH_V_U*255 || im_V[j] < TH_V_L*255)// || flag_U[j] == 1 )
                {
                        flag_V[j] = 1;        
                }
	}
   
        
        // left and right width thresholds
        Th_lr = (threshold-1)/2;
        
        // find the first pixel whose left width is bigger than Th/2
        for (i=0; i<im_w; i++)
        {
                if (flag_V[i]>0)
                {
                        N_1 = i;
                        printf("N_1 = %d\n",i);
                        break;
                }
        }
        
        // compare with the left threshold
        if (N_1 < Th_lr)
        {     N_1 = Th_lr;
        }
        
        // start from this point may save some time
        for (n=N_1; n<im_w-threshold; n++)
	{
		pixel_sum = 0;
			for (m=-Th_lr; m<Th_lr+1; m++)
				pixel_sum = pixel_sum + flag_V[n+m];
				
			if (pixel_sum == threshold)
			{
				index_f[initial] = n;
				initial = initial + 1;
				//printf("n = %i, initial = %i\n", n, initial);
			}
	}
        
        // perform actions 
        if (initial >=1)
	{
		for (p=0; p<initial; p++)
			index_sum = index_sum + index_f[p];
		index_mean = index_sum / initial;
		if (index_mean > im_w/2)
			yaw_ref = -45;
		else
			yaw_ref = 45;
	}
	else 
		yaw_ref = 0;

	printf("Yaw at an angle of %f degree\n", yaw_ref);
	
	*/
	
	
	// Assume the colorspace for YUV is the YCbCr
// Note we downsample the image by a factor 2.

//Changed by YIJ
        #define im_width 640
	int j, obs_index[640] = {0};
	float y[640], u[640], v[640], Y[640], Cb[640], Cr[640];

	for (j=0; j<640; j++)
	{
		u[j] = img_data_s.buf[1280*500*2 + j*4];
		y[j] = img_data_s.buf[1280*500*2 + j*4 + 1];
		v[j] = img_data_s.buf[1280*500*2 + j*4 + 2];
		
		Y[j] = y[j];
	        Cb[j] = u[j];
	        Cr[j] = v[j];
	
	// Using some appropriate thresholds, make two binary maps of Cb and Cr, 
	// and then add them together.

		obs_index[j] = 0;
		//if (Y[j] > 0.8*255 || Y[j] < 0.1*255)
		//	obs_index[j] = obs_index[j] + 1;
		//if (Cb[j]/255 > 0.7 || Cb[j]/255 < 0.3)
		//	obs_index[j] = obs_index[j] + 1;
		if (Cr[j] > 0.5*255 || Cr[j] < 0.3*255)
			obs_index[j] = obs_index[j] + 1;
 
	}
	
	// Identify if there are any poles with the width larger than 2*threshold+1 pixels,
	// and if so, remember its position.

	int index[im_width] = {0}, n, m, count = 0;
	float threshold = 120, pixel_sum;
	for (n = threshold; n < im_width-threshold; n++)
	{
		pixel_sum = 0;
			for (m = -threshold; m < threshold + 1; m++)
				pixel_sum = pixel_sum + obs_index[n+m];
			
			if (pixel_sum >= 2*threshold+1)
			{
				index[count] = n;
				count = count + 1;
			}
	}
	
	// Based on the (average) pole location, determine the target yaw angle.
	// If pole is on the right side turn left by setting negative yaw angle.
	// If it's on the left side, do the opposite.

	int p;
	float index_sum = 0, index_mean;
	if (count >= 1)
	{
		for (p = 0; p < count; p++)
			index_sum = index_sum + index[p];
		index_mean = index_sum / count;
		if (index_mean > 320)
			yaw_ref = -45;
		else
			yaw_ref = 45;
	}
	else
		yaw_ref = 0;
	
	
	
	//vision_results.yaw_ref = img_data[1];
	vision_results.yaw_ref = yaw_ref;
	//yaw_ref_get = yaw_ref;
	
	
	
//----------------------------- changed by Peng LU-------------------------------

    //printf("Vision result %f %f\n", vision_results.Velx, vision_results.Vely);

    /* Send results to main */
    vision_results.cnt++;
    int bytes_written = write(thread_socket, &vision_results, sizeof(vision_results));
    if (bytes_written != sizeof(vision_results)){
      perror("[thread] Failed to write to socket.\n");
    }
    DEBUG_INFO("[thread] Write # %d, (bytes %d)\n",vision_results.cnt, bytes_written);

#ifdef DOWNLINK_VIDEO
    // JPEG encode the image:
    uint32_t quality_factor = 10; //20 if no resize,
    uint8_t dri_header = 0;
    uint32_t image_format = FOUR_TWO_TWO;  // format (in jpeg.h)
    uint8_t *end = encode_image(img->buf, jpegbuf, quality_factor, image_format, dev->w, dev->h, dri_header);
    uint32_t size = end - (jpegbuf);

    //printf("Sending an image ...%u\n", size);
    send_rtp_frame(vsock, jpegbuf, size, dev->w, dev->h, 0, quality_factor, dri_header, 0);
#endif

    // Free the image
    v4l2_image_free(dev, img);
  }

  printf("Thread Closed\n");
  v4l2_close(dev);
  return 0;
}




//----------------------------- changed by Peng LU-------------------------------
float yaw_ref_fun()
{
  return yaw_ref;
}
//----------------------------- changed by Peng LU-------------------------------


