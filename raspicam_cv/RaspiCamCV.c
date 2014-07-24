/////////////////////////////////////////////////////////////
//
// Many source code lines are copied from RaspiVid.c
// Copyright (c) 2012, Broadcom Europe Ltd
// 
// Lines have been added by Pierre Raufast - mai 2013
// pierre.raufast@gmail.com
// to work with OpenCV 2.3
// visit thinkrpi.wordpress.com
// Enjoy !
// This file display camera in a OpenCv window
//
// For a better world, read Giono's Books
// 
/////////////////////////////////////////////////////////////
//
// Emil Valkov - robidouille@valkov.com
//
// Converted to a library, which exposes an interface
// similar to what OpenCV provides, but uses the RaspiCam
// underneath - September 22 2013.
//
// cvCreateCameraCapture -> raspiCamCvCreateCameraCapture
// cvReleaseCapture -> raspiCamCvReleaseCapture
// cvSetCaptureProperty -> raspiCamCvSetCaptureProperty
// cvQueryFrame -> raspiCamCvQueryFrame
//
/////////////////////////////////////////////////////////////

#include "RaspiCamCV.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>

//new
#include <cv.h>
#include <highgui.h>
#include "time.h"

#include "bcm_host.h"
VCOS_LOG_CAT_T raspitex_log_category = VCOS_LOG_INIT("RaspiCV", VCOS_LOG_TRACE);
#define VCOS_LOG_CATEGORY (&raspitex_log_category)
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"

#include "RaspiCamControl.h"

#include <semaphore.h>

#include <stdio.h>

/// Camera number to use - we only have one camera, indexed from 0.
#define CAMERA_NUMBER 0

// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
//#define MMAL_CAMERA_CAPTURE_PORT 2

// Video format information
#define VIDEO_FRAME_RATE_NUM 30
#define VIDEO_FRAME_RATE_DEN 1

/// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 6

// Max bitrate we allow for recording
//const int MAX_BITRATE = 30000000; // 30 Mbits/s
const int MAX_BITRATE = 25000000; // 25 Mbits/s

int mmal_status_to_int(MMAL_STATUS_T status);

typedef struct
{
	FILE *file_handle;						 /// File handle to write buffer data to.
	RASPIVID_STATE *pstate;				  /// pointer to our state in case required in callback
	int abort;									/// Set to 1 in callback if an error occurs to attempt to abort the capture
	char *cb_buff;							  /// Circular buffer
	int	cb_len;								/// Length of buffer
	int	cb_wptr;							  /// Current write pointer
	int	cb_wrap;							  /// Has buffer wrapped at least once?
	int	cb_data;							  /// Valid bytes in buffer
#define IFRAME_BUFSIZE (60*1000)
	int	iframe_buff[IFRAME_BUFSIZE];			 /// buffer of iframe pointers
	int	iframe_buff_wpos;
	int	iframe_buff_rpos;
	char  header_bytes[29];
	int  header_wptr;
	FILE *imv_file_handle;					/// File handle to write inline motion vectors to.
} PORT_USERDATA;

/** Structure containing all state information for the current run
 */
typedef struct _RASPIVID_STATE
{
	int finished;
	int width;								  /// Requested width of image
	int height;								 /// requested height of image
	int bitrate;								/// Requested bitrate
	int framerate;							 /// Requested frame rate (fps)
	int intraperiod;
	int quantisationParameter;
	int graymode;			/// capture in gray only (2x faster)
	int immutableInput;	  /// Flag to specify whether encoder works in place or creates a new buffer. Result is preview can display either
						 /// the camera output or the encoder output (with compression artifacts)
	int profile;
	int bInlineHeaders;
	int inlineMotionVectors;

	int segmentSize;						  /// Segment mode In timed cycle mode, the amount of time the capture is off per cycle
	int segmentWrap;						  /// Point at which to wrap segment counter
	int segmentNumber;						/// Current segment counter
	int splitNow;							  /// Split at next possible i-frame if set to 1.
	int splitWait;							 /// Switch if user wants splited files 
	
	RASPICAM_CAMERA_PARAMETERS camera_parameters; /// Camera setup parameters

	MMAL_COMPONENT_T *camera_component;	 /// Pointer to the camera component
	MMAL_COMPONENT_T *encoder_component;	/// Pointer to the encoder component
	MMAL_CONNECTION_T *preview_connection; /// Pointer to the connection from camera to preview
	MMAL_CONNECTION_T *encoder_connection; /// Pointer to the connection from camera to encoder

	MMAL_POOL_T *preview_pool; /// Pointer to the pool of buffers used by encoder output port
	MMAL_POOL_T *video_pool; /// Pointer to the pool of buffers used by encoder output port
	MMAL_POOL_T *encoder_pool; /// Pointer to the pool of buffers used by encoder output port

	PORT_USERDATA callback_data;		  /// Used to move data to the encoder callback

	IplImage *py, *pu, *pv;
	IplImage *pu_big, *pv_big, *yuvImage,* dstImage;

	VCOS_SEMAPHORE_T capture_sem;
	VCOS_SEMAPHORE_T capture_done_sem;
	
	char *imv_filename;						/// filename of inline Motion Vectors output
	char *filename;							/// filename of output file

} RASPIVID_STATE;

// default status
static void default_status(RASPIVID_STATE *state)
{
	if (!state)
	{
		vcos_assert(0);
		return;
	}

	// Default everything to zero
	memset(state, 0, sizeof(RASPIVID_STATE));

	// Now set anything non-zero
	state->finished = 0;
	state->width = 640;		// use a multiple of 320 (640, 1280)
	state->height = 480;		// use a multiple of 240 (480, 960)
//	state->width = 1280;		// use a multiple of 320 (640, 1280)
//	state->height = 960;		// use a multiple of 240 (480, 960)
	state->bitrate = 10000000; // This is a decent default bitrate for 1080p
	state->framerate = VIDEO_FRAME_RATE_NUM;
	state->intraperiod = 0;	 // Not set
	state->quantisationParameter = 0;
	state->immutableInput = 1;
	state->graymode = 0;		// Gray (1) much faster than color (0)
	state->profile = MMAL_VIDEO_PROFILE_H264_HIGH;
	state->bInlineHeaders = 0;
	state->inlineMotionVectors = 0;
	
	state->segmentSize = 0;  // 0 = not segmenting the file.
	state->segmentNumber = 1;
	state->segmentWrap = 0; // Point at which to wrap segment number back to 1. 0 = no wrap
	state->splitNow = 0;
	state->splitWait = 0;
	
	state->filename = "test.mp4";
	
	// Set up the camera_parameters to default
	raspicamcontrol_set_defaults(&state->camera_parameters);
}


/**
 * Open a file based on the settings in state
 *
 * @param state Pointer to state
 */
static FILE *open_filename(RASPIVID_STATE *pState)
{
	FILE *new_handle = NULL;
	char *tempname = NULL, *filename = NULL;

	if (pState->segmentSize || pState->splitWait)
	{
		// Create a new filename string
		asprintf(&tempname, pState->filename, pState->segmentNumber);
		filename = tempname;
	}
	else
	{
		filename = pState->filename;
	}

	if (filename)
	{
		new_handle = fopen(filename, "wb");
		vcos_log_error("opened file %s, returned fd %d", filename, new_handle);
	}
	else
		vcos_log_error("no filename");

	if (tempname)
		free(tempname);

	return new_handle;
}

/**
 * Open a file based on the settings in state
 *
 * This time for the imv output file
 *
 * @param state Pointer to state
 */
static FILE *open_imv_filename(RASPIVID_STATE *pState)
{
	FILE *new_handle = NULL;
	char *tempname = NULL, *filename = NULL;

	if (pState->segmentSize || pState->splitWait)
	{
		// Create a new filename string
		asprintf(&tempname, pState->imv_filename, pState->segmentNumber);
		filename = tempname;
	}
	else
	{
		filename = pState->imv_filename;
	}

	if (filename)
		new_handle = fopen(filename, "wb");

	if (tempname)
		free(tempname);

	return new_handle;
}

/**
 *  buffer header callback function for video
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
 
 int counter = 0;
 
static void preview_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	MMAL_BUFFER_HEADER_T *new_buffer;
	RASPIVID_STATE * state = (RASPIVID_STATE *)port->userdata;

	counter++;

	if (state)
	{
vcos_log_error("%s: Buffer rxd. Len %d, alloc_size %d, flags %d", __func__, buffer->length, buffer->alloc_size, buffer->flags);
		if (state->finished) {
			vcos_semaphore_post(&state->capture_done_sem);
			return;
		}
		if (buffer->length)
		{
			mmal_buffer_header_mem_lock(buffer);
 
			//
			// *** PR : OPEN CV Stuff here !
			//
			int w=state->width;	// get image size
			int h=state->height;
			int h4=h/4;

			memcpy(state->py->imageData,buffer->data,w*h);	// read Y

			/*if (state->graymode==0)
			{
				memcpy(state->pu->imageData,buffer->data+w*h,w*h4); // read U
				memcpy(state->pv->imageData,buffer->data+w*h+w*h4,w*h4); // read v
			}*/

			if (state->graymode==0)
			{
				memcpy(state->pu->imageData,buffer->data+w*h,w*h4); // read U
				memcpy(state->pv->imageData,buffer->data+w*h+w*h4,w*h4); // read v
			}

			vcos_semaphore_post(&state->capture_done_sem);
			vcos_semaphore_wait(&state->capture_sem);

			mmal_buffer_header_mem_unlock(buffer);
		}
		else
		{
			vcos_log_error("buffer null");
		}
	}
	else
	{
		vcos_log_error("Received a encoder buffer callback with no state");
	}

	// release buffer back to the pool
	mmal_buffer_header_release(buffer);

	// and send one back to the port (if still open)
	if (port->is_enabled)
	{
		MMAL_STATUS_T status;

		new_buffer = mmal_queue_get(state->preview_pool->queue);

		if (new_buffer)
			status = mmal_port_send_buffer(port, new_buffer);

		if (!new_buffer || status != MMAL_SUCCESS)
			vcos_log_error("Unable to return a buffer to the encoder port");
	}
}


/**
 * Create the camera component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return 0 if failed, pointer to component if successful
 *
 */
static MMAL_COMPONENT_T *create_camera_component(RASPIVID_STATE *state)
{
	MMAL_COMPONENT_T *camera = 0;
	MMAL_ES_FORMAT_T *format;
	MMAL_PORT_T *preview_port = NULL;
	MMAL_PORT_T *video_port = NULL;
	//MMAL_PORT_T *still_port = NULL;
	MMAL_STATUS_T status;

	/* Create the component */
	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);

	//printf("point1.0\n");
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to create camera component");
		goto error;
	}

	if (!camera->output_num)
	{
		vcos_log_error("Camera doesn't have output ports");
		goto error;
	}

	status = mmal_component_enable(camera);
	if (status)
	{
		vcos_log_error("error enabling camera component - status %d", status);
		goto error;
	}

	preview_port = camera->output[MMAL_CAMERA_PREVIEW_PORT];
	video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
	//still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

	//  set up the camera configuration
	{
		MMAL_PARAMETER_CAMERA_CONFIG_T cam_config =
		{
			{ MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
			.max_stills_w = state->width,
			.max_stills_h = state->height,
			.stills_yuv422 = 0,
			.one_shot_stills = 0,
			.max_preview_video_w = state->width,
			.max_preview_video_h = state->height,
			.num_preview_video_frames = 3,
			.stills_capture_circular_buffer_height = 0,
			.fast_preview_resume = 0,
			.use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
		};
		mmal_port_parameter_set(camera->control, &cam_config.hdr);
	}
	// Set the encode format on the video  port

	//printf("point1.1\n");
	format = preview_port->format;
	format->encoding_variant = MMAL_ENCODING_I420;
	format->encoding = MMAL_ENCODING_I420;
	format->es->video.width = state->width;
	format->es->video.height = state->height;
	format->es->video.crop.x = 0;
	format->es->video.crop.y = 0;
	format->es->video.crop.width = state->width;
	format->es->video.crop.height = state->height;
	format->es->video.frame_rate.num = state->framerate;
	format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

	status = mmal_port_format_commit(preview_port);
	if (status)
	{
		vcos_log_error("camera video format couldn't be set");
		goto error;
	}

	//printf("point1.11\n");
	format = video_port->format;
	format->encoding_variant = MMAL_ENCODING_I420;
	format->encoding = MMAL_ENCODING_OPAQUE;
	format->es->video.width = state->width;
	format->es->video.height = state->height;
	format->es->video.crop.x = 0;
	format->es->video.crop.y = 0;
	format->es->video.crop.width = state->width;
	format->es->video.crop.height = state->height;
	format->es->video.frame_rate.num = state->framerate;
	format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

	status = mmal_port_format_commit(video_port);
	if (status)
	{
		vcos_log_error("camera video format couldn't be set");
		goto error;
	}
	//mmal_port_format_copy(video_port, video_port2);
	//printf("point1.2\n");

	preview_port->buffer_size = preview_port->buffer_size_recommended;
	preview_port->buffer_num = preview_port->buffer_num_recommended;
	// Ensure there are enough buffers to avoid dropping frames
	if (preview_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
		preview_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

	// Ensure there are enough buffers to avoid dropping frames
	if (video_port->buffer_num < video_port->buffer_num_recommended)
		video_port->buffer_num = video_port->buffer_num_recommended;

	// PR : plug the callback to the video port 
	status = mmal_port_enable(preview_port, preview_buffer_callback);
	if (status)
	{
		vcos_log_error("camera video callback2 error");
		goto error;
	}

	//printf("point1.3\n");
	/*// Set the encode format on the still  port
	format = still_port->format;
	format->encoding = MMAL_ENCODING_OPAQUE;
	format->encoding_variant = MMAL_ENCODING_I420;
	format->es->video.width = state->width;
	format->es->video.height = state->height;
	format->es->video.crop.x = 0;
	format->es->video.crop.y = 0;
	format->es->video.crop.width = state->width;
	format->es->video.crop.height = state->height;
	format->es->video.frame_rate.num = 1;
	format->es->video.frame_rate.den = 1;

	status = mmal_port_format_commit(still_port);
	if (status)
	{
		vcos_log_error("camera still format couldn't be set");
		goto error;
	}*/


	//PR : create pool of message on video port
	MMAL_POOL_T *pool;
	pool = mmal_port_pool_create(preview_port, preview_port->buffer_num, preview_port->buffer_size);
	if (!pool)
	{
		vcos_log_error("Failed to create buffer header pool for preview output port");
	}
	//printf("point1.4\n");
	state->preview_pool = pool;
	
	
	//PR : create pool of message on video port
	MMAL_POOL_T *pool2;
	video_port->buffer_size = video_port->buffer_size_recommended;
	video_port->buffer_num = video_port->buffer_num_recommended;
	//printf("point1.4\n");
	state->video_pool = pool2;

	/*// Ensure there are enough buffers to avoid dropping frames
	if (still_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
		still_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;*/

	/* Enable component */
	//printf("point1.5\n");
	//printf("point1.6\n");

	if (status)
	{
		vcos_log_error("camera component couldn't be enabled");
		goto error;
	}

	//printf("point1.7\n");
	raspicamcontrol_set_all_parameters(camera, &state->camera_parameters);

	//printf("point1.8\n");
	state->camera_component = camera;

	//printf("point1.9\n");
	return camera;

error:

	if (camera)
		mmal_component_destroy(camera);

	return 0;
}

/**
 * Destroy the camera component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_camera_component(RASPIVID_STATE *state)
{
	if (state->camera_component)
	{
		mmal_component_destroy(state->camera_component);
		state->camera_component = NULL;
	}
}

/**
 *  buffer header callback function for encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void encoder_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	vcos_log_error("%s: buffer length %d, alloc_size %d, flags %04X", __func__, buffer->length, buffer->alloc_size, buffer->flags);
	MMAL_BUFFER_HEADER_T *new_buffer;
	static int64_t base_time =  -1;

	// All our segment times based on the receipt of the first encoder callback
	if (base_time == -1)
		base_time = vcos_getmicrosecs64()/1000;

	// We pass our file handle and other stuff in via the userdata field.

	PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;

	if (pData)
	{
		int bytes_written = buffer->length;
		int64_t current_time = vcos_getmicrosecs64()/1000;

		vcos_assert(pData->file_handle);
		if(pData->pstate->inlineMotionVectors) vcos_assert(pData->imv_file_handle);
		
		//else 
		//{
		// For segmented record mode, we need to see if we have exceeded our time/size,
		// but also since we have inline headers turned on we need to break when we get one to
		// ensure that the new stream has the header in it. If we break on an I-frame, the
		// SPS/PPS header is actually in the previous chunk.
		/*if ((buffer->flags & MMAL_BUFFER_HEADER_FLAG_CONFIG) &&
			 ((pData->pstate->segmentSize && current_time > base_time + pData->pstate->segmentSize) ||
			  (pData->pstate->splitWait && pData->pstate->splitNow)))
		{
			FILE *new_handle;

			base_time = current_time;

			pData->pstate->splitNow = 0;
			pData->pstate->segmentNumber++;

			// Only wrap if we have a wrap point set
			if (pData->pstate->segmentWrap && pData->pstate->segmentNumber > pData->pstate->segmentWrap)
				pData->pstate->segmentNumber = 1;

			new_handle = open_filename(pData->pstate);

			if (new_handle)
			{
				fclose(pData->file_handle);
				pData->file_handle = new_handle;
			}

			new_handle = open_imv_filename(pData->pstate);

			if (new_handle)
			{
				fclose(pData->imv_file_handle);
				pData->imv_file_handle = new_handle;
			}
		}*/
		if (buffer->length)
		{
			printf("writing...");
			mmal_buffer_header_mem_lock(buffer);
			if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO)
			{
				if(pData->pstate->inlineMotionVectors)
				{
					bytes_written = fwrite(buffer->data, 1, buffer->length, pData->imv_file_handle);
				}
				else
				{
					//We do not want to save inlineMotionVectors...
					bytes_written = buffer->length;
				}
			}
			else
			{
				bytes_written = fwrite(buffer->data, 1, buffer->length, pData->file_handle);				
			}

			mmal_buffer_header_mem_unlock(buffer);

			if (bytes_written != buffer->length)
			{
				vcos_log_error("Failed to write buffer data (%d from %d)- aborting", bytes_written, buffer->length);
				pData->abort = 1;
			}
		}
		//}
	}
	else
	{
		vcos_log_error("Received a encoder buffer callback with no state");
	}

	// release buffer back to the pool
	mmal_buffer_header_release(buffer);

	// and send one back to the port (if still open)
	if (port->is_enabled)
	{
		MMAL_STATUS_T status;

		new_buffer = mmal_queue_get(pData->pstate->encoder_pool->queue);

		if (new_buffer)
			status = mmal_port_send_buffer(port, new_buffer);

		if (!new_buffer || status != MMAL_SUCCESS)
			vcos_log_error("Unable to return a buffer to the encoder port");
	}
	printf("done.\n");
}

/**
 * Create the encoder component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return 0 if failed, pointer to component if successful
 *
 */
static MMAL_STATUS_T create_encoder_component(RASPIVID_STATE *state)
{
	MMAL_COMPONENT_T *encoder = 0;
	MMAL_PORT_T *encoder_input = NULL, *encoder_output = NULL;
	MMAL_STATUS_T status;
	MMAL_POOL_T *pool;
	//printf("point1\n");

	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_ENCODER, &encoder);

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Unable to create video encoder component");
		goto error;
	}

	if (!encoder->input_num || !encoder->output_num)
	{
		status = MMAL_ENOSYS;
		vcos_log_error("Video encoder doesn't have input/output ports");
		goto error;
	}
	encoder_input = encoder->input[0];
	encoder_output = encoder->output[0];

	// We want same format on input and output
	mmal_format_copy(encoder_output->format, encoder_input->format);

	// Only supporting H264 at the moment
	encoder_output->format->encoding = MMAL_ENCODING_H264;

	encoder_output->format->bitrate = state->bitrate;

	encoder_output->buffer_size = encoder_output->buffer_size_recommended;

	if (encoder_output->buffer_size < encoder_output->buffer_size_min)
		encoder_output->buffer_size = encoder_output->buffer_size_min;

	encoder_output->buffer_num = encoder_output->buffer_num_recommended;

	if (encoder_output->buffer_num < encoder_output->buffer_num_min)
		encoder_output->buffer_num = encoder_output->buffer_num_min;

	// We need to set the frame rate on output to 0, to ensure it gets
	// updated correctly from the input framerate when port connected
	encoder_output->format->es->video.frame_rate.num = 0;
	encoder_output->format->es->video.frame_rate.den = 1;

	// Commit the port changes to the output port
	status = mmal_port_format_commit(encoder_output);
	//printf("point2\n");

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Unable to set format on video encoder output port");
		goto error;
	}

	// Set the rate control parameter
	if (0)
	{
		MMAL_PARAMETER_VIDEO_RATECONTROL_T param = {{ MMAL_PARAMETER_RATECONTROL, sizeof(param)}, MMAL_VIDEO_RATECONTROL_DEFAULT};
		status = mmal_port_parameter_set(encoder_output, &param.hdr);
		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("Unable to set ratecontrol");
			goto error;
		}

	}

	if (state->intraperiod)
	{
		MMAL_PARAMETER_UINT32_T param = {{ MMAL_PARAMETER_INTRAPERIOD, sizeof(param)}, state->intraperiod};
		status = mmal_port_parameter_set(encoder_output, &param.hdr);
		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("Unable to set intraperiod");
			goto error;
		}
	}

	if (state->quantisationParameter)
	{
		MMAL_PARAMETER_UINT32_T param = {{ MMAL_PARAMETER_VIDEO_ENCODE_INITIAL_QUANT, sizeof(param)}, state->quantisationParameter};
		status = mmal_port_parameter_set(encoder_output, &param.hdr);
		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("Unable to set initial QP");
			goto error;
		}

		MMAL_PARAMETER_UINT32_T param2 = {{ MMAL_PARAMETER_VIDEO_ENCODE_MIN_QUANT, sizeof(param)}, state->quantisationParameter};
		status = mmal_port_parameter_set(encoder_output, &param2.hdr);
		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("Unable to set min QP");
			goto error;
		}

		MMAL_PARAMETER_UINT32_T param3 = {{ MMAL_PARAMETER_VIDEO_ENCODE_MAX_QUANT, sizeof(param)}, state->quantisationParameter};
		status = mmal_port_parameter_set(encoder_output, &param3.hdr);
		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("Unable to set max QP");
			goto error;
		}

	}

	{
		MMAL_PARAMETER_VIDEO_PROFILE_T  param;
		param.hdr.id = MMAL_PARAMETER_PROFILE;
		param.hdr.size = sizeof(param);

		//param.profile[0].profile = state->profile;
		param.profile[0].profile = MMAL_VIDEO_PROFILE_H264_HIGH;
		param.profile[0].level = MMAL_VIDEO_LEVEL_H264_4; // This is the only value supported

		status = mmal_port_parameter_set(encoder_output, &param.hdr);
		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("Unable to set H264 profile");
			goto error;
		}
	}

	if (mmal_port_parameter_set_boolean(encoder_input, MMAL_PARAMETER_VIDEO_IMMUTABLE_INPUT, state->immutableInput) != MMAL_SUCCESS)
	{
		vcos_log_error("Unable to set immutable input flag");
		// Continue rather than abort..
	}

	//set INLINE HEADER flag to generate SPS and PPS for every IDR if requested
	if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_HEADER, state->bInlineHeaders) != MMAL_SUCCESS)
	{
		vcos_log_error("failed to set INLINE HEADER FLAG parameters");
		// Continue rather than abort..
	}
	
	//set INLINE VECTORS flag to request motion vector estimates
	if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_VECTORS, state->inlineMotionVectors) != MMAL_SUCCESS)
	{
		vcos_log_error("failed to set INLINE VECTORS parameters");
		// Continue rather than abort..
	}

	//  Enable component
	status = mmal_component_enable(encoder);

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Unable to enable video encoder component");
		goto error;
	}

	/* Create pool of buffer headers for the output port to consume */
	pool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);

	if (!pool)
	{
		vcos_log_error("Failed to create buffer header pool for encoder output port %s", encoder_output->name);
		goto error;
	}

	state->encoder_pool = pool;
	state->encoder_component = encoder;

	/*if (state->verbose)
		fprintf(stderr, "Encoder component done\n");
*/
	return status;

	error:
	if (encoder)
		mmal_component_destroy(encoder);

	state->encoder_component = NULL;

	return status;
}

/**
 * Destroy the encoder component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_encoder_component(RASPIVID_STATE *state)
{
	// Get rid of any port buffers first
	if (state->preview_pool)
	{
		mmal_port_pool_destroy(state->encoder_component->output[0], state->preview_pool);
	}

	if (state->encoder_component)
	{
		mmal_component_destroy(state->encoder_component);
		state->encoder_component = NULL;
	}
}

/**
 * Connect two specific ports together
 *
 * @param output_port Pointer the output port
 * @param input_port Pointer the input port
 * @param Pointer to a mmal connection pointer, reassigned if function successful
 * @return Returns a MMAL_STATUS_T giving result of operation
 *
 */
static MMAL_STATUS_T connect_ports(MMAL_PORT_T *output_port, MMAL_PORT_T *input_port, MMAL_CONNECTION_T **connection)
{
	MMAL_STATUS_T status;

	status =  mmal_connection_create(connection, output_port, input_port, MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);

	if (status == MMAL_SUCCESS)
	{
		status =  mmal_connection_enable(*connection);
		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("Failed to connect ports %s and %s - status %d", output_port->name, input_port->name);
			mmal_connection_destroy(*connection);
		}
	}

	return status;
}

/**
 * Checks if specified port is valid and enabled, then disables it
 *
 * @param port  Pointer the port
 *
 */
static void check_disable_port(MMAL_PORT_T *port)
{
	if (port && port->is_enabled)
		mmal_port_disable(port);
}

RaspiCamCvCapture * raspiCamCvCreateCameraCapture(int index)
{
	RaspiCamCvCapture * capture = (RaspiCamCvCapture*)malloc(sizeof(RaspiCamCvCapture));
	// Our main data storage vessel..
	RASPIVID_STATE * state = (RASPIVID_STATE*)malloc(sizeof(RASPIVID_STATE));
	capture->pState = state;

	MMAL_STATUS_T status = -1;
	MMAL_PORT_T *camera_preview_port = NULL;
	MMAL_PORT_T *camera_video_port = NULL;
	//MMAL_PORT_T *camera_still_port = NULL;
	MMAL_PORT_T *encoder_input_port = NULL;
	MMAL_PORT_T *encoder_output_port = NULL;

	bcm_host_init();

	// read default status
	default_status(state);

	int w = state->width;
	int h = state->height;
	state->py = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 1);		// Y component of YUV I420 frame
	if (state->graymode==0) {
		state->pu = cvCreateImage(cvSize(w/2,h/2), IPL_DEPTH_8U, 1);	// U component of YUV I420 frame
		state->pv = cvCreateImage(cvSize(w/2,h/2), IPL_DEPTH_8U, 1);	// V component of YUV I420 frame
	}
	vcos_semaphore_create(&state->capture_sem, "Capture-Sem", 0);
	vcos_semaphore_create(&state->capture_done_sem, "Capture-Done-Sem", 0);

	if (state->graymode==0) {
		state->pu_big = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 1);
		state->pv_big = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 1);
		state->yuvImage = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 3);
		state->dstImage = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 3); // final picture to display
	}

	//printf("point2.0\n");
	// create camera
	if (!create_camera_component(state))
	{
		vcos_log_error("%s: Failed to create camera component", __func__);
		raspiCamCvReleaseCapture(&capture);
		return NULL;
	} else if ((status = create_encoder_component(state)) != MMAL_SUCCESS)
	{
		vcos_log_error("%s: Failed to create encode component", __func__);
		destroy_camera_component(state);
		return NULL;
	}

	//printf("point2.1\n");
	//create_encoder_component(state);

	camera_preview_port = state->camera_component->output[MMAL_CAMERA_PREVIEW_PORT];
	camera_video_port = state->camera_component->output[MMAL_CAMERA_VIDEO_PORT];
	//camera_still_port = state->camera_component->output[MMAL_CAMERA_CAPTURE_PORT];
	encoder_input_port  = state->encoder_component->input[0];
	encoder_output_port = state->encoder_component->output[0];

	// assign data to use for callback
	camera_preview_port->userdata = (struct MMAL_PORT_USERDATA_T *)state;

	// assign data to use for callback
	camera_video_port->userdata = (struct MMAL_PORT_USERDATA_T *)state;
vcos_log_error("Connecting ports now");
	// Now connect the camera to the encoder
	status = connect_ports(camera_video_port, encoder_input_port, &state->encoder_connection);	
	
	if (state->filename)
	{
		if (state->filename[0] == '-')
		{
			state->callback_data.file_handle = stdout;
		}
		else
		{
			state->callback_data.file_handle = open_filename(state);
		}

		if (!state->callback_data.file_handle)
		{
			// Notify user, carry on but discarding encoded output buffers
			vcos_log_error("%s: Error opening output file: %s\nNo output file will be generated\n", __func__, state->filename);
		}
	}

	// Set up our userdata - this is passed though to the callback where we need the information.
	state->callback_data.pstate = state;
	state->callback_data.abort = 0;
	encoder_output_port->userdata = (struct MMAL_PORT_USERDATA_T *)&state->callback_data;

	// Enable the encoder output port and tell it its callback function
	status = mmal_port_enable(encoder_output_port, encoder_buffer_callback);

	if (status != MMAL_SUCCESS)
	{
		 state->encoder_connection = NULL;
		 vcos_log_error("%s: Failed to connect camera video port to encoder input", __func__);
		 return NULL;
	}

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to setup encoder output");
		return NULL;
	}
	
	//mmal_port_enable(encoder_output_port, encoder_buffer_callback);
	
	// Only encode stuff if we have a filename and it opened
	// Note we use the copy in the callback, as the call back MIGHT change the file handle
	if (state->callback_data.file_handle)
	{
		int running = 1;

		// Send all the buffers to the encoder output port
		{
			int num = mmal_queue_length(state->encoder_pool->queue);
			int q;
			for (q=0;q<num;q++)
			{
				MMAL_BUFFER_HEADER_T *buffer2 = mmal_queue_get(state->encoder_pool->queue);

				if (!buffer2)
					vcos_log_error("Unable to get a required buffer %d from pool queue", q);

				if (mmal_port_send_buffer(encoder_output_port, buffer2)!= MMAL_SUCCESS)
					vcos_log_error("Unable to send a buffer to encoder output port (%d)", q);
			}
		}
	}


	// start capture
	if (mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS)
	{
		vcos_log_error("%s: Failed to start capture", __func__);
		raspiCamCvReleaseCapture(&capture);
		return NULL;
	}

	// Send all the buffers to the preview port

	int num = mmal_queue_length(state->preview_pool->queue);
	int q;
	for (q = 0; q < num; q++)
	{
		MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state->preview_pool->queue);

		if (!buffer)
			vcos_log_error("Unable to get a required buffer %d from pool queue", q);

		if (mmal_port_send_buffer(camera_preview_port, buffer)!= MMAL_SUCCESS)
			vcos_log_error("Unable to send a buffer to encoder output port (%d)", q);
	}

	//mmal_status_to_int(status);	

	// Disable all our ports that are not handled by connections
	//check_disable_port(camera_still_port);

	//if (status != 0)
	//	raspicamcontrol_check_configuration(128);

	vcos_semaphore_wait(&state->capture_done_sem);
	return capture;
}

void raspiCamCvReleaseCapture(RaspiCamCvCapture ** capture)
{
	RASPIVID_STATE * state = (*capture)->pState;

	// Unblock the the callback.
	state->finished = 1;
	vcos_semaphore_post(&state->capture_sem);
	vcos_semaphore_wait(&state->capture_done_sem);

	vcos_semaphore_delete(&state->capture_sem);
	vcos_semaphore_delete(&state->capture_done_sem);

	if (state->camera_component)
		mmal_component_disable(state->camera_component);

	destroy_camera_component(state);

	cvReleaseImage(&state->pu);
	if (state->graymode==0) {
		cvReleaseImage(&state->pv);
		cvReleaseImage(&state->py);
	}

	if (state->graymode==0) {
		cvReleaseImage(&state->pu_big);
		cvReleaseImage(&state->pv_big);
		cvReleaseImage(&state->yuvImage);
		cvReleaseImage(&state->dstImage);
	}

	free(state);
	free(*capture);
	*capture = 0;
}

void raspiCamCvSetCaptureProperty(RaspiCamCvCapture * capture, int property_id, double value)
{
}

void printEncoder(RaspiCamCvCapture * capture) {
	//RASPIVID_STATE * state = capture->pState;
	//printf("%o", state->encoder_component->output[0]);
}

IplImage * raspiCamCvQueryFrame(RaspiCamCvCapture * capture)
{
	RASPIVID_STATE * state = capture->pState;
	vcos_semaphore_post(&state->capture_sem);
	vcos_semaphore_wait(&state->capture_done_sem);

	if (state->graymode==0)
	{
		cvResize(state->pu, state->pu_big, CV_INTER_NN);
		cvResize(state->pv, state->pv_big, CV_INTER_NN);  //CV_INTER_LINEAR looks better but it's slower
		cvMerge(state->py, state->pu_big, state->pv_big, NULL, state->yuvImage);

		cvCvtColor(state->yuvImage,state->dstImage,CV_YCrCb2RGB);	// convert in RGB color space (slow)
		return state->dstImage;
	}
	return state->py;
}

int counter2 = 0;

IplImage * raspiCamCvQueryFrame_New(RaspiCamCvCapture * capture, int mode)
{
	RASPIVID_STATE * state = capture->pState;
	vcos_semaphore_post(&state->capture_sem);
	vcos_semaphore_wait(&state->capture_done_sem);
	counter2++;
	printf("video_buffer_callback: %d\tFrames: %d\n", counter, counter2);

	if (state->graymode==0)
	//if (mode==0)
	{
		if (mode == 0) {
			return state->py;			
		} else if (mode == 1) {
			cvResize(state->pu, state->pu_big, CV_INTER_NN);
			cvResize(state->pv, state->pv_big, CV_INTER_NN);  //CV_INTER_LINEAR looks better but it's slower
			//cvResize(state->pu, state->pu_big, CV_INTER_AREA);
			//cvResize(state->pv, state->pv_big, CV_INTER_AREA);
			cvMerge(state->py, state->pu_big, state->pv_big, NULL, state->yuvImage);
			//cvMerge(state->py, state->pu, state->pv, NULL, state->yuvImage);

			cvCvtColor(state->yuvImage,state->dstImage,CV_YCrCb2RGB);	// convert in RGB color space (slow)
			return state->dstImage;
		} else if (mode == 2) {
			cvResize(state->pu, state->pu_big, CV_INTER_NN);
			cvResize(state->pv, state->pv_big, CV_INTER_NN);  //CV_INTER_LINEAR looks better but it's slower
			//cvResize(state->pu, state->pu_big, CV_INTER_AREA);
			//cvResize(state->pv, state->pv_big, CV_INTER_AREA);
			cvMerge(state->py, state->pu_big, state->pv_big, NULL, state->yuvImage);
			//cvMerge(state->py, state->pu, state->pv, NULL, state->yuvImage);

			//cvCvtColor(state->yuvImage,state->dstImage,CV_YCrCb2RGB);	// convert in RGB color space (slow)
			return state->yuvImage;
		}
	}
	return state->py;
}

IplImage * raspiCamCvQueryFrame_New_Final(RaspiCamCvCapture * capture) {
	RASPIVID_STATE * state = capture->pState;
	vcos_semaphore_post(&state->capture_sem);
	vcos_semaphore_wait(&state->capture_done_sem);

	cvResize(state->pu, state->pu_big, CV_INTER_NN);
	cvResize(state->pv, state->pv_big, CV_INTER_NN);  //CV_INTER_LINEAR looks better but it's slower
	//cvResize(state->pu, state->pu_big, CV_INTER_AREA);
	//cvResize(state->pv, state->pv_big, CV_INTER_AREA);
	cvMerge(state->py, state->pu_big, state->pv_big, NULL, state->yuvImage);
	//cvMerge(state->py, state->pu, state->pv, NULL, state->yuvImage);

	//cvCvtColor(state->yuvImage,state->dstImage,CV_YCrCb2RGB);	// convert in RGB color space (slow)
	return state->yuvImage;
}
