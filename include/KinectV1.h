#pragma once 

#include <Windows.h>
#include "NuiApi.h"
#include "Kinect.h"
#include <opencv2/opencv.hpp>


/*	Kinect v1 Specs
*	Depth Resolution: 320 x 240 px
*	Color Resolution: 640 x 480 px
*	Depth Range Near Mode: 40 cm to 4000 cm
*	Horizontal FOV: 57°
*	Vertical FOv: 43°
*/



// Mat type for each usage
static const int COLOR_TYPE = CV_8UC4;
static const int DEPTH_TYPE = CV_16U;


class KinectV1 : public Kinect
{

private:

	//Kinect Stuff
	INuiSensor* pNuiSensor;
	HANDLE pColorStreamHandle;
	HANDLE pDepthStreamHandle;

	// Image stream data
	BYTE* colorBuffer;
	INT colorBufferSize;
	INT colorBufferPitch;
	BYTE* depthBuffer;
	INT depthBufferSize;
	INT	depthBufferPitch;


	//OpenCV Mats
	cv::Mat colorMat;
	cv::Mat depthMat;
	DWORD width;
	DWORD height;

	int initializeDefaultSensor()
	{
		INuiSensor * _pNuiSensor;
		HRESULT hr;

		int iSensorCount = 0;
		hr = NuiGetSensorCount(&iSensorCount);
		if (FAILED(hr))
		{
			return -1;
		}

		// Look at each Kinect sensor
		for (int i = 0; i < iSensorCount; ++i)
		{
			// Create the sensor so we can check status, if we can't create it, move on to the next
			hr = NuiCreateSensorByIndex(i, &_pNuiSensor);
			if (FAILED(hr))
			{
				continue;
			}

			// Get the status of the sensor, and if connected, then we can initialize it
			hr = _pNuiSensor->NuiStatus();
			if (S_OK == hr)
			{
				pNuiSensor = _pNuiSensor;
				break;
			}

			// This sensor wasn't OK, so release it since we're not using it
			_pNuiSensor->Release();
		}

		if (NULL != pNuiSensor)
		{
			// Initialize the Kinect and specify that we'll be using color
			hr = pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH);
			if (SUCCEEDED(hr))
			{
				// Init Matrices
				NuiImageResolutionToSize(NUI_IMAGE_RESOLUTION_640x480, width, height);
				cv::Size size(width, height);
				colorMat.create(size, COLOR_TYPE);
				depthMat.create(size, DEPTH_TYPE);


				// Open a color image stream to receive color frames
				hr = pNuiSensor->NuiImageStreamOpen(
					NUI_IMAGE_TYPE_COLOR,
					NUI_IMAGE_RESOLUTION_640x480,
					0,
					2,
					NULL,
					&pColorStreamHandle);

				// Open depth stream
				hr = pNuiSensor->NuiImageStreamOpen(
					NUI_IMAGE_TYPE_DEPTH,
					NUI_IMAGE_RESOLUTION_640x480,
					0,
					2,
					NULL,
					&pDepthStreamHandle);

			}
		}

		//Set Nearmode
		hr = pNuiSensor->NuiImageStreamSetImageFrameFlags(pDepthStreamHandle, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE);
		if (FAILED(hr)){
			std::cerr << "Cant set NearMode" << std::endl;
		}


		if (NULL == pNuiSensor || FAILED(hr))
		{
			std::cerr << "No ready Kinect found!" << std::endl;
			return -1;
		}

		return 0;
	};

	HRESULT updateDepthData()
	{

		HRESULT hr = UpdateDepthFrame();

		if (FAILED(hr))
		{
			return hr;
		}

		// Check if image is valid
		if (depthBufferPitch == 0)
		{
			return E_NUI_FRAME_NO_DATA;
		}

		// Copy image information into Mat
		USHORT* pBufferRun = reinterpret_cast<USHORT*>(depthBuffer);

		for (UINT y = 0; y < height; ++y)
		{
			// Get row pointer for depth Mat
			USHORT* pDepthRow = depthMat.ptr<USHORT>(y);

			for (UINT x = 0; x < width; ++x)
			{
				pDepthRow[x] = pBufferRun[y * width + x];
			}
		}

		return S_OK;
	}

	HRESULT UpdateDepthFrame()
	{

		// Get next image stream frame
		NUI_IMAGE_FRAME imageFrame;

		HRESULT hr = pNuiSensor->NuiImageStreamGetNextFrame(
			pDepthStreamHandle,
			0,
			&imageFrame);
		if (FAILED(hr))
		{
			return hr;
		}

		// Lock frame texture to allow for copy
		INuiFrameTexture* pTexture = imageFrame.pFrameTexture;
		NUI_LOCKED_RECT lockedRect;
		pTexture->LockRect(0, &lockedRect, NULL, 0);

		// Check if image is valid
		if (lockedRect.Pitch != 0)
		{
			// Copy image information into buffer
			BYTE* pBuffer = lockedRect.pBits;
			INT size = lockedRect.size;
			INT pitch = lockedRect.Pitch;

			// Only reallocate memory if the buffer size has changed
			if (size != depthBufferSize)
			{
				delete[] depthBuffer;
				depthBuffer = new BYTE[size];
				depthBufferSize = size;
			}
			memcpy_s(depthBuffer, size, pBuffer, size);

			depthBufferPitch = pitch;

		}

		// Unlock texture
		pTexture->UnlockRect(0);

		// Release image stream frame
		hr = pNuiSensor->NuiImageStreamReleaseFrame(pDepthStreamHandle, &imageFrame);

		return hr;
	}

	HRESULT updateColorData()
	{

		NUI_IMAGE_FRAME imageFrame;
		NUI_LOCKED_RECT LockedRect;
		if (pNuiSensor->NuiImageStreamGetNextFrame(pColorStreamHandle, 0, &imageFrame) < 0)
			return S_FALSE;


		INuiFrameTexture* texture = imageFrame.pFrameTexture;
		texture->LockRect(0, &LockedRect, NULL, 0);
		if (LockedRect.Pitch != 0)
		{
			// Copy image information into buffer
			BYTE * ColorBuffer = static_cast<BYTE *>(LockedRect.pBits);
			INT size = LockedRect.size;
			INT pitch = LockedRect.Pitch;

			// Only reallocate memory if the buffer size has changed
			if (size != colorBufferSize)
			{
				delete[] colorBuffer;
				colorBuffer = new BYTE[size];
				colorBufferSize = size;
			}
			memcpy_s(colorBuffer, size, ColorBuffer, size);

			colorBufferPitch = pitch;

			// use the simple structure of iplimage to convert from byte* to iplimage to mat
			// TODO: function who converts Byte directly to Mat.
			IplImage* color = cvCreateImageHeader(cvSize(640, 480), IPL_DEPTH_8U, 4);
			cvSetData(color, ColorBuffer, LockedRect.Pitch);
			colorMat = cv::cvarrToMat(color, false, true, 0);

		}

		texture->UnlockRect(0);
		pNuiSensor->NuiImageStreamReleaseFrame(pColorStreamHandle, &imageFrame);

		return S_OK;
	};

public:

	KinectV1()
	{
		initializeDefaultSensor();
	};

	~KinectV1()
	{
		if (pNuiSensor)
		{
			pNuiSensor->NuiShutdown();
		}

		SafeRelease(pNuiSensor);
	};

	void update()
	{
		HRESULT hr;

		hr = updateColorData();
		if (FAILED(hr))
		{
			std::cerr << "Couldn't update Color Data" << std::endl;
		}

		hr = updateDepthData();
		if (FAILED(hr))
		{
			std::cerr << "Couldn't update Depth Data. Updaterate to high? " << std::endl;
		}

	};

	cv::Mat* getColorMat()
	{
		return &colorMat;
	};

	cv::Mat* getDepthMat()
	{
		return &depthMat;
	};
};