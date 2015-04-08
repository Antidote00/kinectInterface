#pragma once 

#include <Windows.h>
#include "NuiApi.h"
#include "Kinect.h"
#include <opencv2/opencv.hpp>


// Mat type for each usage
static const int COLOR_TYPE = CV_8UC4;
static const int DEPTH_TYPE = CV_16U;


class KinectV1 : public Kinect
{

private:

	static const int        cColorWidth = 640;
	static const int        cColorHeight = 480;

	static const int        cStatusMessageMaxLen = MAX_PATH * 2;
	
	//Kinect Stuff
	INuiSensor*             m_pNuiSensor;
	HANDLE                  m_pColorStreamHandle;
	HANDLE					m_pDepthStreamHandle;

	// Image stream data
	BYTE*	_ColorBuffer;
	INT		_colorBufferSize;
	INT		_colorBufferPitch;
	BYTE*	_DepthBuffer;
	INT		_depthBufferSize;
	INT		_depthBufferPitch;


	//OpenCV Mats
	cv::Mat colorMat;
	cv::Mat depthMat;
	DWORD width;
	DWORD height;

	// Buffer for Converting
	unsigned int colorBufferSize;
	unsigned int depthBufferSize;

	int initializeDefaultSensor()
	{
		INuiSensor * pNuiSensor;
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
			hr = NuiCreateSensorByIndex(i, &pNuiSensor);
			if (FAILED(hr))
			{
				continue;
			}

			// Get the status of the sensor, and if connected, then we can initialize it
			hr = pNuiSensor->NuiStatus();
			if (S_OK == hr)
			{
				m_pNuiSensor = pNuiSensor;
				break;
			}

			// This sensor wasn't OK, so release it since we're not using it
			pNuiSensor->Release();
		}

		if (NULL != m_pNuiSensor)
		{
			// Initialize the Kinect and specify that we'll be using color
			hr = m_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH);
			if (SUCCEEDED(hr))
			{
				// Init Matrices
				NuiImageResolutionToSize(NUI_IMAGE_RESOLUTION_640x480, width, height);
				cv::Size size(width, height);
				colorMat.create(size, COLOR_TYPE);
				depthMat.create(size, DEPTH_TYPE);


				// Open a color image stream to receive color frames
				hr = m_pNuiSensor->NuiImageStreamOpen(
					NUI_IMAGE_TYPE_COLOR,
					NUI_IMAGE_RESOLUTION_640x480,
					0,
					2,
					NULL,
					&m_pColorStreamHandle);

				// Open depth stream
				hr = m_pNuiSensor->NuiImageStreamOpen(
					NUI_IMAGE_TYPE_DEPTH,
					NUI_IMAGE_RESOLUTION_640x480,
					0,
					2,
					NULL,
					&m_pDepthStreamHandle);

			}
		}

		//Set Nearmode
		hr = m_pNuiSensor->NuiImageStreamSetImageFrameFlags(m_pDepthStreamHandle, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE);
		if (FAILED(hr)){
			std::cerr << "Cant set NearMode" << std::endl;
		}


		if (NULL == m_pNuiSensor || FAILED(hr))
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
		if (_depthBufferPitch == 0)
		{
			return E_NUI_FRAME_NO_DATA;
		}

		// Copy image information into Mat
		USHORT* pBufferRun = reinterpret_cast<USHORT*>(_DepthBuffer);

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

		HRESULT hr = m_pNuiSensor->NuiImageStreamGetNextFrame(
			m_pDepthStreamHandle,
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
			if (size != _depthBufferSize)
			{
				delete[] _DepthBuffer;
				_DepthBuffer = new BYTE[size];
				_depthBufferSize = size;
			}
			memcpy_s(_DepthBuffer, size, pBuffer, size);

			_depthBufferPitch = pitch;

		}

		// Unlock texture
		pTexture->UnlockRect(0);

		// Release image stream frame
		hr = m_pNuiSensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &imageFrame);

		return hr;
	}

	HRESULT updateColorData()
	{

		NUI_IMAGE_FRAME imageFrame;
		NUI_LOCKED_RECT LockedRect;
		if (m_pNuiSensor->NuiImageStreamGetNextFrame(m_pColorStreamHandle, 0, &imageFrame) < 0)
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
			if (size != _colorBufferSize)
			{
				delete[] _ColorBuffer;
				_ColorBuffer = new BYTE[size];
				_colorBufferSize = size;
			}
			memcpy_s(_ColorBuffer, size, ColorBuffer, size);

			_colorBufferPitch = pitch;

			// use the simple structure of iplimage to convert from byte* to iplimage to mat
			// TODO: function who converts Byte directly to Mat.
			IplImage* color = cvCreateImageHeader(cvSize(640, 480), IPL_DEPTH_8U, 4);
			cvSetData(color, ColorBuffer, LockedRect.Pitch);
			colorMat = cv::cvarrToMat(color, false, true, 0);

		}

		texture->UnlockRect(0);
		m_pNuiSensor->NuiImageStreamReleaseFrame(m_pColorStreamHandle, &imageFrame);

		return S_OK;
	};

public:

	KinectV1()
	{
		initializeDefaultSensor();
	};

	~KinectV1()
	{
		if (m_pNuiSensor)
		{
			m_pNuiSensor->NuiShutdown();
		}

		SafeRelease(m_pNuiSensor);
	};

	void update()
	{
		HRESULT hr;

		hr = updateColorData();
		if (FAILED(hr))
		{
			std::cerr << "Cant update Color Data" << std::endl;
		}

		hr = updateDepthData();
		if (FAILED(hr))
		{
			std::cerr << "Cant update Depth Data" << std::endl;
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