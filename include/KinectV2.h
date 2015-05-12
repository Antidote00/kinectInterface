#pragma once 

#include <Windows.h>
#include <Kinect.h>
#include "Kinect.h"
#include <opencv2/opencv.hpp>

/*	Kinect v2 Specs
*	Depth Resolution: 512 x 424 px
*	Color Resolution: 1920 x 1080 px
*	Depth Range: 50 cm to 8000 cm
*	Horizontal FOV: 70°
*	Vertical FOv: 60°
*/

class KinectV2 : public Kinect
{

private:

	static const int cColorWidth = 1920;
	static const int cColorHeight = 1080;

	//Kinect Variables
	IKinectSensor* pSensor;
	IColorFrameSource* pColorSource;
	IColorFrameReader* pColorReader;
	IFrameDescription* pColorDescription;
	IDepthFrameSource* pDepthSource;
	IDepthFrameReader* pDepthReader;
	IFrameDescription* pDepthDescription;

	//OpenCV Mats
	cv::Mat colorBufferMat;
	cv::Mat colorMat;
	cv::Mat depthBufferMat;
	cv::Mat depthMat;

	// Buffer for Converting
	unsigned int colorBufferSize;
	unsigned int depthBufferSize;

private:

	int initializeDefaultSensor()
	{
		// Sensor
		HRESULT hResult = S_OK;
		hResult = GetDefaultKinectSensor(&pSensor);
		if (FAILED(hResult)){
			std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
			return -1;
		}
		hResult = pSensor->Open();
		if (FAILED(hResult)){
			std::cerr << "Error : IKinectSensor::Open()" << std::endl;
			return -1;
		}
		// Color Source
		hResult = pSensor->get_ColorFrameSource(&pColorSource);
		if (FAILED(hResult)){
			std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
			return -1;
		}
		// Color Reader
		hResult = pColorSource->OpenReader(&pColorReader);
		if (FAILED(hResult)){
			std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
			return -1;
		}

		// Color Description
		hResult = pColorSource->get_FrameDescription(&pColorDescription);
		if (FAILED(hResult)){
			std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
			return -1;
		}

		//Depth Source
		hResult = pSensor->get_DepthFrameSource(&pDepthSource);
		if (FAILED(hResult)){
			std::cerr << "Error : IKinectSensor::get_DepthFrameSource()" << std::endl;
			return -1;
		}

		//Depth Reader
		hResult = pDepthSource->OpenReader(&pDepthReader);
		if (FAILED(hResult)){
			std::cerr << "Error : IDepthFrameSource::OpenReader()" << std::endl;
			return -1;
		}

		//Depth Descriptor 
		IFrameDescription* pDepthDescription;
		hResult = pDepthSource->get_FrameDescription(&pDepthDescription);
		if (FAILED(hResult)){
			std::cerr << "Error : IDepthFrameSource::get_FrameDescription()" << std::endl;
			return -1;
		}

		//Create Color Mat
		int width = 0;
		int height = 0;
		pColorDescription->get_Width(&width); // 1920
		pColorDescription->get_Height(&height); // 1080
		colorBufferSize = width * height * 4 * sizeof(unsigned char);
		colorBufferMat.create(height, width, CV_8UC4);
		colorMat.create(height / 2, width / 2, CV_8UC4);

		//Create Depth Mat
		pDepthDescription->get_Width(&width); // 512
		pDepthDescription->get_Height(&height); // 424
		depthBufferSize = width * height * sizeof(unsigned short);
		// Range ( Range of Depth is 500-8000[mm], Range of Detection is 500-4500[mm] )
		unsigned short min = 0;
		unsigned short max = 0;
		pDepthSource->get_DepthMinReliableDistance(&min); // 500
		pDepthSource->get_DepthMaxReliableDistance(&max); // 4500
		depthBufferMat.create(height, width, CV_16UC1);
		depthMat.create(height, width, CV_8UC1);
	};

public:

	KinectV2()
	{
		initializeDefaultSensor();
	};

	~KinectV2()
	{
		//Release everything
		SafeRelease(pColorSource);
		SafeRelease(pColorReader);
		SafeRelease(pColorDescription);
		SafeRelease(pDepthSource);
		SafeRelease(pDepthReader);
		SafeRelease(pDepthDescription);
		if (pSensor){
			pSensor->Close();
		}
		SafeRelease(pSensor);
	}


	void update()
	{
		// Color Frame
		IColorFrame* pColorFrame = nullptr;
		HRESULT hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
		if (SUCCEEDED(hResult)){
			hResult = pColorFrame->CopyConvertedFrameDataToArray(colorBufferSize, reinterpret_cast<BYTE*>(colorBufferMat.data), ColorImageFormat::ColorImageFormat_Bgra);
			if (SUCCEEDED(hResult)){
				cv::resize(colorBufferMat, colorMat, cv::Size(), 0.5, 0.5);
			}
		}
		SafeRelease(pColorFrame);

		// Depth Frame
		IDepthFrame* pDepthFrame = nullptr;
		hResult = pDepthReader->AcquireLatestFrame(&pDepthFrame);
		if (SUCCEEDED(hResult)){
			hResult = pDepthFrame->AccessUnderlyingBuffer(&depthBufferSize, reinterpret_cast<UINT16**>(&depthBufferMat.data));
			if (SUCCEEDED(hResult)){
				depthBufferMat.convertTo(depthMat, CV_8U, -255.0f / 8000.0f, 255.0f);
			}
		}
		SafeRelease(pDepthFrame);

	};

	// Get the Color Matrix with half of the Resolution.
	cv::Mat* getColorMat()
	{
		return &colorMat;
	};

	cv::Mat* getDepthMat()
	{
		return &depthMat;
	};

};