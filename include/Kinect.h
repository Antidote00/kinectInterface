#pragma once 

#include <opencv2/opencv.hpp>

enum KINECTVERSION
{
	KINECTV1 = 1,
	KINECTV2 = 2
};

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL){
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
};


class Kinect
{

protected:
	Kinect(){}

public:

	virtual ~Kinect(){}

	virtual void update() = 0;
	virtual cv::Mat* getColorMat() = 0;
	virtual cv::Mat* getDepthMat() = 0;
	//virtual KINECTVERSION getKinectVersion() = 0;
};