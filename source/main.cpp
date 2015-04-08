#include "KinectV1.h"
#include "KinectV2.h"
#include "Kinect.h"

int main()
{
	Kinect* Kinect = new KinectV1();

	cv::Mat* colorMat;
	cv::Mat* depthMat;
	cv::namedWindow("Color");
	cv::namedWindow("Depth");

	while (1)
	{
		//Update Color and Depth Stream
		Kinect->update();
		colorMat = Kinect->getColorMat();
		depthMat = Kinect->getDepthMat();

		// Show me what u got 
		cv::imshow("Color", *colorMat);
		cv::imshow("Depth", *depthMat);
		if (cv::waitKey(30) == VK_ESCAPE)
		{
			break;
		}
	}

}