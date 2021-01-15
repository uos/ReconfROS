#include "vs2mm.h"
#include <opencv2/opencv.hpp>

int main()
{
	// load image and resize
	cv::Mat orig = cv::imread("test.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat img;
	cv::resize(orig, img, cv::Size(), 0.1, 0.1, CV_INTER_LINEAR);
	cv::Mat out(img.size(), img.type());
	video_stream stream("videostream");

	// convert image to stream
	uint8_t* img_ptr = (uint8_t*)img.data;
	for(int y = 0; y < img.rows; y++)
	{
		for( int x = 0; x < img.cols; x++)
		{
			stream_interface p;
			p.data = *img_ptr++;
			p.user = (x == 0 && y == 0);
			p.last = (x == img.cols-1);
			stream << p;
		}
	}

	// start IP top function
	vs2mm((uint32_t*)out.data, img.cols, img.rows, img.cols, stream);

	// compare output with input
	img_ptr = (uint8_t*)img.data;
	uint8_t* img_out = (uint8_t*)out.data;
	bool error = false;
	for(int y = 0; y < img.rows; y++)
	{
		for( int x = 0; x < img.cols; x++)
		{
			if(*img_out++ != *img_ptr++)
				error = true;
		}
	}
	cv::imwrite("out.jpg", out);
	return error ? -1 : 0;
}
