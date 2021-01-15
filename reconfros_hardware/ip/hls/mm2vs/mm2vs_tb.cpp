#include "mm2vs.h"
#include <opencv2/opencv.hpp>

int main()
{
	// Load image and resize
	cv::Mat orig = cv::imread("test.jpg");
	cv::Mat img;
	cv::resize(orig, img, cv::Size(), 0.1, 0.1, CV_INTER_LINEAR);
	video_stream stream("videostream");

	// start IP top function
	mm2vs((uint32_t*)img.data, img.cols, img.rows, img.cols, stream);

	// compare output with input and check protocol
	cv::Mat out(img.size(), img.type());
	uint8_t* img_ptr = (uint8_t*)img.data;
	uint8_t* img_out = (uint8_t*)out.data;
	bool error = false;
	for(int y = 0; y < img.rows; y++)
	{
		for( int x = 0; x < img.cols; x++)
		{
			stream_interface p;
			stream >> p;
			for(int i = 0; i < 3; i++)
			{
				uint8_t d = p.data.range(8*i+7, 8*i);
				*img_out++ = d;
				if(d != *img_ptr++)
					error = true;
			}
			if(p.user != (y == 0 && x == 0))
				error = true;
			if(p.last != (x >= img.cols-1))
				error = true;
		}
	}
	cv::imwrite("out.jpg", out);
	return error ? -1 : 0;
}
