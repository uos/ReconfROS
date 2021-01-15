#include "trail_detection.h"
#include <hls_opencv.h>

int main()
{
	// Load image and resize
	cv::Mat orig = cv::imread("test.jpg");
	cv::Mat reference = cv::imread("reference.png", cv::IMREAD_GRAYSCALE);
	cv::Mat img;
	cv::resize(orig, img, cv::Size(), 0.1, 0.1, CV_INTER_LINEAR);
	cv::Mat out(img.size(), CV_8UC1);
	rgb_stream in_stream("input");
	gray_stream out_stream("output");

	// set parameters
	uint16_t index_table[INDEX_TABLE_SIZE] = {0xFFFF};
	weight_t index_weights[INDEX_TABLE_SIZE] = {0};

	index_table[0] = 8;
	index_table[1] = 16;
	index_table[2] = 32;
	index_table[3] = 64;

	index_weights[0] = 1;
	index_weights[1] = 1;
	index_weights[2] = 1;
	index_weights[3] = 1;

	int8_t blue_offset = 30;
	uint8_t threshold = 127;
	result_t result_x = 0;
	result_t result_y = 0;

	// convert image to stream
	cvMat2AXIvideo(img, in_stream);

	// invoke top-function
	trail_detection(in_stream, out_stream, img.cols, img.rows, index_table, index_weights, blue_offset, threshold, result_x, result_y);

	// convert output stream to image
	AXIvideo2cvMat(out_stream, out);

	cv::circle(out, cv::Point(result_x, result_y), 5, cv::Scalar(128), -1);

	// compare with reference
	bool failed = false;
	for(int y = 0; y < img.rows; y++)
	{
		for(int x = 0; x < img.cols; x++)
		{
			uint8_t out_pix = out.at<uint8_t>(y, x);
			uint8_t ref = reference.at<uint8_t>(y, x);
			if(out_pix != ref)
			{
				//out.at<uint8_t>(y, x) = 128;
				failed = true;
			}
		}
	}

	cv::imwrite("out.png", out);

	std::cout << "Failed: " << failed << std::endl;

	return failed;
}
