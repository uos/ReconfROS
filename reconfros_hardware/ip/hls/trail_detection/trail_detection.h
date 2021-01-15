#ifndef _TRAIL_DETECTION_
#define _TRAIL_DETECTION_

#include <hls_video.h>

#define MAX_HEIGHT 720
#define MAX_WIDTH 1280

// Fixed parameters in hardware (from SW team)
#define CLOSING_KSIZE 7
#define OPENING_KSIZE 19
#define BLUR_KSIZE 5
#define BLUR_SIGMA 0 // automatic calculation

// Maximum number of blocks for the weighted average step
#define INDEX_TABLE_SIZE 16

typedef hls::Mat<MAX_HEIGHT, MAX_WIDTH, HLS_8UC3> mat24;   // BGR- Image with 3*8Bit=24Bit
typedef hls::Mat<MAX_HEIGHT, MAX_WIDTH, HLS_8UC1> mat8;    // Grayscale image with 8Bit
typedef hls::Mat<MAX_HEIGHT, MAX_WIDTH, HLS_SC(1,1)> mat1; // Binary Image with 1Bit

// Definitions for streaming ports
typedef ap_axiu<24, 1 ,1, 1> rgb_interface;
typedef hls::stream<rgb_interface> rgb_stream;

typedef ap_axiu<8, 1 ,1, 1> gray_interface;
typedef hls::stream<gray_interface> gray_stream;

// Types for weights and result
typedef ap_fixed<16, 4> weight_t;
typedef ap_fixed<16, 12> result_t;

void trail_detection(rgb_stream& src, gray_stream& dst, ap_uint<12> width, ap_uint<12> height,
		uint16_t index_table[INDEX_TABLE_SIZE], weight_t index_weights[INDEX_TABLE_SIZE],
		int8_t blue_offset, uint8_t threshold,
		result_t& result_x, result_t& result_y);

#endif
