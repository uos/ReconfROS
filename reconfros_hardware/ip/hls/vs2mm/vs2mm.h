#ifndef __VS2MM_H__
#define __VS2MM_H__

#include <hls_video.h>

typedef ap_axiu<8, 1, 1, 1> stream_interface;
typedef hls::stream<stream_interface> video_stream;

void vs2mm(uint32_t* mem, ap_uint<12> width, ap_uint<12> height, ap_uint<12> stride, video_stream& stream);

#endif
