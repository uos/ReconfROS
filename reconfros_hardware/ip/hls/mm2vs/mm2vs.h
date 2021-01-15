#ifndef __MM2VS_H__
#define __MM2VS_H__

#include <hls_video.h>

typedef ap_axiu<24, 1, 1, 1> stream_interface;
typedef hls::stream<stream_interface> video_stream;

void mm2vs(uint32_t* mem, ap_uint<12> width, ap_uint<12> height, ap_uint<12> stride, video_stream& stream);

#endif
