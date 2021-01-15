#include "mm2vs.h"

/**
 * Read image from memory and output as video stream
 */
void mm2vs(uint32_t* mem, ap_uint<12> width, ap_uint<12> height, ap_uint<12> stride, video_stream& stream)
{
#pragma HLS INTERFACE s_axilite port=return bundle=regs
#pragma HLS INTERFACE m_axi depth=2764800 latency=6 port=mem offset=slave
#pragma HLS INTERFACE s_axilite port=width bundle=regs
#pragma HLS INTERFACE s_axilite port=height bundle=regs
#pragma HLS INTERFACE s_axilite port=stride bundle=regs
#pragma HLS INTERFACE axis register both port=stream

#pragma HLS DATAFLOW

	hls::stream<ap_uint<32> > buffer("buffer");
#pragma HLS STREAM variable=buffer depth=16 dim=1

	// Process 1: Read data from memory into buffer
	ap_uint<21> rowAddr = 0;
	y_loop1: for(ap_uint<12> y = 0; y < height; y++)
	{
#pragma HLS LOOP_TRIPCOUNT min=720 max=720
		// * 3 bytes per pixel / 4 bytes per memory access
		x_loop1: for(ap_uint<12> x = 0; x < (width*3/4); x++)
		{
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=960 max=960
			buffer << mem[rowAddr + x];
		}
		rowAddr += stride*3/4;
	}

	// Process 2: Repack 32-bit words to 24-bit pixels and write to stream
	bool sof = true;
	y_loop2: for(ap_uint<12> y = 0; y < height; y++)
	{
#pragma HLS LOOP_TRIPCOUNT min=720 max=720
		x_loop2: for(ap_uint<12> x = 0; x < (width/4); x++)
		{
#pragma HLS PIPELINE II=4
#pragma HLS LOOP_TRIPCOUNT min=320 max=320
			// read 3 words and output as 4 pixel
			ap_uint<96> buf;
			for(int i = 0; i < 3; i++)
			{
				buf.range(32*i+31, 32*i) = buffer.read();
			}
			for(int i = 0; i < 4; i++)
			{
				stream_interface pixel;
				pixel.data = buf.range(24*i+23, 24*i);
				pixel.user = sof;
				pixel.last = (x >= (width/4)-1 && i == 3);
				stream << pixel;
				sof = false;
			}
		}
	}
}
