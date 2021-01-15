#include "vs2mm.h"

/**
 * Read image from memory and output as video stream
 */
void vs2mm(uint32_t* mem, ap_uint<12> width, ap_uint<12> height, ap_uint<12> stride, video_stream& stream)
{
#pragma HLS INTERFACE s_axilite port=return bundle=regs
#pragma HLS INTERFACE m_axi depth=2304 latency=6 port=mem offset=slave
#pragma HLS INTERFACE s_axilite port=width bundle=regs
#pragma HLS INTERFACE s_axilite port=height bundle=regs
#pragma HLS INTERFACE s_axilite port=stride bundle=regs
#pragma HLS INTERFACE axis register both port=stream

	stream_interface pix;
	// wait for start of frame
	do
	{
#pragma HLS LOOP_TRIPCOUNT min=1 max=1
#pragma HLS PIPELINE II=1
		stream >> pix;
	} while (!pix.user);

	ap_uint<21> rowAddr = 0;
	y_loop2: for(ap_uint<12> y = 0; y < height; y++)
	{
#pragma HLS LOOP_TRIPCOUNT min=720 max=720
		x_loop2: for(ap_uint<12> x = 0; x < (width/4); x++)
		{
#pragma HLS PIPELINE II=4
#pragma HLS LOOP_TRIPCOUNT min=320 max=320
			ap_uint<32> buf;
			// read four pixels at once and write as 32 bit to mem
			for(int i = 0; i < 4; i++)
			{
				buf.range(8*i+7, 8*i) = pix.data;
				// read pixel for the next iteration. Because of the wait for SOF pix is one pixel ahead.
				if (y < height-1 || x < width/4-1 || i < 3)
				{
					stream >> pix;
				}
			}
			mem[rowAddr + x] = buf;
		}
		rowAddr += stride/4;
	}
}
