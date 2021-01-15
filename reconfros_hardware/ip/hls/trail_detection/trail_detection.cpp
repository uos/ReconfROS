#include "trail_detection.h"

/**
 * Apply green-to-black, gray conversion and thresholding in one step.
 */
void rgb_thresh(mat24& src, mat1& dst, int8_t blue_offset, uint8_t threshold)
{
	int cols = src.cols;
	int rows = src.rows;
	assert(rows <= MAX_HEIGHT);
	assert(cols <= MAX_WIDTH);
	hls::Scalar<3, uint8_t> s;
	hls::Scalar<1, uint8_t> g;
	hls::Scalar<1, ap_uint<1> > d;
	// BGR2GRAY kernel from the HLS OpenCV implementation
	hls::kernel_CvtColor<HLS_BGR2GRAY, uint8_t, uint8_t> cvtColor;

	loop_height: for(int y = 0; y < rows; y++) {
		loop_width: for (int x = 0; x < cols; x++) {
#pragma HLS LOOP_FLATTEN OFF
#pragma HLS PIPELINE
			src >> s;

			// gray conversion
			cvtColor.apply(s, g);
			if((s.val[1] >= s.val[0] + blue_offset) || g.val[0] <= threshold) // green or too dark?
			{
				d.val[0] = 0;
			}
			else
			{
				d.val[0] = 1;
			}

			dst << d;
		}
	}
}

/**
 * compute the weighted average of the x and y values of all white pixels.
 */
void trail_extraction(mat1& src, uint16_t index_table[INDEX_TABLE_SIZE], weight_t index_weights[INDEX_TABLE_SIZE],
		result_t& result_x, result_t& result_y)
{
	int cols = src.cols;
	int rows = src.rows;
	assert(rows <= MAX_HEIGHT);
	assert(cols <= MAX_WIDTH);
	hls::Scalar<1, ap_uint<1> > s;
	uint8_t idx = 0;
	weight_t weightssum = 0;
	result_t res_x[INDEX_TABLE_SIZE];
	result_t res_y[INDEX_TABLE_SIZE];
	uint32_t sumpos_x = 0;
	uint32_t cnt = 0;

	loop_height: for(int y = 0; y < rows; y++) {
		if(idx < INDEX_TABLE_SIZE && y == index_table[idx]) // reached end of current block?
		{
			// add weighted average of x values and weighted y value to results table
			if(cnt)
			{
				res_x[idx] = sumpos_x * index_weights[idx] / cnt;
				res_y[idx] = y * index_weights[idx];
				weightssum += index_weights[idx];
			}
			else
			{
				res_x[idx] = 0;
				res_y[idx] = 0;
			}
			idx++;
			sumpos_x = 0;
			cnt = 0;
		}

		loop_width: for (int x = 0; x < cols; x++) {
#pragma HLS LOOP_FLATTEN OFF
#pragma HLS PIPELINE
			src >> s;
			if(s.val[0])
			{
				// accumulate x values
				sumpos_x += x;
				cnt++;
			}
		}
	}

	// data types for storing the sum without overflow
	ap_fixed<result_t::width + INDEX_TABLE_SIZE, result_t::iwidth + INDEX_TABLE_SIZE> sum_res_x = 0;
	ap_fixed<result_t::width + INDEX_TABLE_SIZE, result_t::iwidth + INDEX_TABLE_SIZE> sum_res_y = 0;
	if(weightssum)
	{
		// compute sum of valid table entries
		for(int i = 0; i < INDEX_TABLE_SIZE; i++)
		{
#pragma HLS UNROLL
			if(i < idx)
			{
				sum_res_x += res_x[i];
				sum_res_y += res_y[i];
			}
		}
		result_x = sum_res_x / weightssum;
		result_y = sum_res_y / weightssum;
	}
	else
    {
        result_x = 0;
        result_y = 0;
    }
}

/**
 * Convert binary image to 8 bit grayscale.
 */
void bin_to_gray(mat1& src, mat8& dst)
{
	int cols = src.cols;
	int rows = src.rows;
	assert(rows <= MAX_HEIGHT);
	assert(cols <= MAX_WIDTH);
	hls::Scalar<1, ap_uint<1> > s;
	hls::Scalar<1, uint8_t> d;

	loop_height: for(int y = 0; y < rows; y++) {
		loop_width: for (int x = 0; x < cols; x++) {
#pragma HLS LOOP_FLATTEN OFF
#pragma HLS PIPELINE
			src >> s;
			d.val[0] = s.val[0] ? 255 : 0;
			dst << d;
		}
	}
}

/**
 * Specialized erode kernel for binary images
 */
class erode_bin_kernel
{
    public:
    template<typename SRC_T,typename DST_T,typename FILTER_T, int F_HEIGHT, int F_WIDTH>
    void apply(hls::Window<F_HEIGHT,F_WIDTH,FILTER_T>	        &_kernel_filter,
    		hls::Window<F_HEIGHT,F_WIDTH,SRC_T>	                &_kernel_pixel,
            DST_T               				&out)
    {
#pragma HLS INLINE
        out = -1; // init with all 1s; ap_int with one bit stores only 0 and -1
    loop_height: for( int m=0;m< F_HEIGHT;m++)
        {
        loop_width: for( int n=0;n<F_WIDTH;n++)
            {
        		// find min -> only 0s and 1s -> and operation returns 0 if there is at least one 0
        		// or with inverted kernel to exclude pixels outside of the structuring element
                out &= ~_kernel_filter.val[F_HEIGHT-m-1][F_WIDTH-1-n] | _kernel_pixel.val[F_HEIGHT-m-1][(F_WIDTH-1-n)];
            }
        }
    }
};
/**
 * Specialized dilate kernel for binary images
 */
class dilate_bin_kernel
{
    public:
    template<typename SRC_T,typename DST_T,typename FILTER_T, int F_HEIGHT, int F_WIDTH>
    void apply(hls::Window<F_HEIGHT,F_WIDTH,FILTER_T>	        &_kernel_filter,
    		hls::Window<F_HEIGHT,F_WIDTH,SRC_T>	                &_kernel_pixel,
            DST_T               				&out)
    {
#pragma HLS INLINE
        out = 0; // init with all 0s
    loop_height: for( int m=0;m< F_HEIGHT;m++)
        {
        loop_width: for( int n=0;n<F_WIDTH;n++)
            {
				// find max -> only 0s and 1s -> or operation returns 1 if there is at least one 1
				// and with inverted kernel to exclude pixels outside of the structuring element
                out |= _kernel_filter.val[F_HEIGHT-m-1][F_WIDTH-1-n] & _kernel_pixel.val[F_HEIGHT-m-1][(F_WIDTH-1-n)];
            }
        }
    }
};

/**
 * Unrolled version of the HLS version.
 * Reduces the resources for big kernel sizes by calculating the kernel completely during compile time.
 * ksize and anchor must be constants to benefit from it.
 */
template <typename SRC_T,typename SIZE_T,typename POINT_T,int HEIGHT,int WIDTH>
void getStructuringElementUnrolled(
        int 				shape,
		hls::Size_<SIZE_T> 	        ksize,
		hls::Point_<POINT_T>            anchor,
		hls::Window<HEIGHT,WIDTH,SRC_T> &result)
{
#pragma HLS inline
    int i, j;
    int r = 0, c = 0;
    ap_fixed<31,11,AP_RND> inv_r2 = 0;

    if( ksize.width==1&&ksize.height == 1 )
        shape = hls::MORPH_RECT;

    if( shape == hls::MORPH_ELLIPSE )
    {
        r = ksize.height/2;
        c = ksize.width/2;
        if(r!=0)
        {
            inv_r2 =(ap_fixed<31,11,AP_RND>) 1/(r*r) ;
        }
        else
            inv_r2=0;
    }
 loop_height: for( i = 0; i < ksize.height; i++ )
    {
#pragma HLS UNROLL
        int j1 = 0, j2 = 0;
        if( shape == hls::MORPH_RECT || (shape ==hls::MORPH_CROSS && i == anchor.y) )
            j2 = ksize.width;
        else if( shape == hls::MORPH_CROSS ) {
            j1 = anchor.x;
            j2 = j1 + 1;
        } else
        {
            int dy = i - r;
            if( hls::abs(dy) <= r )
            {
			  ap_fixed<12,12,AP_RND> dxx = (c*::hls::sqrt(double(((r-dy)*(r+dy))*inv_r2)));///////saturate_cast<int>  nearly rand    <- not our comment...
                int dx=dxx;
                j1 = (c - dx)> 0?( c - dx):0;
                j2 =  (c + dx + 1)< ksize.width?(c + dx + 1):ksize.width;
            }
        }
    loop_width: for( j = 0; j < ksize.width; j++ ) {
#pragma HLS UNROLL
            if(shape == hls::MORPH_RECT || (j >= j1 && j < j2)) {
                result.val[i][j] = 1;
            } else {
                result.val[i][j] = 0;
            }
        }
    }
}

/**
 * Create structuring element and apply custom erode kernel
 */
template<int KSIZE>
void erode(mat1& src, mat1& dst)
{
	hls::Window<KSIZE, KSIZE, ap_int<1> > structElem;
	getStructuringElementUnrolled(hls::MORPH_ELLIPSE, hls::Size(KSIZE, KSIZE), hls::Point(KSIZE/2, KSIZE/2), structElem);
	hls::Point_<int> anchor(KSIZE/2,KSIZE/2);
	hls::filter_opr<erode_bin_kernel, hls::BORDER_REPLICATE>::filter(src,dst,structElem,anchor,src.rows,src.cols);
}

/**
 * Create structuring element and apply custom dilate kernel
 */
template<int KSIZE>
void dilate(mat1& src, mat1& dst)
{
	hls::Window<KSIZE, KSIZE, ap_int<1> > structElem;
	getStructuringElementUnrolled(hls::MORPH_ELLIPSE, hls::Size(KSIZE, KSIZE), hls::Point(KSIZE/2, KSIZE/2), structElem);
	hls::Point_<int> anchor(KSIZE/2,KSIZE/2);
	hls::filter_opr<dilate_bin_kernel, hls::BORDER_REPLICATE>::filter(src,dst,structElem,anchor,src.rows,src.cols);
}

/**
 * Top-function of the IP.
 * Apply all image processing steps according to the defined pipeline to the source image
 */
void trail_detection(rgb_stream& src, gray_stream& dst, ap_uint<12> width, ap_uint<12> height,
		uint16_t index_table[INDEX_TABLE_SIZE], weight_t index_weights[INDEX_TABLE_SIZE],
		int8_t blue_offset, uint8_t threshold,
		result_t& result_x, result_t& result_y)
{
#pragma HLS ARRAY_PARTITION variable=index_table complete dim=1
#pragma HLS ARRAY_PARTITION variable=index_weights complete dim=1
#pragma HLS INTERFACE axis register both port=src
#pragma HLS INTERFACE axis register both port=dst
#pragma HLS INTERFACE s_axilite port=width
#pragma HLS INTERFACE s_axilite port=height
#pragma HLS INTERFACE s_axilite port=index_table
#pragma HLS INTERFACE s_axilite port=index_weights
#pragma HLS INTERFACE s_axilite port=blue_offset
#pragma HLS INTERFACE s_axilite port=threshold
#pragma HLS INTERFACE s_axilite port=result_x
#pragma HLS INTERFACE s_axilite port=result_y
#pragma HLS INTERFACE s_axilite port=return

#pragma HLS DATAFLOW
	// buffer between functions
	mat24 mat_src(height, width);
	mat24 gauss(height, width);
	mat1 thresh(height, width);
	mat1 erode1(height, width);
	mat1 dilate1(height, width);
	mat1 dilate2(height, width);
	mat1 erode2(height, width);
	mat1 extract(height, width);
	mat1 bin(height, width);
	mat8 mat_dst(height, width);

	// Apply image processing functions
	hls::AXIvideo2Mat(src, mat_src);
	hls::GaussianBlur<BLUR_KSIZE, BLUR_KSIZE>(mat_src, gauss, BLUR_SIGMA, BLUR_SIGMA);
	rgb_thresh(gauss, thresh, blue_offset, threshold);

	dilate<CLOSING_KSIZE>(thresh, dilate1);
	erode<CLOSING_KSIZE>(dilate1, erode1);
	erode<OPENING_KSIZE>(erode1, erode2);
	dilate<OPENING_KSIZE>(erode2, dilate2);

	hls::Duplicate(dilate2, extract, bin);
	trail_extraction(extract, index_table, index_weights, result_x, result_y);
	bin_to_gray(bin, mat_dst);
	hls::Mat2AXIvideo(mat_dst, dst);
}
