/*
* 1394-Based Digital Camera Control Library
*
* Bayer pattern decoding functions
*
* Written by Damien Douxchamps and Frederic Devernay
* The original VNG and AHD Bayer decoding are from Dave Coffin's DCRAW.
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
//#include "conversions.h"
#include "bayer.h"


#define CLIP(in, out)\
   in = in < 0 ? 0 : in;\
   in = in > 255 ? 255 : in;\
   out=in;

#define CLIP16(in, out, bits)\
   in = in < 0 ? 0 : in;\
   in = in > ((1<<bits)-1) ? ((1<<bits)-1) : in;\
   out=in;

void
ClearBorders(uint8_t *rgb, int sx, int sy, int w)
{
	int i, j;
	// black edges are added with a width w:
	i = 3 * sx * w - 1;
	j = 3 * sx * sy - 1;
	while (i >= 0) {
		rgb[i--] = 0;
		rgb[j--] = 0;
	}

	int low = sx * (w - 1) * 3 - 1 + w * 3;
	i = low + sx * (sy - w * 2 + 1) * 3;
	while (i > low) {
		j = 6 * w;
		while (j > 0) {
			rgb[i--] = 0;
			j--;
		}
		i -= (sx - 2 * w) * 3;
	}
}

void
ClearBorders_uint16(uint16_t * rgb, int sx, int sy, int w)
{
	int i, j;

	// black edges:
	i = 3 * sx * w - 1;
	j = 3 * sx * sy - 1;
	while (i >= 0) {
		rgb[i--] = 0;
		rgb[j--] = 0;
	}

	int low = sx * (w - 1) * 3 - 1 + w * 3;
	i = low + sx * (sy - w * 2 + 1) * 3;
	while (i > low) {
		j = 6 * w;
		while (j > 0) {
			rgb[i--] = 0;
			j--;
		}
		i -= (sx - 2 * w) * 3;
	}

}

/**************************************************************
*     Color conversion functions for cameras that can        *
* output raw-Bayer pattern images, such as some Basler and   *
* Point Grey camera. Most of the algos presented here come   *
* from http://www-ise.stanford.edu/~tingchen/ and have been  *
* converted from Matlab to C and extended to all elementary  *
* patterns.                                                  *
**************************************************************/

/* 8-bits versions */
/* insprired by OpenCV's Bayer decoding */

dc1394error_t
dc1394_bayer_NearestNeighbor(const uint8_t * bayer, uint8_t * rgb, int sx, int sy, int tile)
{
	const int bayerStep = sx;
	const int rgbStep = 3 * sx;
	int width = sx;
	int height = sy;
	int blue = tile == DC1394_COLOR_FILTER_BGGR
		|| tile == DC1394_COLOR_FILTER_GBRG ? -1 : 1;
	int start_with_green = tile == DC1394_COLOR_FILTER_GBRG
		|| tile == DC1394_COLOR_FILTER_GRBG;
	int i, imax, iinc;

	if ((tile>DC1394_COLOR_FILTER_MAX) || (tile<DC1394_COLOR_FILTER_MIN))
		return DC1394_INVALID_COLOR_FILTER;

	/* add black border */
	imax = sx * sy * 3;
	for (i = sx * (sy - 1) * 3; i < imax; i++) {
		rgb[i] = 0;
	}
	iinc = (sx - 1) * 3;
	for (i = (sx - 1) * 3; i < imax; i += iinc) {
		rgb[i++] = 0;
		rgb[i++] = 0;
		rgb[i++] = 0;
	}

	rgb += 1;
	width -= 1;
	height -= 1;

	for (; height--; bayer += bayerStep, rgb += rgbStep) {
		//int t0, t1;
		const uint8_t *bayerEnd = bayer + width;

		if (start_with_green) {
			rgb[-blue] = bayer[1];
			rgb[0] = bayer[bayerStep + 1];
			rgb[blue] = bayer[bayerStep];
			bayer++;
			rgb += 3;
		}

		if (blue > 0) {
			for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
				rgb[-1] = bayer[0];
				rgb[0] = bayer[1];
				rgb[1] = bayer[bayerStep + 1];

				rgb[2] = bayer[2];
				rgb[3] = bayer[bayerStep + 2];
				rgb[4] = bayer[bayerStep + 1];
			}
		}
		else {
			for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
				rgb[1] = bayer[0];
				rgb[0] = bayer[1];
				rgb[-1] = bayer[bayerStep + 1];

				rgb[4] = bayer[2];
				rgb[3] = bayer[bayerStep + 2];
				rgb[2] = bayer[bayerStep + 1];
			}
		}

		if (bayer < bayerEnd) {
			rgb[-blue] = bayer[0];
			rgb[0] = bayer[1];
			rgb[blue] = bayer[bayerStep + 1];
			bayer++;
			rgb += 3;
		}

		bayer -= width;
		rgb -= width * 3;

		blue = -blue;
		start_with_green = !start_with_green;
	}

	return DC1394_SUCCESS;
}

/* OpenCV's Bayer decoding */
dc1394error_t
dc1394_bayer_Bilinear(const uint8_t * bayer, uint8_t * rgb, int sx, int sy, int tile)
{
	const int bayerStep = sx;
	const int rgbStep = 3 * sx;
	int width = sx;
	int height = sy;
	/*
	the two letters  of the OpenCV name are respectively
	the 4th and 3rd letters from the blinky name,
	and we also have to switch R and B (OpenCV is BGR)

	CV_BayerBG2BGR <-> DC1394_COLOR_FILTER_BGGR
	CV_BayerGB2BGR <-> DC1394_COLOR_FILTER_GBRG
	CV_BayerGR2BGR <-> DC1394_COLOR_FILTER_GRBG

	int blue = tile == CV_BayerBG2BGR || tile == CV_BayerGB2BGR ? -1 : 1;
	int start_with_green = tile == CV_BayerGB2BGR || tile == CV_BayerGR2BGR;
	*/
	int blue = tile == DC1394_COLOR_FILTER_BGGR
		|| tile == DC1394_COLOR_FILTER_GBRG ? -1 : 1;
	int start_with_green = tile == DC1394_COLOR_FILTER_GBRG
		|| tile == DC1394_COLOR_FILTER_GRBG;

	if ((tile>DC1394_COLOR_FILTER_MAX) || (tile<DC1394_COLOR_FILTER_MIN))
		return DC1394_INVALID_COLOR_FILTER;

	ClearBorders(rgb, sx, sy, 1);
	rgb += rgbStep + 3 + 1;
	height -= 2;
	width -= 2;

	for (; height--; bayer += bayerStep, rgb += rgbStep) {
		int t0, t1;
		const uint8_t *bayerEnd = bayer + width;

		if (start_with_green) {
			/* OpenCV has a bug in the next line, which was
			t0 = (bayer[0] + bayer[bayerStep * 2] + 1) >> 1; */
			t0 = (bayer[1] + bayer[bayerStep * 2 + 1] + 1) >> 1;
			t1 = (bayer[bayerStep] + bayer[bayerStep + 2] + 1) >> 1;
			rgb[-blue] = (uint8_t)t0;
			rgb[0] = bayer[bayerStep + 1];
			rgb[blue] = (uint8_t)t1;
			bayer++;
			rgb += 3;
		}

		if (blue > 0) {
			for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
				t0 = (bayer[0] + bayer[2] + bayer[bayerStep * 2] +
					bayer[bayerStep * 2 + 2] + 2) >> 2;
				t1 = (bayer[1] + bayer[bayerStep] +
					bayer[bayerStep + 2] + bayer[bayerStep * 2 + 1] +
					2) >> 2;
				rgb[-1] = (uint8_t)t0;
				rgb[0] = (uint8_t)t1;
				rgb[1] = bayer[bayerStep + 1];

				t0 = (bayer[2] + bayer[bayerStep * 2 + 2] + 1) >> 1;
				t1 = (bayer[bayerStep + 1] + bayer[bayerStep + 3] +
					1) >> 1;
				rgb[2] = (uint8_t)t0;
				rgb[3] = bayer[bayerStep + 2];
				rgb[4] = (uint8_t)t1;
			}
		}
		else {
			for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
				t0 = (bayer[0] + bayer[2] + bayer[bayerStep * 2] +
					bayer[bayerStep * 2 + 2] + 2) >> 2;
				t1 = (bayer[1] + bayer[bayerStep] +
					bayer[bayerStep + 2] + bayer[bayerStep * 2 + 1] +
					2) >> 2;
				rgb[1] = (uint8_t)t0;
				rgb[0] = (uint8_t)t1;
				rgb[-1] = bayer[bayerStep + 1];

				t0 = (bayer[2] + bayer[bayerStep * 2 + 2] + 1) >> 1;
				t1 = (bayer[bayerStep + 1] + bayer[bayerStep + 3] +
					1) >> 1;
				rgb[4] = (uint8_t)t0;
				rgb[3] = bayer[bayerStep + 2];
				rgb[2] = (uint8_t)t1;
			}
		}

		if (bayer < bayerEnd) {
			t0 = (bayer[0] + bayer[2] + bayer[bayerStep * 2] +
				bayer[bayerStep * 2 + 2] + 2) >> 2;
			t1 = (bayer[1] + bayer[bayerStep] +
				bayer[bayerStep + 2] + bayer[bayerStep * 2 + 1] +
				2) >> 2;
			rgb[-blue] = (uint8_t)t0;
			rgb[0] = (uint8_t)t1;
			rgb[blue] = bayer[bayerStep + 1];
			bayer++;
			rgb += 3;
		}

		bayer -= width;
		rgb -= width * 3;

		blue = -blue;
		start_with_green = !start_with_green;
	}
	return DC1394_SUCCESS;
}

/* High-Quality Linear Interpolation For Demosaicing Of
Bayer-Patterned Color Images, by Henrique S. Malvar, Li-wei He, and
Ross Cutler, in ICASSP'04 */
dc1394error_t
dc1394_bayer_HQLinear(const uint8_t * bayer, uint8_t * rgb, int sx, int sy, int tile)
{
	const int bayerStep = sx;
	const int rgbStep = 3 * sx;
	int width = sx;
	int height = sy;
	int blue = tile == DC1394_COLOR_FILTER_BGGR
		|| tile == DC1394_COLOR_FILTER_GBRG ? -1 : 1;
	int start_with_green = tile == DC1394_COLOR_FILTER_GBRG
		|| tile == DC1394_COLOR_FILTER_GRBG;

	if ((tile>DC1394_COLOR_FILTER_MAX) || (tile<DC1394_COLOR_FILTER_MIN))
		return DC1394_INVALID_COLOR_FILTER;

	ClearBorders(rgb, sx, sy, 2);
	rgb += 2 * rgbStep + 6 + 1;
	height -= 4;
	width -= 4;

	/* We begin with a (+1 line,+1 column) offset with respect to bilinear decoding, so start_with_green is the same, but blue is opposite */
	blue = -blue;

	for (; height--; bayer += bayerStep, rgb += rgbStep) {
		int t0, t1;
		const uint8_t *bayerEnd = bayer + width;
		const int bayerStep2 = bayerStep * 2;
		const int bayerStep3 = bayerStep * 3;
		const int bayerStep4 = bayerStep * 4;

		if (start_with_green) {
			/* at green pixel */
			rgb[0] = bayer[bayerStep2 + 2];
			t0 = rgb[0] * 5
				+ ((bayer[bayerStep + 2] + bayer[bayerStep3 + 2]) << 2)
				- bayer[2]
				- bayer[bayerStep + 1]
				- bayer[bayerStep + 3]
				- bayer[bayerStep3 + 1]
				- bayer[bayerStep3 + 3]
				- bayer[bayerStep4 + 2]
				+ ((bayer[bayerStep2] + bayer[bayerStep2 + 4] + 1) >> 1);
			t1 = rgb[0] * 5 +
				((bayer[bayerStep2 + 1] + bayer[bayerStep2 + 3]) << 2)
				- bayer[bayerStep2]
				- bayer[bayerStep + 1]
				- bayer[bayerStep + 3]
				- bayer[bayerStep3 + 1]
				- bayer[bayerStep3 + 3]
				- bayer[bayerStep2 + 4]
				+ ((bayer[2] + bayer[bayerStep4 + 2] + 1) >> 1);
			t0 = (t0 + 4) >> 3;
			CLIP(t0, rgb[-blue]);
			t1 = (t1 + 4) >> 3;
			CLIP(t1, rgb[blue]);
			bayer++;
			rgb += 3;
		}

		if (blue > 0) {
			for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
				/* B at B */
				rgb[1] = bayer[bayerStep2 + 2];
				/* R at B */
				t0 = ((bayer[bayerStep + 1] + bayer[bayerStep + 3] +
					bayer[bayerStep3 + 1] + bayer[bayerStep3 + 3]) << 1)
					-
					(((bayer[2] + bayer[bayerStep2] +
					bayer[bayerStep2 + 4] + bayer[bayerStep4 +
					2]) * 3 + 1) >> 1)
					+ rgb[1] * 6;
				/* G at B */
				t1 = ((bayer[bayerStep + 2] + bayer[bayerStep2 + 1] +
					bayer[bayerStep2 + 3] + bayer[bayerStep3 + 2]) << 1)
					- (bayer[2] + bayer[bayerStep2] +
					bayer[bayerStep2 + 4] + bayer[bayerStep4 + 2])
					+ (rgb[1] << 2);
				t0 = (t0 + 4) >> 3;
				CLIP(t0, rgb[-1]);
				t1 = (t1 + 4) >> 3;
				CLIP(t1, rgb[0]);
				/* at green pixel */
				rgb[3] = bayer[bayerStep2 + 3];
				t0 = rgb[3] * 5
					+ ((bayer[bayerStep + 3] + bayer[bayerStep3 + 3]) << 2)
					- bayer[3]
					- bayer[bayerStep + 2]
					- bayer[bayerStep + 4]
					- bayer[bayerStep3 + 2]
					- bayer[bayerStep3 + 4]
					- bayer[bayerStep4 + 3]
					+
					((bayer[bayerStep2 + 1] + bayer[bayerStep2 + 5] +
					1) >> 1);
				t1 = rgb[3] * 5 +
					((bayer[bayerStep2 + 2] + bayer[bayerStep2 + 4]) << 2)
					- bayer[bayerStep2 + 1]
					- bayer[bayerStep + 2]
					- bayer[bayerStep + 4]
					- bayer[bayerStep3 + 2]
					- bayer[bayerStep3 + 4]
					- bayer[bayerStep2 + 5]
					+ ((bayer[3] + bayer[bayerStep4 + 3] + 1) >> 1);
				t0 = (t0 + 4) >> 3;
				CLIP(t0, rgb[2]);
				t1 = (t1 + 4) >> 3;
				CLIP(t1, rgb[4]);
			}
		}
		else {
			for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
				/* R at R */
				rgb[-1] = bayer[bayerStep2 + 2];
				/* B at R */
				t0 = ((bayer[bayerStep + 1] + bayer[bayerStep + 3] +
					bayer[bayerStep3 + 1] + bayer[bayerStep3 + 3]) << 1)
					-
					(((bayer[2] + bayer[bayerStep2] +
					bayer[bayerStep2 + 4] + bayer[bayerStep4 +
					2]) * 3 + 1) >> 1)
					+ rgb[-1] * 6;
				/* G at R */
				t1 = ((bayer[bayerStep + 2] + bayer[bayerStep2 + 1] +
					bayer[bayerStep2 + 3] + bayer[bayerStep * 3 +
					2]) << 1)
					- (bayer[2] + bayer[bayerStep2] +
					bayer[bayerStep2 + 4] + bayer[bayerStep4 + 2])
					+ (rgb[-1] << 2);
				t0 = (t0 + 4) >> 3;
				CLIP(t0, rgb[1]);
				t1 = (t1 + 4) >> 3;
				CLIP(t1, rgb[0]);

				/* at green pixel */
				rgb[3] = bayer[bayerStep2 + 3];
				t0 = rgb[3] * 5
					+ ((bayer[bayerStep + 3] + bayer[bayerStep3 + 3]) << 2)
					- bayer[3]
					- bayer[bayerStep + 2]
					- bayer[bayerStep + 4]
					- bayer[bayerStep3 + 2]
					- bayer[bayerStep3 + 4]
					- bayer[bayerStep4 + 3]
					+
					((bayer[bayerStep2 + 1] + bayer[bayerStep2 + 5] +
					1) >> 1);
				t1 = rgb[3] * 5 +
					((bayer[bayerStep2 + 2] + bayer[bayerStep2 + 4]) << 2)
					- bayer[bayerStep2 + 1]
					- bayer[bayerStep + 2]
					- bayer[bayerStep + 4]
					- bayer[bayerStep3 + 2]
					- bayer[bayerStep3 + 4]
					- bayer[bayerStep2 + 5]
					+ ((bayer[3] + bayer[bayerStep4 + 3] + 1) >> 1);
				t0 = (t0 + 4) >> 3;
				CLIP(t0, rgb[4]);
				t1 = (t1 + 4) >> 3;
				CLIP(t1, rgb[2]);
			}
		}

		if (bayer < bayerEnd) {
			/* B at B */
			rgb[blue] = bayer[bayerStep2 + 2];
			/* R at B */
			t0 = ((bayer[bayerStep + 1] + bayer[bayerStep + 3] +
				bayer[bayerStep3 + 1] + bayer[bayerStep3 + 3]) << 1)
				-
				(((bayer[2] + bayer[bayerStep2] +
				bayer[bayerStep2 + 4] + bayer[bayerStep4 +
				2]) * 3 + 1) >> 1)
				+ rgb[blue] * 6;
			/* G at B */
			t1 = (((bayer[bayerStep + 2] + bayer[bayerStep2 + 1] +
				bayer[bayerStep2 + 3] + bayer[bayerStep3 + 2])) << 1)
				- (bayer[2] + bayer[bayerStep2] +
				bayer[bayerStep2 + 4] + bayer[bayerStep4 + 2])
				+ (rgb[blue] << 2);
			t0 = (t0 + 4) >> 3;
			CLIP(t0, rgb[-blue]);
			t1 = (t1 + 4) >> 3;
			CLIP(t1, rgb[0]);
			bayer++;
			rgb += 3;
		}

		bayer -= width;
		rgb -= width * 3;

		blue = -blue;
		start_with_green = !start_with_green;
	}

	return DC1394_SUCCESS;

}

/* coriander's Bayer decoding */
/* Edge Sensing Interpolation II from http://www-ise.stanford.edu/~tingchen/ */
/*   (Laroche,Claude A.  "Apparatus and method for adaptively
interpolating a full color image utilizing chrominance gradients"
U.S. Patent 5,373,322) */
dc1394error_t
dc1394_bayer_EdgeSense(const uint8_t * bayer, uint8_t * rgb, int sx, int sy, int tile)
{
	/* Removed due to patent concerns */
	return DC1394_FUNCTION_NOT_SUPPORTED;
}

/* coriander's Bayer decoding */
dc1394error_t
dc1394_bayer_Downsample(const uint8_t * bayer, uint8_t * rgb, int sx, int sy, int tile)
{
	uint8_t *outR, *outG, *outB;
	register int i, j;
	int tmp;

	switch (tile) {
	case DC1394_COLOR_FILTER_GRBG:
	case DC1394_COLOR_FILTER_BGGR:
		outR = &rgb[0];
		outG = &rgb[1];
		outB = &rgb[2];
		break;
	case DC1394_COLOR_FILTER_GBRG:
	case DC1394_COLOR_FILTER_RGGB:
		outR = &rgb[2];
		outG = &rgb[1];
		outB = &rgb[0];
		break;
	default:
		return DC1394_INVALID_COLOR_FILTER;
	}

	switch (tile) {
	case DC1394_COLOR_FILTER_GRBG:        //---------------------------------------------------------
	case DC1394_COLOR_FILTER_GBRG:
		for (i = 0; i < sy*sx; i += (sx << 1)) {
			for (j = 0; j < sx; j += 2) {
				tmp = ((bayer[i + j] + bayer[i + sx + j + 1]) >> 1);
				CLIP(tmp, outG[((i >> 2) + (j >> 1)) * 3]);
				tmp = bayer[i + sx + j + 1];
				CLIP(tmp, outR[((i >> 2) + (j >> 1)) * 3]);
				tmp = bayer[i + sx + j];
				CLIP(tmp, outB[((i >> 2) + (j >> 1)) * 3]);
			}
		}
		break;
	case DC1394_COLOR_FILTER_BGGR:        //---------------------------------------------------------
	case DC1394_COLOR_FILTER_RGGB:
		for (i = 0; i < sy*sx; i += (sx << 1)) {
			for (j = 0; j < sx; j += 2) {
				tmp = ((bayer[i + sx + j] + bayer[i + j + 1]) >> 1);
				CLIP(tmp, outG[((i >> 2) + (j >> 1)) * 3]);
				tmp = bayer[i + sx + j + 1];
				CLIP(tmp, outR[((i >> 2) + (j >> 1)) * 3]);
				tmp = bayer[i + j];
				CLIP(tmp, outB[((i >> 2) + (j >> 1)) * 3]);
			}
		}
		break;
	}

	return DC1394_SUCCESS;

}

/* this is the method used inside AVT cameras. See AVT docs. */
dc1394error_t
dc1394_bayer_Simple(const uint8_t * bayer, uint8_t * rgb, int sx, int sy, int tile)
{
	const int bayerStep = sx;
	const int rgbStep = 3 * sx;
	int width = sx;
	int height = sy;
	int blue = tile == DC1394_COLOR_FILTER_BGGR
		|| tile == DC1394_COLOR_FILTER_GBRG ? -1 : 1;
	int start_with_green = tile == DC1394_COLOR_FILTER_GBRG
		|| tile == DC1394_COLOR_FILTER_GRBG;
	int i, imax, iinc;

	if ((tile>DC1394_COLOR_FILTER_MAX) || (tile<DC1394_COLOR_FILTER_MIN))
		return DC1394_INVALID_COLOR_FILTER;

	/* add black border */
	imax = sx * sy * 3;
	for (i = sx * (sy - 1) * 3; i < imax; i++) {
		rgb[i] = 0;
	}
	iinc = (sx - 1) * 3;
	for (i = (sx - 1) * 3; i < imax; i += iinc) {
		rgb[i++] = 0;
		rgb[i++] = 0;
		rgb[i++] = 0;
	}

	rgb += 1;
	width -= 1;
	height -= 1;

	for (; height--; bayer += bayerStep, rgb += rgbStep) {
		const uint8_t *bayerEnd = bayer + width;

		if (start_with_green) {
			rgb[-blue] = bayer[1];
			rgb[0] = (bayer[0] + bayer[bayerStep + 1] + 1) >> 1;
			rgb[blue] = bayer[bayerStep];
			bayer++;
			rgb += 3;
		}

		if (blue > 0) {
			for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
				rgb[-1] = bayer[0];
				rgb[0] = (bayer[1] + bayer[bayerStep] + 1) >> 1;
				rgb[1] = bayer[bayerStep + 1];

				rgb[2] = bayer[2];
				rgb[3] = (bayer[1] + bayer[bayerStep + 2] + 1) >> 1;
				rgb[4] = bayer[bayerStep + 1];
			}
		}
		else {
			for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
				rgb[1] = bayer[0];
				rgb[0] = (bayer[1] + bayer[bayerStep] + 1) >> 1;
				rgb[-1] = bayer[bayerStep + 1];

				rgb[4] = bayer[2];
				rgb[3] = (bayer[1] + bayer[bayerStep + 2] + 1) >> 1;
				rgb[2] = bayer[bayerStep + 1];
			}
		}

		if (bayer < bayerEnd) {
			rgb[-blue] = bayer[0];
			rgb[0] = (bayer[1] + bayer[bayerStep] + 1) >> 1;
			rgb[blue] = bayer[bayerStep + 1];
			bayer++;
			rgb += 3;
		}

		bayer -= width;
		rgb -= width * 3;

		blue = -blue;
		start_with_green = !start_with_green;
	}

	return DC1394_SUCCESS;

}

/* 16-bits versions */

/* insprired by OpenCV's Bayer decoding */
dc1394error_t
dc1394_bayer_NearestNeighbor_uint16(const uint16_t * bayer, uint16_t * rgb, int sx, int sy, int tile, int bits)
{
	const int bayerStep = sx;
	const int rgbStep = 3 * sx;
	int width = sx;
	int height = sy;
	int blue = tile == DC1394_COLOR_FILTER_BGGR
		|| tile == DC1394_COLOR_FILTER_GBRG ? -1 : 1;
	int start_with_green = tile == DC1394_COLOR_FILTER_GBRG
		|| tile == DC1394_COLOR_FILTER_GRBG;
	int i, iinc, imax;

	if ((tile>DC1394_COLOR_FILTER_MAX) || (tile<DC1394_COLOR_FILTER_MIN))
		return DC1394_INVALID_COLOR_FILTER;

	/* add black border */
	imax = sx * sy * 3;
	for (i = sx * (sy - 1) * 3; i < imax; i++) {
		rgb[i] = 0;
	}
	iinc = (sx - 1) * 3;
	for (i = (sx - 1) * 3; i < imax; i += iinc) {
		rgb[i++] = 0;
		rgb[i++] = 0;
		rgb[i++] = 0;
	}

	rgb += 1;
	height -= 1;
	width -= 1;

	for (; height--; bayer += bayerStep, rgb += rgbStep) {
		//int t0, t1;
		const uint16_t *bayerEnd = bayer + width;

		if (start_with_green) {
			rgb[-blue] = bayer[1];
			rgb[0] = bayer[bayerStep + 1];
			rgb[blue] = bayer[bayerStep];
			bayer++;
			rgb += 3;
		}

		if (blue > 0) {
			for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
				rgb[-1] = bayer[0];
				rgb[0] = bayer[1];
				rgb[1] = bayer[bayerStep + 1];

				rgb[2] = bayer[2];
				rgb[3] = bayer[bayerStep + 2];
				rgb[4] = bayer[bayerStep + 1];
			}
		}
		else {
			for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
				rgb[1] = bayer[0];
				rgb[0] = bayer[1];
				rgb[-1] = bayer[bayerStep + 1];

				rgb[4] = bayer[2];
				rgb[3] = bayer[bayerStep + 2];
				rgb[2] = bayer[bayerStep + 1];
			}
		}

		if (bayer < bayerEnd) {
			rgb[-blue] = bayer[0];
			rgb[0] = bayer[1];
			rgb[blue] = bayer[bayerStep + 1];
			bayer++;
			rgb += 3;
		}

		bayer -= width;
		rgb -= width * 3;

		blue = -blue;
		start_with_green = !start_with_green;
	}

	return DC1394_SUCCESS;

}
/* OpenCV's Bayer decoding */
dc1394error_t
dc1394_bayer_Bilinear_uint16(const uint16_t * bayer, uint16_t * rgb, int sx, int sy, int tile, int bits)
{
	const int bayerStep = sx;
	const int rgbStep = 3 * sx;
	int width = sx;
	int height = sy;
	int blue = tile == DC1394_COLOR_FILTER_BGGR
		|| tile == DC1394_COLOR_FILTER_GBRG ? -1 : 1;
	int start_with_green = tile == DC1394_COLOR_FILTER_GBRG
		|| tile == DC1394_COLOR_FILTER_GRBG;

	if ((tile>DC1394_COLOR_FILTER_MAX) || (tile<DC1394_COLOR_FILTER_MIN))
		return DC1394_INVALID_COLOR_FILTER;

	rgb += rgbStep + 3 + 1;
	height -= 2;
	width -= 2;

	for (; height--; bayer += bayerStep, rgb += rgbStep) {
		int t0, t1;
		const uint16_t *bayerEnd = bayer + width;

		if (start_with_green) {
			/* OpenCV has a bug in the next line, which was
			t0 = (bayer[0] + bayer[bayerStep * 2] + 1) >> 1; */
			t0 = (bayer[1] + bayer[bayerStep * 2 + 1] + 1) >> 1;
			t1 = (bayer[bayerStep] + bayer[bayerStep + 2] + 1) >> 1;
			rgb[-blue] = (uint16_t)t0;
			rgb[0] = bayer[bayerStep + 1];
			rgb[blue] = (uint16_t)t1;
			bayer++;
			rgb += 3;
		}

		if (blue > 0) {
			for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
				t0 = (bayer[0] + bayer[2] + bayer[bayerStep * 2] +
					bayer[bayerStep * 2 + 2] + 2) >> 2;
				t1 = (bayer[1] + bayer[bayerStep] +
					bayer[bayerStep + 2] + bayer[bayerStep * 2 + 1] +
					2) >> 2;
				rgb[-1] = (uint16_t)t0;
				rgb[0] = (uint16_t)t1;
				rgb[1] = bayer[bayerStep + 1];

				t0 = (bayer[2] + bayer[bayerStep * 2 + 2] + 1) >> 1;
				t1 = (bayer[bayerStep + 1] + bayer[bayerStep + 3] +
					1) >> 1;
				rgb[2] = (uint16_t)t0;
				rgb[3] = bayer[bayerStep + 2];
				rgb[4] = (uint16_t)t1;
			}
		}
		else {
			for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
				t0 = (bayer[0] + bayer[2] + bayer[bayerStep * 2] +
					bayer[bayerStep * 2 + 2] + 2) >> 2;
				t1 = (bayer[1] + bayer[bayerStep] +
					bayer[bayerStep + 2] + bayer[bayerStep * 2 + 1] +
					2) >> 2;
				rgb[1] = (uint16_t)t0;
				rgb[0] = (uint16_t)t1;
				rgb[-1] = bayer[bayerStep + 1];

				t0 = (bayer[2] + bayer[bayerStep * 2 + 2] + 1) >> 1;
				t1 = (bayer[bayerStep + 1] + bayer[bayerStep + 3] +
					1) >> 1;
				rgb[4] = (uint16_t)t0;
				rgb[3] = bayer[bayerStep + 2];
				rgb[2] = (uint16_t)t1;
			}
		}

		if (bayer < bayerEnd) {
			t0 = (bayer[0] + bayer[2] + bayer[bayerStep * 2] +
				bayer[bayerStep * 2 + 2] + 2) >> 2;
			t1 = (bayer[1] + bayer[bayerStep] +
				bayer[bayerStep + 2] + bayer[bayerStep * 2 + 1] +
				2) >> 2;
			rgb[-blue] = (uint16_t)t0;
			rgb[0] = (uint16_t)t1;
			rgb[blue] = bayer[bayerStep + 1];
			bayer++;
			rgb += 3;
		}

		bayer -= width;
		rgb -= width * 3;

		blue = -blue;
		start_with_green = !start_with_green;
	}

	return DC1394_SUCCESS;

}

/* High-Quality Linear Interpolation For Demosaicing Of
Bayer-Patterned Color Images, by Henrique S. Malvar, Li-wei He, and
Ross Cutler, in ICASSP'04 */
dc1394error_t
dc1394_bayer_HQLinear_uint16(const uint16_t * bayer, uint16_t * rgb, int sx, int sy, int tile, int bits)
{
	const int bayerStep = sx;
	const int rgbStep = 3 * sx;
	int width = sx;
	int height = sy;
	/*
	the two letters  of the OpenCV name are respectively
	the 4th and 3rd letters from the blinky name,
	and we also have to switch R and B (OpenCV is BGR)

	CV_BayerBG2BGR <-> DC1394_COLOR_FILTER_BGGR
	CV_BayerGB2BGR <-> DC1394_COLOR_FILTER_GBRG
	CV_BayerGR2BGR <-> DC1394_COLOR_FILTER_GRBG

	int blue = tile == CV_BayerBG2BGR || tile == CV_BayerGB2BGR ? -1 : 1;
	int start_with_green = tile == CV_BayerGB2BGR || tile == CV_BayerGR2BGR;
	*/
	int blue = tile == DC1394_COLOR_FILTER_BGGR
		|| tile == DC1394_COLOR_FILTER_GBRG ? -1 : 1;
	int start_with_green = tile == DC1394_COLOR_FILTER_GBRG
		|| tile == DC1394_COLOR_FILTER_GRBG;

	if ((tile>DC1394_COLOR_FILTER_MAX) || (tile<DC1394_COLOR_FILTER_MIN))
		return DC1394_INVALID_COLOR_FILTER;

	ClearBorders_uint16(rgb, sx, sy, 2);
	rgb += 2 * rgbStep + 6 + 1;
	height -= 4;
	width -= 4;

	/* We begin with a (+1 line,+1 column) offset with respect to bilinear decoding, so start_with_green is the same, but blue is opposite */
	blue = -blue;

	for (; height--; bayer += bayerStep, rgb += rgbStep) {
		int t0, t1;
		const uint16_t *bayerEnd = bayer + width;
		const int bayerStep2 = bayerStep * 2;
		const int bayerStep3 = bayerStep * 3;
		const int bayerStep4 = bayerStep * 4;

		if (start_with_green) {
			/* at green pixel */
			rgb[0] = bayer[bayerStep2 + 2];
			t0 = rgb[0] * 5
				+ ((bayer[bayerStep + 2] + bayer[bayerStep3 + 2]) << 2)
				- bayer[2]
				- bayer[bayerStep + 1]
				- bayer[bayerStep + 3]
				- bayer[bayerStep3 + 1]
				- bayer[bayerStep3 + 3]
				- bayer[bayerStep4 + 2]
				+ ((bayer[bayerStep2] + bayer[bayerStep2 + 4] + 1) >> 1);
			t1 = rgb[0] * 5 +
				((bayer[bayerStep2 + 1] + bayer[bayerStep2 + 3]) << 2)
				- bayer[bayerStep2]
				- bayer[bayerStep + 1]
				- bayer[bayerStep + 3]
				- bayer[bayerStep3 + 1]
				- bayer[bayerStep3 + 3]
				- bayer[bayerStep2 + 4]
				+ ((bayer[2] + bayer[bayerStep4 + 2] + 1) >> 1);
			t0 = (t0 + 4) >> 3;
			CLIP16(t0, rgb[-blue], bits);
			t1 = (t1 + 4) >> 3;
			CLIP16(t1, rgb[blue], bits);
			bayer++;
			rgb += 3;
		}

		if (blue > 0) {
			for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
				/* B at B */
				rgb[1] = bayer[bayerStep2 + 2];
				/* R at B */
				t0 = ((bayer[bayerStep + 1] + bayer[bayerStep + 3] +
					bayer[bayerStep3 + 1] + bayer[bayerStep3 + 3]) << 1)
					-
					(((bayer[2] + bayer[bayerStep2] +
					bayer[bayerStep2 + 4] + bayer[bayerStep4 +
					2]) * 3 + 1) >> 1)
					+ rgb[1] * 6;
				/* G at B */
				t1 = ((bayer[bayerStep + 2] + bayer[bayerStep2 + 1] +
					bayer[bayerStep2 + 3] + bayer[bayerStep * 3 +
					2]) << 1)
					- (bayer[2] + bayer[bayerStep2] +
					bayer[bayerStep2 + 4] + bayer[bayerStep4 + 2])
					+ (rgb[1] << 2);
				t0 = (t0 + 4) >> 3;
				CLIP16(t0, rgb[-1], bits);
				t1 = (t1 + 4) >> 3;
				CLIP16(t1, rgb[0], bits);
				/* at green pixel */
				rgb[3] = bayer[bayerStep2 + 3];
				t0 = rgb[3] * 5
					+ ((bayer[bayerStep + 3] + bayer[bayerStep3 + 3]) << 2)
					- bayer[3]
					- bayer[bayerStep + 2]
					- bayer[bayerStep + 4]
					- bayer[bayerStep3 + 2]
					- bayer[bayerStep3 + 4]
					- bayer[bayerStep4 + 3]
					+
					((bayer[bayerStep2 + 1] + bayer[bayerStep2 + 5] +
					1) >> 1);
				t1 = rgb[3] * 5 +
					((bayer[bayerStep2 + 2] + bayer[bayerStep2 + 4]) << 2)
					- bayer[bayerStep2 + 1]
					- bayer[bayerStep + 2]
					- bayer[bayerStep + 4]
					- bayer[bayerStep3 + 2]
					- bayer[bayerStep3 + 4]
					- bayer[bayerStep2 + 5]
					+ ((bayer[3] + bayer[bayerStep4 + 3] + 1) >> 1);
				t0 = (t0 + 4) >> 3;
				CLIP16(t0, rgb[2], bits);
				t1 = (t1 + 4) >> 3;
				CLIP16(t1, rgb[4], bits);
			}
		}
		else {
			for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
				/* R at R */
				rgb[-1] = bayer[bayerStep2 + 2];
				/* B at R */
				t0 = ((bayer[bayerStep + 1] + bayer[bayerStep + 3] +
					bayer[bayerStep * 3 + 1] + bayer[bayerStep3 +
					3]) << 1)
					-
					(((bayer[2] + bayer[bayerStep2] +
					bayer[bayerStep2 + 4] + bayer[bayerStep4 +
					2]) * 3 + 1) >> 1)
					+ rgb[-1] * 6;
				/* G at R */
				t1 = ((bayer[bayerStep + 2] + bayer[bayerStep2 + 1] +
					bayer[bayerStep2 + 3] + bayer[bayerStep3 + 2]) << 1)
					- (bayer[2] + bayer[bayerStep2] +
					bayer[bayerStep2 + 4] + bayer[bayerStep4 + 2])
					+ (rgb[-1] << 2);
				t0 = (t0 + 4) >> 3;
				CLIP16(t0, rgb[1], bits);
				t1 = (t1 + 4) >> 3;
				CLIP16(t1, rgb[0], bits);

				/* at green pixel */
				rgb[3] = bayer[bayerStep2 + 3];
				t0 = rgb[3] * 5
					+ ((bayer[bayerStep + 3] + bayer[bayerStep3 + 3]) << 2)
					- bayer[3]
					- bayer[bayerStep + 2]
					- bayer[bayerStep + 4]
					- bayer[bayerStep3 + 2]
					- bayer[bayerStep3 + 4]
					- bayer[bayerStep4 + 3]
					+
					((bayer[bayerStep2 + 1] + bayer[bayerStep2 + 5] +
					1) >> 1);
				t1 = rgb[3] * 5 +
					((bayer[bayerStep2 + 2] + bayer[bayerStep2 + 4]) << 2)
					- bayer[bayerStep2 + 1]
					- bayer[bayerStep + 2]
					- bayer[bayerStep + 4]
					- bayer[bayerStep3 + 2]
					- bayer[bayerStep3 + 4]
					- bayer[bayerStep2 + 5]
					+ ((bayer[3] + bayer[bayerStep4 + 3] + 1) >> 1);
				t0 = (t0 + 4) >> 3;
				CLIP16(t0, rgb[4], bits);
				t1 = (t1 + 4) >> 3;
				CLIP16(t1, rgb[2], bits);
			}
		}

		if (bayer < bayerEnd) {
			/* B at B */
			rgb[blue] = bayer[bayerStep2 + 2];
			/* R at B */
			t0 = ((bayer[bayerStep + 1] + bayer[bayerStep + 3] +
				bayer[bayerStep3 + 1] + bayer[bayerStep3 + 3]) << 1)
				-
				(((bayer[2] + bayer[bayerStep2] +
				bayer[bayerStep2 + 4] + bayer[bayerStep4 +
				2]) * 3 + 1) >> 1)
				+ rgb[blue] * 6;
			/* G at B */
			t1 = (((bayer[bayerStep + 2] + bayer[bayerStep2 + 1] +
				bayer[bayerStep2 + 3] + bayer[bayerStep3 + 2])) << 1)
				- (bayer[2] + bayer[bayerStep2] +
				bayer[bayerStep2 + 4] + bayer[bayerStep4 + 2])
				+ (rgb[blue] << 2);
			t0 = (t0 + 4) >> 3;
			CLIP16(t0, rgb[-blue], bits);
			t1 = (t1 + 4) >> 3;
			CLIP16(t1, rgb[0], bits);
			bayer++;
			rgb += 3;
		}

		bayer -= width;
		rgb -= width * 3;

		blue = -blue;
		start_with_green = !start_with_green;
	}

	return DC1394_SUCCESS;
}

/* coriander's Bayer decoding */
dc1394error_t
dc1394_bayer_EdgeSense_uint16(const uint16_t * bayer, uint16_t * rgb, int sx, int sy, int tile, int bits)
{
	/* Removed due to patent concerns */
	return DC1394_FUNCTION_NOT_SUPPORTED;
}

/* coriander's Bayer decoding */
dc1394error_t
dc1394_bayer_Downsample_uint16(const uint16_t * bayer, uint16_t * rgb, int sx, int sy, int tile, int bits)
{
	uint16_t *outR, *outG, *outB;
	register int i, j;
	int tmp;

	switch (tile) {
	case DC1394_COLOR_FILTER_GRBG:
	case DC1394_COLOR_FILTER_BGGR:
		outR = &rgb[0];
		outG = &rgb[1];
		outB = &rgb[2];
		break;
	case DC1394_COLOR_FILTER_GBRG:
	case DC1394_COLOR_FILTER_RGGB:
		outR = &rgb[2];
		outG = &rgb[1];
		outB = &rgb[0];
		break;
	default:
		return DC1394_INVALID_COLOR_FILTER;
	}

	switch (tile) {
	case DC1394_COLOR_FILTER_GRBG:        //---------------------------------------------------------
	case DC1394_COLOR_FILTER_GBRG:
		for (i = 0; i < sy*sx; i += (sx << 1)) {
			for (j = 0; j < sx; j += 2) {
				tmp =
					((bayer[i + j] + bayer[i + sx + j + 1]) >> 1);
				CLIP16(tmp, outG[((i >> 2) + (j >> 1)) * 3], bits);
				tmp = bayer[i + sx + j + 1];
				CLIP16(tmp, outR[((i >> 2) + (j >> 1)) * 3], bits);
				tmp = bayer[i + sx + j];
				CLIP16(tmp, outB[((i >> 2) + (j >> 1)) * 3], bits);
			}
		}
		break;
	case DC1394_COLOR_FILTER_BGGR:        //---------------------------------------------------------
	case DC1394_COLOR_FILTER_RGGB:
		for (i = 0; i < sy*sx; i += (sx << 1)) {
			for (j = 0; j < sx; j += 2) {
				tmp =
					((bayer[i + sx + j] + bayer[i + j + 1]) >> 1);
				CLIP16(tmp, outG[((i >> 2) + (j >> 1)) * 3], bits);
				tmp = bayer[i + sx + j + 1];
				CLIP16(tmp, outR[((i >> 2) + (j >> 1)) * 3], bits);
				tmp = bayer[i + j];
				CLIP16(tmp, outB[((i >> 2) + (j >> 1)) * 3], bits);
			}
		}
		break;
	}

	return DC1394_SUCCESS;

}

/* coriander's Bayer decoding */
dc1394error_t
dc1394_bayer_Simple_uint16(const uint16_t * bayer, uint16_t * rgb, int sx, int sy, int tile, int bits)
{
	uint16_t *outR, *outG, *outB;
	register int i, j;
	int tmp, base;

	// sx and sy should be even
	switch (tile) {
	case DC1394_COLOR_FILTER_GRBG:
	case DC1394_COLOR_FILTER_BGGR:
		outR = &rgb[0];
		outG = &rgb[1];
		outB = &rgb[2];
		break;
	case DC1394_COLOR_FILTER_GBRG:
	case DC1394_COLOR_FILTER_RGGB:
		outR = &rgb[2];
		outG = &rgb[1];
		outB = &rgb[0];
		break;
	default:
		return DC1394_INVALID_COLOR_FILTER;
	}

	switch (tile) {
	case DC1394_COLOR_FILTER_GRBG:
	case DC1394_COLOR_FILTER_BGGR:
		outR = &rgb[0];
		outG = &rgb[1];
		outB = &rgb[2];
		break;
	case DC1394_COLOR_FILTER_GBRG:
	case DC1394_COLOR_FILTER_RGGB:
		outR = &rgb[2];
		outG = &rgb[1];
		outB = &rgb[0];
		break;
	default:
		outR = NULL;
		outG = NULL;
		outB = NULL;
		break;
	}

	switch (tile) {
	case DC1394_COLOR_FILTER_GRBG:        //---------------------------------------------------------
	case DC1394_COLOR_FILTER_GBRG:
		for (i = 0; i < sy - 1; i += 2) {
			for (j = 0; j < sx - 1; j += 2) {
				base = i * sx + j;
				tmp = ((bayer[base] + bayer[base + sx + 1]) >> 1);
				CLIP16(tmp, outG[base * 3], bits);
				tmp = bayer[base + 1];
				CLIP16(tmp, outR[base * 3], bits);
				tmp = bayer[base + sx];
				CLIP16(tmp, outB[base * 3], bits);
			}
		}
		for (i = 0; i < sy - 1; i += 2) {
			for (j = 1; j < sx - 1; j += 2) {
				base = i * sx + j;
				tmp = ((bayer[base + 1] + bayer[base + sx]) >> 1);
				CLIP16(tmp, outG[(base)* 3], bits);
				tmp = bayer[base];
				CLIP16(tmp, outR[(base)* 3], bits);
				tmp = bayer[base + 1 + sx];
				CLIP16(tmp, outB[(base)* 3], bits);
			}
		}
		for (i = 1; i < sy - 1; i += 2) {
			for (j = 0; j < sx - 1; j += 2) {
				base = i * sx + j;
				tmp = ((bayer[base + sx] + bayer[base + 1]) >> 1);
				CLIP16(tmp, outG[base * 3], bits);
				tmp = bayer[base + sx + 1];
				CLIP16(tmp, outR[base * 3], bits);
				tmp = bayer[base];
				CLIP16(tmp, outB[base * 3], bits);
			}
		}
		for (i = 1; i < sy - 1; i += 2) {
			for (j = 1; j < sx - 1; j += 2) {
				base = i * sx + j;
				tmp = ((bayer[base] + bayer[base + 1 + sx]) >> 1);
				CLIP16(tmp, outG[(base)* 3], bits);
				tmp = bayer[base + sx];
				CLIP16(tmp, outR[(base)* 3], bits);
				tmp = bayer[base + 1];
				CLIP16(tmp, outB[(base)* 3], bits);
			}
		}
		break;
	case DC1394_COLOR_FILTER_BGGR:        //---------------------------------------------------------
	case DC1394_COLOR_FILTER_RGGB:
		for (i = 0; i < sy - 1; i += 2) {
			for (j = 0; j < sx - 1; j += 2) {
				base = i * sx + j;
				tmp = ((bayer[base + sx] + bayer[base + 1]) >> 1);
				CLIP16(tmp, outG[base * 3], bits);
				tmp = bayer[base + sx + 1];
				CLIP16(tmp, outR[base * 3], bits);
				tmp = bayer[base];
				CLIP16(tmp, outB[base * 3], bits);
			}
		}
		for (i = 1; i < sy - 1; i += 2) {
			for (j = 0; j < sx - 1; j += 2) {
				base = i * sx + j;
				tmp = ((bayer[base] + bayer[base + 1 + sx]) >> 1);
				CLIP16(tmp, outG[(base)* 3], bits);
				tmp = bayer[base + 1];
				CLIP16(tmp, outR[(base)* 3], bits);
				tmp = bayer[base + sx];
				CLIP16(tmp, outB[(base)* 3], bits);
			}
		}
		for (i = 0; i < sy - 1; i += 2) {
			for (j = 1; j < sx - 1; j += 2) {
				base = i * sx + j;
				tmp = ((bayer[base] + bayer[base + sx + 1]) >> 1);
				CLIP16(tmp, outG[base * 3], bits);
				tmp = bayer[base + sx];
				CLIP16(tmp, outR[base * 3], bits);
				tmp = bayer[base + 1];
				CLIP16(tmp, outB[base * 3], bits);
			}
		}
		for (i = 1; i < sy - 1; i += 2) {
			for (j = 1; j < sx - 1; j += 2) {
				base = i * sx + j;
				tmp = ((bayer[base + 1] + bayer[base + sx]) >> 1);
				CLIP16(tmp, outG[(base)* 3], bits);
				tmp = bayer[base];
				CLIP16(tmp, outR[(base)* 3], bits);
				tmp = bayer[base + 1 + sx];
				CLIP16(tmp, outB[(base)* 3], bits);
			}
		}
		break;
	}

	/* add black border */
	for (i = sx * (sy - 1) * 3; i < sx * sy * 3; i++) {
		rgb[i] = 0;
	}
	for (i = (sx - 1) * 3; i < sx * sy * 3; i += (sx - 1) * 3) {
		rgb[i++] = 0;
		rgb[i++] = 0;
		rgb[i++] = 0;
	}

	return DC1394_SUCCESS;

}

/* Variable Number of Gradients, from dcraw <http://www.cybercom.net/~dcoffin/dcraw/> */
/* Ported to libdc1394 by Frederic Devernay */

#define FORC3 for (c=0; c < 3; c++)

#define SQR(x) ((x)*(x))
#define ABS(x) (((int)(x) ^ ((int)(x) >> 31)) - ((int)(x) >> 31))
#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif
#define LIM(x,min,max) MAX(min,MIN(x,max))
#define ULIM(x,y,z) ((y) < (z) ? LIM(x,y,z) : LIM(x,z,y))
/*
In order to inline this calculation, I make the risky
assumption that all filter patterns can be described
by a repeating pattern of eight rows and two columns

Return values are either 0/1/2/3 = G/M/C/Y or 0/1/2/3 = R/G1/B/G2
*/
#define FC(row,col) \
        (filters >> ((((row) << 1 & 14) + ((col) & 1)) << 1) & 3)

/*
This algorithm is officially called:

"Interpolation using a Threshold-based variable number of gradients"

described in http://www-ise.stanford.edu/~tingchen/algodep/vargra.html

I've extended the basic idea to work with non-Bayer filter arrays.
Gradients are numbered clockwise from NW=0 to W=7.
*/
static const signed char bayervng_terms[] = {
	-2, -2, +0, -1, 0, 0x01, -2, -2, +0, +0, 1, 0x01, -2, -1, -1, +0, 0, 0x01,
	-2, -1, +0, -1, 0, 0x02, -2, -1, +0, +0, 0, 0x03, -2, -1, +0, +1, 1, 0x01,
	-2, +0, +0, -1, 0, 0x06, -2, +0, +0, +0, 1, 0x02, -2, +0, +0, +1, 0, 0x03,
	-2, +1, -1, +0, 0, 0x04, -2, +1, +0, -1, 1, 0x04, -2, +1, +0, +0, 0, 0x06,
	-2, +1, +0, +1, 0, 0x02, -2, +2, +0, +0, 1, 0x04, -2, +2, +0, +1, 0, 0x04,
	-1, -2, -1, +0, 0, 0x80, -1, -2, +0, -1, 0, 0x01, -1, -2, +1, -1, 0, 0x01,
	-1, -2, +1, +0, 1, 0x01, -1, -1, -1, +1, 0, 0x88, -1, -1, +1, -2, 0, 0x40,
	-1, -1, +1, -1, 0, 0x22, -1, -1, +1, +0, 0, 0x33, -1, -1, +1, +1, 1, 0x11,
	-1, +0, -1, +2, 0, 0x08, -1, +0, +0, -1, 0, 0x44, -1, +0, +0, +1, 0, 0x11,
	-1, +0, +1, -2, 1, 0x40, -1, +0, +1, -1, 0, 0x66, -1, +0, +1, +0, 1, 0x22,
	-1, +0, +1, +1, 0, 0x33, -1, +0, +1, +2, 1, 0x10, -1, +1, +1, -1, 1, 0x44,
	-1, +1, +1, +0, 0, 0x66, -1, +1, +1, +1, 0, 0x22, -1, +1, +1, +2, 0, 0x10,
	-1, +2, +0, +1, 0, 0x04, -1, +2, +1, +0, 1, 0x04, -1, +2, +1, +1, 0, 0x04,
	+0, -2, +0, +0, 1, 0x80, +0, -1, +0, +1, 1, 0x88, +0, -1, +1, -2, 0, 0x40,
	+0, -1, +1, +0, 0, 0x11, +0, -1, +2, -2, 0, 0x40, +0, -1, +2, -1, 0, 0x20,
	+0, -1, +2, +0, 0, 0x30, +0, -1, +2, +1, 1, 0x10, +0, +0, +0, +2, 1, 0x08,
	+0, +0, +2, -2, 1, 0x40, +0, +0, +2, -1, 0, 0x60, +0, +0, +2, +0, 1, 0x20,
	+0, +0, +2, +1, 0, 0x30, +0, +0, +2, +2, 1, 0x10, +0, +1, +1, +0, 0, 0x44,
	+0, +1, +1, +2, 0, 0x10, +0, +1, +2, -1, 1, 0x40, +0, +1, +2, +0, 0, 0x60,
	+0, +1, +2, +1, 0, 0x20, +0, +1, +2, +2, 0, 0x10, +1, -2, +1, +0, 0, 0x80,
	+1, -1, +1, +1, 0, 0x88, +1, +0, +1, +2, 0, 0x08, +1, +0, +2, -1, 0, 0x40,
	+1, +0, +2, +1, 0, 0x10
}, bayervng_chood[] = { -1, -1, -1, 0, -1, +1, 0, +1, +1, +1, +1, 0, +1, -1, 0, -1 };




/* AHD interpolation ported from dcraw to libdc1394 by Samuel Audet */
static dc1394bool_t ahd_inited = DC1394_FALSE; /* WARNING: not multi-processor safe */

#define CLIPOUT(x)        LIM(x,0,255)
#define CLIPOUT16(x,bits) LIM(x,0,((1<<bits)-1))

static const double xyz_rgb[3][3] = {                        /* XYZ from RGB */
		{ 0.412453, 0.357580, 0.180423 },
		{ 0.212671, 0.715160, 0.072169 },
		{ 0.019334, 0.119193, 0.950227 } };
static const float d65_white[3] = { 0.950456, 1, 1.088754 };

/*
Adaptive Homogeneity-Directed interpolation is based on
the work of Keigo Hirakawa, Thomas Parks, and Paul Lee.
*/
#define TS 256                /* Tile Size */

dc1394error_t
dc1394_bayer_decoding_8bit(const uint8_t * bayer, uint8_t * rgb, uint32_t sx, uint32_t sy, dc1394color_filter_t tile, dc1394bayer_method_t method)
{
	switch (method) {
	case DC1394_BAYER_METHOD_NEAREST:
		return dc1394_bayer_NearestNeighbor(bayer, rgb, sx, sy, tile);
	case DC1394_BAYER_METHOD_SIMPLE:
		return dc1394_bayer_Simple(bayer, rgb, sx, sy, tile);
	case DC1394_BAYER_METHOD_BILINEAR:
		return dc1394_bayer_Bilinear(bayer, rgb, sx, sy, tile);
	case DC1394_BAYER_METHOD_HQLINEAR:
		return dc1394_bayer_HQLinear(bayer, rgb, sx, sy, tile);
	case DC1394_BAYER_METHOD_DOWNSAMPLE:
		return dc1394_bayer_Downsample(bayer, rgb, sx, sy, tile);
	case DC1394_BAYER_METHOD_EDGESENSE:
		return dc1394_bayer_EdgeSense(bayer, rgb, sx, sy, tile);
	default:
		return DC1394_INVALID_BAYER_METHOD;
	}

}

dc1394error_t
dc1394_bayer_decoding_16bit(const uint16_t * bayer, uint16_t * rgb, uint32_t sx, uint32_t sy, dc1394color_filter_t tile, dc1394bayer_method_t method, uint32_t bits)
{
	switch (method) {
	case DC1394_BAYER_METHOD_NEAREST:
		return dc1394_bayer_NearestNeighbor_uint16(bayer, rgb, sx, sy, tile, bits);
	case DC1394_BAYER_METHOD_SIMPLE:
		return dc1394_bayer_Simple_uint16(bayer, rgb, sx, sy, tile, bits);
	case DC1394_BAYER_METHOD_BILINEAR:
		return dc1394_bayer_Bilinear_uint16(bayer, rgb, sx, sy, tile, bits);
	case DC1394_BAYER_METHOD_HQLINEAR:
		return dc1394_bayer_HQLinear_uint16(bayer, rgb, sx, sy, tile, bits);
	case DC1394_BAYER_METHOD_DOWNSAMPLE:
		return dc1394_bayer_Downsample_uint16(bayer, rgb, sx, sy, tile, bits);
	case DC1394_BAYER_METHOD_EDGESENSE:
		return dc1394_bayer_EdgeSense_uint16(bayer, rgb, sx, sy, tile, bits);

	default:
		return DC1394_INVALID_BAYER_METHOD;
	}

}

#if 0
dc1394error_t
Adapt_buffer_bayer(dc1394video_frame_t *in, dc1394video_frame_t *out, dc1394bayer_method_t method)
{
	uint32_t bpp;

	// conversions will halve the buffer size if the method is DOWNSAMPLE:
	out->size[0] = in->size[0];
	out->size[1] = in->size[1];
	if (method == DC1394_BAYER_METHOD_DOWNSAMPLE) {
		out->size[0] /= 2; // ODD SIZE CASES NOT TAKEN INTO ACCOUNT
		out->size[1] /= 2;
	}

	// as a convention we divide the image position by two in the case of a DOWNSAMPLE:
	out->position[0] = in->position[0];
	out->position[1] = in->position[1];
	if (method == DC1394_BAYER_METHOD_DOWNSAMPLE) {
		out->position[0] /= 2;
		out->position[1] /= 2;
	}

	// the destination color coding is ALWAYS RGB. Set this.
	if ((in->color_coding == DC1394_COLOR_CODING_RAW16) ||
		(in->color_coding == DC1394_COLOR_CODING_MONO16))
		out->color_coding = DC1394_COLOR_CODING_RGB16;
	else
		out->color_coding = DC1394_COLOR_CODING_RGB8;

	// keep the color filter value in all cases. If the format is not raw it will not be further used anyway
	out->color_filter = in->color_filter;

	// The output is never YUV, hence nothing to do about YUV byte order

	// bit depth is conserved for 16 bit and set to 8bit for 8bit:
	if ((in->color_coding == DC1394_COLOR_CODING_RAW16) ||
		(in->color_coding == DC1394_COLOR_CODING_MONO16))
		out->data_depth = in->data_depth;
	else
		out->data_depth = 8;

	// don't know what to do with stride... >>>> TODO: STRIDE SHOULD BE TAKEN INTO ACCOUNT... <<<<
	// out->stride=??

	// the video mode should not change. Color coding and other stuff can be accessed in specific fields of this struct
	out->video_mode = in->video_mode;

	// padding is kept:
	out->padding_bytes = in->padding_bytes;

	// image bytes changes:    >>>> TODO: STRIDE SHOULD BE TAKEN INTO ACCOUNT... <<<<
	dc1394_get_color_coding_bit_size(out->color_coding, &bpp);
	out->image_bytes = (out->size[0] * out->size[1] * bpp) / 8;

	// total is image_bytes + padding_bytes
	out->total_bytes = out->image_bytes + out->padding_bytes;

	// bytes-per-packet and packets_per_frame are internal data that can be kept as is.
	out->packet_size = in->packet_size;
	out->packets_per_frame = in->packets_per_frame;

	// timestamp, frame_behind, id and camera are copied too:
	out->timestamp = in->timestamp;
	out->frames_behind = in->frames_behind;
	out->camera = in->camera;
	out->id = in->id;

	// verify memory allocation:
	if (out->total_bytes>out->allocated_image_bytes) {
		free(out->image);
		out->image = (uint8_t*)malloc(out->total_bytes*sizeof(uint8_t));
		if (out->image)
			out->allocated_image_bytes = out->total_bytes*sizeof(uint8_t);
		else
			out->allocated_image_bytes = 0;
	}

	// Copy padding bytes:
	if (out->image)
		memcpy(&(out->image[out->image_bytes]), &(in->image[in->image_bytes]), out->padding_bytes);

	out->little_endian = 0; // not used before 1.32 is out.
	out->data_in_padding = 0; // not used before 1.32 is out.

	if (out->image)
		return DC1394_SUCCESS;

	return DC1394_MEMORY_ALLOCATION_FAILURE;
}

dc1394error_t
dc1394_debayer_frames(dc1394video_frame_t *in, dc1394video_frame_t *out, dc1394bayer_method_t method)
{
	if ((method<DC1394_BAYER_METHOD_MIN) || (method>DC1394_BAYER_METHOD_MAX))
		return DC1394_INVALID_BAYER_METHOD;

	switch (in->color_coding) {
	case DC1394_COLOR_CODING_RAW8:
	case DC1394_COLOR_CODING_MONO8:

		if (DC1394_SUCCESS != Adapt_buffer_bayer(in, out, method))
			return DC1394_MEMORY_ALLOCATION_FAILURE;

		switch (method) {
		case DC1394_BAYER_METHOD_NEAREST:
			return dc1394_bayer_NearestNeighbor(in->image, out->image, in->size[0], in->size[1], in->color_filter);
		case DC1394_BAYER_METHOD_SIMPLE:
			return dc1394_bayer_Simple(in->image, out->image, in->size[0], in->size[1], in->color_filter);
		case DC1394_BAYER_METHOD_BILINEAR:
			return dc1394_bayer_Bilinear(in->image, out->image, in->size[0], in->size[1], in->color_filter);
		case DC1394_BAYER_METHOD_HQLINEAR:
			return dc1394_bayer_HQLinear(in->image, out->image, in->size[0], in->size[1], in->color_filter);
		case DC1394_BAYER_METHOD_DOWNSAMPLE:
			return dc1394_bayer_Downsample(in->image, out->image, in->size[0], in->size[1], in->color_filter);
		case DC1394_BAYER_METHOD_EDGESENSE:
			return dc1394_bayer_EdgeSense(in->image, out->image, in->size[0], in->size[1], in->color_filter);
		case DC1394_BAYER_METHOD_VNG:
			return dc1394_bayer_VNG(in->image, out->image, in->size[0], in->size[1], in->color_filter);
		case DC1394_BAYER_METHOD_AHD:
			return dc1394_bayer_AHD(in->image, out->image, in->size[0], in->size[1], in->color_filter);
		}
		break;
	case DC1394_COLOR_CODING_MONO16:
	case DC1394_COLOR_CODING_RAW16:

		if (DC1394_SUCCESS != Adapt_buffer_bayer(in, out, method))
			return DC1394_MEMORY_ALLOCATION_FAILURE;

		switch (method) {
		case DC1394_BAYER_METHOD_NEAREST:
			return dc1394_bayer_NearestNeighbor_uint16((uint16_t*)in->image, (uint16_t*)out->image, in->size[0], in->size[1], in->color_filter, in->data_depth);
		case DC1394_BAYER_METHOD_SIMPLE:
			return dc1394_bayer_Simple_uint16((uint16_t*)in->image, (uint16_t*)out->image, in->size[0], in->size[1], in->color_filter, in->data_depth);
		case DC1394_BAYER_METHOD_BILINEAR:
			return dc1394_bayer_Bilinear_uint16((uint16_t*)in->image, (uint16_t*)out->image, in->size[0], in->size[1], in->color_filter, in->data_depth);
		case DC1394_BAYER_METHOD_HQLINEAR:
			return dc1394_bayer_HQLinear_uint16((uint16_t*)in->image, (uint16_t*)out->image, in->size[0], in->size[1], in->color_filter, in->data_depth);
		case DC1394_BAYER_METHOD_DOWNSAMPLE:
			return dc1394_bayer_Downsample_uint16((uint16_t*)in->image, (uint16_t*)out->image, in->size[0], in->size[1], in->color_filter, in->data_depth);
		case DC1394_BAYER_METHOD_EDGESENSE:
			return dc1394_bayer_EdgeSense_uint16((uint16_t*)in->image, (uint16_t*)out->image, in->size[0], in->size[1], in->color_filter, in->data_depth);
		case DC1394_BAYER_METHOD_VNG:
			return dc1394_bayer_VNG_uint16((uint16_t*)in->image, (uint16_t*)out->image, in->size[0], in->size[1], in->color_filter, in->data_depth);
		case DC1394_BAYER_METHOD_AHD:
			return dc1394_bayer_AHD_uint16((uint16_t*)in->image, (uint16_t*)out->image, in->size[0], in->size[1], in->color_filter, in->data_depth);
		}
		break;
	default:
		return DC1394_FUNCTION_NOT_SUPPORTED;
	}

	return DC1394_SUCCESS;
}
#endif