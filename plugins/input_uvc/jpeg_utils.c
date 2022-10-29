/*******************************************************************************
# Linux-UVC streaming input-plugin for MJPG-streamer                           #
#                                                                              #
# This package work with the Logitech UVC based webcams with the mjpeg feature #
#                                                                              #
#   Orginally Copyright (C) 2005 2006 Laurent Pinchart &&  Michel Xhaard       #
#   Modifications Copyright (C) 2006  Gabriel A. Devenyi                       #
#   Modifications Copyright (C) 2007  Tom Stöveken                             #
#                                                                              #
# This program is free software; you can redistribute it and/or modify         #
# it under the terms of the GNU General Public License as published by         #
# the Free Software Foundation; either version 2 of the License, or            #
# (at your option) any later version.                                          #
#                                                                              #
# This program is distributed in the hope that it will be useful,              #
# but WITHOUT ANY WARRANTY; without even the implied warranty of               #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                #
# GNU General Public License for more details.                                 #
#                                                                              #
# You should have received a copy of the GNU General Public License            #
# along with this program; if not, write to the Free Software                  #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA    #
#                                                                              #
*******************************************************************************/

#include <stdio.h>
#include <jpeglib.h>
#include <stdlib.h>

#include "v4l2uvc.h"

#define OUTPUT_BUF_SIZE  4096

typedef struct {
    struct jpeg_destination_mgr pub; /* public fields */

    JOCTET * buffer;    /* start of buffer */

    unsigned char *outbuffer;
    int outbuffer_size;
    unsigned char *outbuffer_cursor;
    int *written;

} mjpg_destination_mgr;

typedef mjpg_destination_mgr * mjpg_dest_ptr;

/******************************************************************************
Description.:
Input Value.:
Return Value:
******************************************************************************/
METHODDEF(void) init_destination(j_compress_ptr cinfo)
{
    mjpg_dest_ptr dest = (mjpg_dest_ptr) cinfo->dest;

    /* Allocate the output buffer --- it will be released when done with image */
    dest->buffer = (JOCTET *)(*cinfo->mem->alloc_small)((j_common_ptr) cinfo, JPOOL_IMAGE, OUTPUT_BUF_SIZE * sizeof(JOCTET));

    *(dest->written) = 0;

    dest->pub.next_output_byte = dest->buffer;
    dest->pub.free_in_buffer = OUTPUT_BUF_SIZE;
}

/******************************************************************************
Description.: called whenever local jpeg buffer fills up
Input Value.:
Return Value:
******************************************************************************/
METHODDEF(boolean) empty_output_buffer(j_compress_ptr cinfo)
{
    mjpg_dest_ptr dest = (mjpg_dest_ptr) cinfo->dest;

    memcpy(dest->outbuffer_cursor, dest->buffer, OUTPUT_BUF_SIZE);
    dest->outbuffer_cursor += OUTPUT_BUF_SIZE;
    *(dest->written) += OUTPUT_BUF_SIZE;

    dest->pub.next_output_byte = dest->buffer;
    dest->pub.free_in_buffer = OUTPUT_BUF_SIZE;

    return TRUE;
}

/******************************************************************************
Description.: called by jpeg_finish_compress after all data has been written.
              Usually needs to flush buffer.
Input Value.:
Return Value:
******************************************************************************/
METHODDEF(void) term_destination(j_compress_ptr cinfo)
{
    mjpg_dest_ptr dest = (mjpg_dest_ptr) cinfo->dest;
    size_t datacount = OUTPUT_BUF_SIZE - dest->pub.free_in_buffer;

    /* Write any data remaining in the buffer */
    memcpy(dest->outbuffer_cursor, dest->buffer, datacount);
    dest->outbuffer_cursor += datacount;
    *(dest->written) += datacount;
}

/******************************************************************************
Description.: Prepare for output to a stdio stream.
Input Value.: buffer is the already allocated buffer memory that will hold
              the compressed picture. "size" is the size in bytes.
Return Value: -
******************************************************************************/
GLOBAL(void) dest_buffer(j_compress_ptr cinfo, unsigned char *buffer, int size, int *written)
{
    mjpg_dest_ptr dest;

    if(cinfo->dest == NULL) {
        cinfo->dest = (struct jpeg_destination_mgr *)(*cinfo->mem->alloc_small)((j_common_ptr) cinfo, JPOOL_PERMANENT, sizeof(mjpg_destination_mgr));
    }

    dest = (mjpg_dest_ptr) cinfo->dest;
    dest->pub.init_destination = init_destination;
    dest->pub.empty_output_buffer = empty_output_buffer;
    dest->pub.term_destination = term_destination;
    dest->outbuffer = buffer;
    dest->outbuffer_size = size;
    dest->outbuffer_cursor = buffer;
    dest->written = written;
}


static void lcd_point_rgb(int x, int y, int r, int g, int b, struct fb_param *fp)
{
	unsigned int v = 0;
	unsigned char *p = fp->fb_addr;
	int bpp = fp->fb_bpp;

	if ((x < 0 || x >= fp->fb_xres) || (y < 0 || y >= fp->fb_yres)) {
		/* out of LCD display area */
		return;
	}

	switch (bpp) {
		case 16:
			v |= ((b >> 3) & 0x1f);
			v |= ((g >> 2) & 0x3f) << 5;
			v |= ((r >> 3) & 0x1f) << 11;

			p += ((y * fp->fb_xres) + x) * 2;
			*p++ = v;
			*p++ = v >> 8;
			break;

		case 18:
			v |= ((b >> 2) & 0x3f);
			v |= ((g >> 2) & 0x3f) << 6;
			v |= ((r >> 2) & 0x3f) << 12;

			p += ((y * fp->fb_xres) + x) * 3;
			*p++ = v;
			*p++ = v >> 8;
			*p++ = v >> 16;
			break;

		case 24:
			p += ((y * fp->fb_xres) + x) * 3;
			*p++ = b;
			*p++ = g;
			*p++ = r;
			break;

		case 32:
			p += ((y * fp->fb_xres) + x) * 4;
			*p++ = b;
			*p++ = g;
			*p++ = r;
			break;
	}
}

/******************************************************************************
Description.: yuv2jpeg function is based on compress_yuv_to_jpeg written by
              Gabriel A. Devenyi.
              It uses the destination manager implemented above to compress
              YUYV data to JPEG. Most other implementations use the
              "jpeg_stdio_dest" from libjpeg, which can not store compressed
              pictures to memory instead of a file.
Input Value.: video structure from v4l2uvc.c/h, destination buffer and buffersize
              the buffer must be large enough, no error/size checking is done!
Return Value: the buffer will contain the compressed data
******************************************************************************/
int compress_yuv_to_jpeg(struct vdIn *vd, unsigned char *buffer, int size, int quality, int fmt, struct fb_param *fp)
{
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    JSAMPROW row_pointer[1];
    unsigned char *line_buffer, *yuyv, *frame_buffer, *fb;
    int z;
    static int written;

    line_buffer = calloc(vd->width * 3, 1);
    frame_buffer = calloc(vd->height * vd->width * 3, 1);
    yuyv = vd->framebuffer;

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    /* jpeg_stdio_dest (&cinfo, file); */
    dest_buffer(&cinfo, buffer, size, &written);

    cinfo.image_width = vd->width;
    cinfo.image_height = vd->height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);

    jpeg_start_compress(&cinfo, TRUE);

    z = 0;
    fb = frame_buffer;
    while(cinfo.next_scanline < vd->height) {
        int x;
        unsigned char *ptr = line_buffer;

        for(x = 0; x < vd->width; x++) {
            int r, g, b;
            int y, u, v;
            y=u=v=0;

            switch(fmt) {
            case V4L2_PIX_FMT_YUYV:
                if(!z)
                    y = yuyv[0] << 8;
                else
                    y = yuyv[2] << 8;
                u = yuyv[1] - 128;
                v = yuyv[3] - 128;
                break;
            case V4L2_PIX_FMT_YVYU:
                if(!z)
                    y = yuyv[0] << 8;
                else
                    y = yuyv[2] << 8;
                v = yuyv[1] - 128;
                u = yuyv[3] - 128;
                break;
            case V4L2_PIX_FMT_UYVY:
                if(!z)
                    y = yuyv[1] << 8;
                else
                    y = yuyv[3] << 8;
                u = yuyv[0] - 128;
                v = yuyv[2] - 128;
                break;
            case V4L2_PIX_FMT_VYUY:
                if(!z)
                    y = yuyv[1] << 8;
                else
                    y = yuyv[3] << 8;
                v = yuyv[0] - 128;
                u = yuyv[2] - 128;
                break;
            }    
            
            r = (y + (359 * v)) >> 8;
            g = (y - (88 * u) - (183 * v)) >> 8;
            b = (y + (454 * u)) >> 8;

            *(ptr++) = (r > 255) ? 255 : ((r < 0) ? 0 : r);
            *(ptr++) = (g > 255) ? 255 : ((g < 0) ? 0 : g);
            *(ptr++) = (b > 255) ? 255 : ((b < 0) ? 0 : b);

            if(z++) {
                z = 0;
                yuyv += 4;
            }
        }
        // copy one line rgb to frame buffer
        memcpy(fb, line_buffer, vd->width * 3);
        fb += vd->width * 3;
        row_pointer[0] = line_buffer;
        jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    free(line_buffer);
    int x,y;
    fb = frame_buffer;
    for (y=0; y<vd->height; y++)
        for(x=0; x<vd->width; x++) {
            lcd_point_rgb(x, y, *(fb), *(fb+1), *(fb+2), fp);
            fb += 3;
        }
    free(frame_buffer);
    return (written);
}

