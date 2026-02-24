//----------------------------------------------------------------------------
// Anti-Grain Geometry - Version 2.4
// Copyright (C) 2002-2005 Maxim Shemanarev (http://www.antigrain.com)
//
// Permission to copy, use, modify, sell and distribute this software
// is granted provided this copyright notice appears in all copies.
// This software is provided "as is" without express or implied
// warranty, and with no claim as to its suitability for any purpose.
//
//----------------------------------------------------------------------------
// Contact: mcseem@antigrain.com
//          mcseemagg@yahoo.com
//          http://www.antigrain.com
//----------------------------------------------------------------------------
//
// Stack Blur (grayscale only).
//
// Based on the Stack Blur algorithm by Mario Klingemann:
//   http://www.quasimondo.com/StackBlurForCanvas/StackBlurDemo.html
//
// The algorithm performs two separable 1-D passes (horizontal then vertical).
// For radius r the effective sigma is approximately r/sqrt(3), giving a
// visually smooth result comparable to a Gaussian blur.
//
// Img requirements (e.g. agg::pixfmt_gray8):
//   unsigned       width()  const
//   unsigned       height() const
//   color_type     pixel(int x, int y) const
//   void           copy_pixel(int x, int y, const color_type&)
//   color_type::value_type v  -- gray value
//   color_type(unsigned v_)   -- constructible from unsigned
//
//----------------------------------------------------------------------------

#ifndef AGG_BLUR_INCLUDED
#define AGG_BLUR_INCLUDED

#include "agg_array.h"

namespace agg
{

    //------------------------------------------------------------------------
    // stack_blur_gray8
    //
    // Performs an in-place stack blur on a grayscale image.
    //   img -- pixel-format object wrapping the buffer
    //   rx  -- horizontal blur radius (0 = no horizontal blur)
    //   ry  -- vertical   blur radius (0 = no vertical   blur)
    //------------------------------------------------------------------------
    template<class Img>
    void stack_blur_gray8(Img& img, unsigned rx, unsigned ry)
    {
        typedef typename Img::color_type color_type;
        typedef pod_vector<color_type>   stack_type;

        unsigned x, y, xp, yp, i;
        unsigned stack_ptr, stack_start;
        unsigned sum, sum_in, sum_out;
        color_type pix;

        const unsigned w  = img.width();
        const unsigned h  = img.height();
        const unsigned wm = w - 1;
        const unsigned hm = h - 1;

        // ---------------------------------------------------------------
        // Horizontal pass
        // ---------------------------------------------------------------
        if (rx > 0)
        {
            if (rx > 254) rx = 254;
            const unsigned div   = rx * 2 + 1;
            const unsigned wsum  = (rx + 1) * (rx + 1);  // sum of triangular weights
            const unsigned whalf = wsum >> 1;             // for rounding

            stack_type stack(div);

            for (y = 0; y < h; y++)
            {
                sum = sum_in = sum_out = 0;

                // Load the stack: left half (border-clamped) + right half
                pix = img.pixel(0, y);
                for (i = 0; i <= rx; i++)
                {
                    stack[i]  = pix;
                    sum      += pix.v * (i + 1);
                    sum_out  += pix.v;
                }
                for (i = 1; i <= rx; i++)
                {
                    pix       = img.pixel(i > wm ? wm : i, y);
                    stack[rx + i] = pix;
                    sum      += pix.v * (rx + 1 - i);
                    sum_in   += pix.v;
                }

                stack_ptr = rx;
                xp        = rx;
                if (xp > wm) xp = wm;
                pix = img.pixel(xp, y);

                for (x = 0; x < w; x++)
                {
                    // Write blurred pixel
                    color_type out_pix(static_cast<unsigned>((sum + whalf) / wsum));
                    img.copy_pixel(x, y, out_pix);

                    sum -= sum_out;

                    // Position in the circular stack that exits sum_out
                    stack_start = stack_ptr + div - rx;
                    if (stack_start >= div) stack_start -= div;

                    sum_out -= stack[stack_start].v;

                    // Advance the look-ahead pointer and fetch new pixel
                    if (++xp > wm) xp = wm;
                    pix = img.pixel(xp, y);

                    stack[stack_start]  = pix;
                    sum_in             += pix.v;
                    sum                += sum_in;

                    if (++stack_ptr >= div) stack_ptr = 0;
                    sum_out += stack[stack_ptr].v;
                    sum_in  -= stack[stack_ptr].v;
                }
            }
        }

        // ---------------------------------------------------------------
        // Vertical pass
        // ---------------------------------------------------------------
        if (ry > 0)
        {
            if (ry > 254) ry = 254;
            const unsigned div   = ry * 2 + 1;
            const unsigned wsum  = (ry + 1) * (ry + 1);
            const unsigned whalf = wsum >> 1;

            stack_type stack(div);

            for (x = 0; x < w; x++)
            {
                sum = sum_in = sum_out = 0;

                pix = img.pixel(x, 0);
                for (i = 0; i <= ry; i++)
                {
                    stack[i]  = pix;
                    sum      += pix.v * (i + 1);
                    sum_out  += pix.v;
                }
                for (i = 1; i <= ry; i++)
                {
                    pix           = img.pixel(x, i > hm ? hm : i);
                    stack[ry + i] = pix;
                    sum          += pix.v * (ry + 1 - i);
                    sum_in       += pix.v;
                }

                stack_ptr = ry;
                yp        = ry;
                if (yp > hm) yp = hm;
                pix = img.pixel(x, yp);

                for (y = 0; y < h; y++)
                {
                    color_type out_pix(static_cast<unsigned>((sum + whalf) / wsum));
                    img.copy_pixel(x, y, out_pix);

                    sum -= sum_out;

                    stack_start = stack_ptr + div - ry;
                    if (stack_start >= div) stack_start -= div;

                    sum_out -= stack[stack_start].v;

                    if (++yp > hm) yp = hm;
                    pix = img.pixel(x, yp);

                    stack[stack_start]  = pix;
                    sum_in             += pix.v;
                    sum                += sum_in;

                    if (++stack_ptr >= div) stack_ptr = 0;
                    sum_out += stack[stack_ptr].v;
                    sum_in  -= stack[stack_ptr].v;
                }
            }
        }
    }

} // namespace agg

#endif // AGG_BLUR_INCLUDED
