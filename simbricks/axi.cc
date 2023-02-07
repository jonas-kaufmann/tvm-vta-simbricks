/*
 * Copyright 2023 Max Planck Institute for Software Systems, and
 * National University of Singapore
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "axi.h"

void AXIReader::do_read(uint64_t addr, void *buf, size_t len)
{
}

AXIReader::AXIReader(AXIChannelReadAddr &addrP_, AXIChannelReadData &dataP_)
  : addrP(addrP_), dataP(dataP_)
{
}

void AXIReader::step()
{
  /* FIXME: TODO */
}


void AXIWriter::do_write(uint64_t addr, const void *buf, size_t len)
{
}

AXIWriter::AXIWriter(AXIChannelWriteAddr &addrP_, AXIChannelWriteData &dataP_,
    AXIChannelWriteResp &respP_)
  : addrP(addrP_), dataP(dataP_), respP(respP_)
{
}

void AXIWriter::step()
{
  /* FIXME: TODO */
}