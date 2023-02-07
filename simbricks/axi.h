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

#include <verilated.h>

struct AXIChannelReadAddr {
    // Signal widths
    size_t addr_bits;
    size_t id_bits;
    size_t user_bits;

    // Signals
    CData *ready;   // <-
    CData *valid;   // ->

    void *addr;    // ->
    void *id;      // ->
    void *user;    // ->
    CData *len;    // ->
    CData *size;   // ->
    CData *burst;  // ->
    CData *lock;   // ->
    CData *cache;  // ->
    CData *prot;   // ->
    CData *qos;    // ->
    CData *region; // ->
};

struct AXIChannelReadData {
    // Signal widths
    size_t data_bits;
    size_t id_bits;
    size_t user_bits;

    // Signals
    CData *ready;   // ->
    CData *valid;   // <-

    CData *resp;   // <-
    void *data;    // <-
    void *id;      // <-
    void *user;    // <-
    CData *last;   // <-
};


struct AXIChannelWriteAddr {
    // Signal widths
    size_t addr_bits;
    size_t id_bits;
    size_t user_bits;

    // Signals
    CData *ready;   // <-
    CData *valid;   // ->

    void *addr;    // ->
    void *id;      // ->
    void *user;    // ->
    CData *len;    // ->
    CData *size;   // ->
    CData *burst;  // ->
    CData *lock;   // ->
    CData *cache;  // ->
    CData *prot;   // ->
    CData *qos;    // ->
    CData *region; // ->
};

struct AXIChannelWriteData {
    // Signal widths
    size_t data_bits;
    size_t id_bits;
    size_t user_bits;

    // Signals
    CData *ready;   // <-
    CData *valid;   // ->

    void *data;    // ->
    void *id;      // ->
    void *user;    // ->
    void *strb;    // ->
    CData *last;   // ->
};

struct AXIChannelWriteResp {
    // Signal widths
    size_t id_bits;
    size_t user_bits;

    // Signals
    CData *ready;   // ->
    CData *valid;   // <-

    CData *resp;   // ->
    void *id;      // ->
    void *user;    // ->
};

class AXIReader {
  protected:
    AXIChannelReadAddr &addrP;
    AXIChannelReadData &dataP;

    virtual void do_read(uint64_t addr, void *buf, size_t len);
  public:
    AXIReader(AXIChannelReadAddr &addrP_, AXIChannelReadData &dataP_);
    void step();
};

class AXIWriter {
  protected:
    AXIChannelWriteAddr &addrP;
    AXIChannelWriteData &dataP;
    AXIChannelWriteResp &respP;

    virtual void do_write(uint64_t addr, const void *buf, size_t len);
  public:
    AXIWriter(AXIChannelWriteAddr &addrP_, AXIChannelWriteData &dataP_,
        AXIChannelWriteResp &respP_);
    void step();
};