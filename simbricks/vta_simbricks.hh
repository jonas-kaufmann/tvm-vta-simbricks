/*
 * Copyright 2024 Max Planck Institute for Software Systems, and
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
#pragma once

#include <simbricks/rtl/axi/axi.hh>

#include "obj_dir/VVTAShell.h"

class VTAMemReader : public AXIReader {
 public:
  explicit VTAMemReader(VVTAShell &top) : AXIReader{} {
    // set up address port
    addrP_.addr_bits = 32;
    addrP_.id_bits = 8;
    addrP_.user_bits = 5;

    addrP_.ready = &top.io_mem_ar_ready;
    addrP_.valid = &top.io_mem_ar_valid;
    addrP_.addr = &top.io_mem_ar_bits_addr;
    addrP_.id = &top.io_mem_ar_bits_id;
    addrP_.user = &top.io_mem_ar_bits_user;
    addrP_.len = &top.io_mem_ar_bits_len;
    addrP_.size = &top.io_mem_ar_bits_size;
    addrP_.burst = &top.io_mem_ar_bits_burst;
    addrP_.lock = &top.io_mem_ar_bits_lock;
    addrP_.cache = &top.io_mem_ar_bits_cache;
    addrP_.prot = &top.io_mem_ar_bits_prot;
    addrP_.qos = &top.io_mem_ar_bits_qos;
    addrP_.region = &top.io_mem_ar_bits_region;

    // set up data port
    dataP_.data_bits = 64;
    dataP_.id_bits = 8;
    dataP_.user_bits = 5;

    dataP_.ready = &top.io_mem_r_ready;
    dataP_.valid = &top.io_mem_r_valid;
    dataP_.data = &top.io_mem_r_bits_data;
    dataP_.resp = &top.io_mem_r_bits_resp;
    dataP_.last = &top.io_mem_r_bits_last;
    dataP_.id = &top.io_mem_r_bits_id;
  }

 protected:
  void doRead(AXIOperationT *axi_op) override;
};

class VTAMemWriter : public AXIWriter {
 public:
  explicit VTAMemWriter(VVTAShell &top) : AXIWriter{} {
    // set up address port
    addrP_.addr_bits = 32;
    addrP_.id_bits = 8;
    addrP_.user_bits = 5;

    addrP_.ready = &top.io_mem_aw_ready;
    addrP_.valid = &top.io_mem_aw_valid;
    addrP_.addr = &top.io_mem_aw_bits_addr;
    addrP_.id = &top.io_mem_aw_bits_id;
    addrP_.user = &top.io_mem_aw_bits_user;
    addrP_.len = &top.io_mem_aw_bits_len;
    addrP_.size = &top.io_mem_aw_bits_size;
    addrP_.burst = &top.io_mem_aw_bits_burst;
    addrP_.lock = &top.io_mem_aw_bits_lock;
    addrP_.cache = &top.io_mem_aw_bits_cache;
    addrP_.prot = &top.io_mem_aw_bits_prot;
    addrP_.qos = &top.io_mem_aw_bits_qos;
    addrP_.region = &top.io_mem_aw_bits_region;

    // set up data port
    dataP_.data_bits = 64;
    dataP_.id_bits = 8;
    dataP_.user_bits = 5;

    dataP_.ready = &top.io_mem_w_ready;
    dataP_.valid = &top.io_mem_w_valid;
    dataP_.data = &top.io_mem_w_bits_data;
    dataP_.strb = &top.io_mem_w_bits_strb;
    dataP_.last = &top.io_mem_w_bits_last;

    // set up response port
    respP_.id_bits = 8;
    respP_.user_bits = 5;

    respP_.ready = &top.io_mem_b_ready;
    respP_.valid = &top.io_mem_b_valid;
    respP_.resp = &top.io_mem_b_bits_resp;
    respP_.id = &top.io_mem_b_bits_id;
    respP_.user = &top.io_mem_b_bits_user;
  }

 protected:
  void doWrite(AXIOperationT *axi_op) override;
};