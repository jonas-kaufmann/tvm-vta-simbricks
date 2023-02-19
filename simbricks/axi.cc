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

#include <iostream>

#include "axi.h"

//#define AXI_DEBUG 1

extern uint64_t main_time;

AXIReader::AXIReader(AXIChannelReadAddr &addrP_, AXIChannelReadData &dataP_)
  : addrP(addrP_), dataP(dataP_), curOp(nullptr)
{
}

void AXIReader::step()
{
  *addrP.ready = 1;
  if (*addrP.valid) {
    uint64_t id = 0;
    memcpy(&id, addrP.id, (addrP.id_bits + 7) / 8);

    uint64_t addr = 0;
    memcpy(&addr, addrP.addr, (addrP.addr_bits + 7) / 8);

    AXIReadOp *op = new AXIReadOp(
        addr, ((uint64_t) *addrP.len + 1) * ((dataP.data_bits + 7) / 8), id);
#ifdef AXI_DEBUG
    std::cout << main_time << " AXI R: new op=" << op << " addr="
              << op->addr << " len=" << op->len << " id=" << op->id
              << std::endl;
#endif
    doRead(op);
  }

  size_t step = (dataP.data_bits + 7) / 8;
  if (!curOp && !pending.empty()) {
    curOp = pending.front();
    curOff = 0;
    pending.pop_front();
#ifdef AXI_DEBUG
    std::cout << main_time << " AXI R: starting response op=" << curOp
              << " ready=" << (unsigned) *dataP.ready << " id=" << curOp->id
              << std::endl;
#endif

    memcpy(dataP.data, curOp->buf, step);
    memcpy(dataP.id, &curOp->id, (dataP.id_bits + 7) / 8);
    *dataP.valid = 1;

    curOff += step;
    *dataP.last = (curOff == curOp->len);

    if (*dataP.last) {
#ifdef AXI_DEBUG
      std::cout << main_time << " AXI R: completed op=" << curOp << " id="
                << curOp->id << std::endl;
  #endif
      delete curOp;
      curOp = nullptr;
    }
  } else if (curOp && *dataP.ready) {
#ifdef AXI_DEBUG
    std::cout << main_time << " AXI R: step op=" << curOp << " off="
              << curOff << " id=" << curOp->id << std::endl;
#endif
    memcpy(dataP.data, curOp->buf + curOff, step);

    curOff += step;
    *dataP.last = (curOff == curOp->len);
    if (*dataP.last) {
#ifdef AXI_DEBUG
      std::cout << main_time << " AXI R: completed op=" << curOp << " id="
                << curOp->id << std::endl;
#endif
      delete curOp;
      curOp = nullptr;
    }
  } else if (!curOp) {
    *dataP.valid = 0;
  }

}

void AXIReader::readDone(AXIReadOp *op)
{
#ifdef AXI_DEBUG
  std::cout << main_time << " AXI R: enqueue op=" << op << std::endl;
  std::cout << "    ";
  for (size_t i = 0; i < op->len; i++) {
    std::cout << (unsigned) op->buf[i] << " ";
  }
  std::cout << std::endl;
#endif
  pending.push_back(op);
}


AXIWriter::AXIWriter(AXIChannelWriteAddr &addrP_, AXIChannelWriteData &dataP_,
    AXIChannelWriteResp &respP_)
  : addrP(addrP_), dataP(dataP_), respP(respP_), complOp(nullptr)
{
}

void AXIWriter::step()
{
  if (complOp && (*respP.ready || complWasReady)) {
#ifdef AXI_DEBUG
    std::cout << main_time << " AXI W: complete op=" << complOp << std::endl;
#endif
    delete complOp;
    complOp = nullptr;
    *respP.valid = 0;
  }

  if (!complOp && !completed.empty()) {
    complOp = completed.front();
    completed.pop_front();

#ifdef AXI_DEBUG
    std::cout << main_time << " AXI W: issuing completion op=" << complOp
              << std::endl;
#endif

    memcpy(respP.id, &complOp->id, (respP.id_bits + 7) / 8);
    *respP.valid = 1;
    complWasReady = *respP.ready;
  }


  *addrP.ready = 1;
  if (*addrP.valid) {
    uint64_t id = 0;
    memcpy(&id, addrP.id, (addrP.id_bits + 7) / 8);

    uint64_t addr = 0;
    memcpy(&addr, addrP.addr, (addrP.addr_bits + 7) / 8);

    AXIReadOp *op = new AXIReadOp(
        addr, ((uint64_t) *addrP.len + 1) * ((dataP.data_bits + 7) / 8), id);
#ifdef AXI_DEBUG
    std::cout << main_time << " AXI W: new op=" << op << " addr="
              << op->addr << " len=" << op->len << " id=" << op->id
              << std::endl;
#endif
    if (pending.find(id) != pending.end()) {
      std::cerr << "AXI W id " << id << " is already pending" << std::endl;
      abort();
    }
    pending[id] = op;
  }

  *dataP.ready = 1;
  if (*dataP.valid) {
    uint64_t id = 0;
    memcpy(&id, dataP.id, (dataP.id_bits + 7) / 8);

    auto it = pending.find(id);
    if (it == pending.end()) {
      std::cerr << "AXI W data " << id << " is unknown" << std::endl;
      abort();
    }

    AXIReadOp *op = it->second;

#ifdef AXI_DEBUG
    std::cout << main_time << " AXI W: data id=" << id << " op=" << op
              << " last=" << (unsigned) *dataP.last << std::endl;
#endif

    size_t step = (dataP.data_bits + 7) / 8;
    memcpy(op->buf + op->off, dataP.data, step);
    op->off += step;
    if (op->off > op->len) {
      std::cerr << "AXI W operation too long?" << std::endl;
      abort();
    } else if (op->off == op->len) {
      if (!*dataP.last) {
        std::cerr << "AXI W operation is done but last is not set?"
                  << std::endl;
        abort();
      }

      pending.erase(it);
      doWrite(op);
    }
  }
}

void AXIWriter::writeDone(AXIReadOp *op)
{
#ifdef AXI_DEBUG
  std::cout << main_time << " AXI W: completed write for op=" << op
            << std::endl;
#endif
  completed.push_back(op);
}