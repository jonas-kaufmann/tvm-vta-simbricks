/*
 * Copyright 2021 Max Planck Institute for Software Systems, and
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

#include <signal.h>
#include <verilated.h>

#include <deque>
#include <iostream>
#include <set>
#ifdef TRACE_ENABLED
#include <verilated_vcd_c.h>
#endif

#include <simbricks/base/cxxatomicfix.h>
extern "C" {
#include <simbricks/nicif/nicif.h>
}

#include "obj_dir/VVTAShell.h"
#include "axi.h"

struct DMAOp;

static uint64_t clock_period = 10 * 1000ULL;  // 10ns -> 100MHz
static size_t dev_mem_size = 1024 * 1024 * 1024;

static volatile int exiting = 0;
static bool terminated = false;
uint64_t main_time = 0;
static struct SimbricksNicIf nicif;

static void *dev_mem;

#ifdef TRACE_ENABLED
static VerilatedVcdC *trace;
#endif

static volatile union SimbricksProtoPcieD2H *d2h_alloc(void);

static void sigint_handler(int dummy) {
  exiting = 1;
}

static void sigusr1_handler(int dummy) {
  fprintf(stderr, "main_time = %lu\n", main_time);
}

double sc_time_stamp() {
  return main_time;
}

static void reset_inputs(VVTAShell *top) {
  top->clock = 0;
  top->reset = 0;

  top->io_host_aw_valid = 0;
  top->io_host_aw_bits_addr = 0;
  top->io_host_w_valid = 0;
  top->io_host_w_bits_data = 0;
  top->io_host_w_bits_strb = 0;
  top->io_host_b_ready = 0;
  top->io_host_ar_valid = 0;
  top->io_host_ar_bits_addr = 0;
  top->io_host_r_ready = 0;

  top->io_mem_aw_ready = 0;
  top->io_mem_w_ready = 0;
  top->io_mem_b_valid = 0;
  top->io_mem_b_bits_resp = 0;
  top->io_mem_b_bits_id = 0;
  top->io_mem_b_bits_user = 0;
  top->io_mem_ar_ready = 0;
  top->io_mem_r_valid = 0;
  top->io_mem_r_bits_data = 0;
  top->io_mem_r_bits_resp = 0;
  top->io_mem_r_bits_last = 0;
  top->io_mem_r_bits_id = 0;
  top->io_mem_r_bits_user = 0;
}

static void report_output(const char *label, uint64_t val) {
  if (val == 0)
    return;

  std::cout << "    " << label << " = " << val << std::endl;
}

static void report_outputs(VVTAShell *top) {
  report_output("in:  clock", top->clock);
  report_output("in:  reset", top->reset);

  report_output("in:  io_host_aw_valid", top->io_host_aw_valid);
  report_output("out: io_host_aw_ready", top->io_host_aw_ready);
  report_output("in:  io_host_aw_bits_addr", top->io_host_aw_bits_addr);
  report_output("in:  io_host_w_valid", top->io_host_w_valid);
  report_output("out: io_host_w_ready", top->io_host_w_ready);
  report_output("in:  io_host_w_bits_data", top->io_host_w_bits_data);
  report_output("in:  io_host_w_bits_strb", top->io_host_w_bits_strb);
  report_output("in:  io_host_b_ready", top->io_host_b_ready);
  report_output("out: io_host_b_valid", top->io_host_b_valid);
  report_output("out: io_host_b_bits_resp", top->io_host_b_bits_resp);
  report_output("in:  io_host_ar_valid", top->io_host_ar_valid);
  report_output("out: io_host_ar_ready", top->io_host_ar_ready);
  report_output("in:  io_host_ar_bits_addr", top->io_host_ar_bits_addr);
  report_output("in:  io_host_r_ready", top->io_host_r_ready);
  report_output("out: io_host_r_valid", top->io_host_r_valid);
  report_output("out: io_host_r_bits_data", top->io_host_r_bits_data);
  report_output("out: io_host_r_bits_resp", top->io_host_r_bits_resp);

  report_output("in:  io_mem_aw_ready", top->io_mem_aw_ready);
  report_output("out: io_mem_aw_valid", top->io_mem_aw_valid);
  report_output("out: io_mem_aw_bits_addr", top->io_mem_aw_bits_addr);
  report_output("out: io_mem_aw_bits_id", top->io_mem_aw_bits_id);
  report_output("out: io_mem_aw_bits_user", top->io_mem_aw_bits_user);
  report_output("out: io_mem_aw_bits_len", top->io_mem_aw_bits_len);
  report_output("out: io_mem_aw_bits_size", top->io_mem_aw_bits_size);
  report_output("out: io_mem_aw_bits_burst", top->io_mem_aw_bits_burst);
  report_output("out: io_mem_aw_bits_lock", top->io_mem_aw_bits_lock);
  report_output("out: io_mem_aw_bits_cache", top->io_mem_aw_bits_cache);
  report_output("out: io_mem_aw_bits_prot", top->io_mem_aw_bits_prot);
  report_output("out: io_mem_aw_bits_qos", top->io_mem_aw_bits_qos);
  report_output("out: io_mem_aw_bits_region", top->io_mem_aw_bits_region);
  report_output("in:  io_mem_w_ready", top->io_mem_w_ready);
  report_output("out: io_mem_w_valid", top->io_mem_w_valid);
  report_output("out: io_mem_w_bits_data", top->io_mem_w_bits_data);
  report_output("out: io_mem_w_bits_strb", top->io_mem_w_bits_strb);
  report_output("out: io_mem_w_bits_last", top->io_mem_w_bits_last);
  report_output("out: io_mem_w_bits_id", top->io_mem_w_bits_id);
  report_output("out: io_mem_w_bits_user", top->io_mem_w_bits_user);
  report_output("in:  io_mem_b_valid", top->io_mem_b_valid);
  report_output("in:  io_mem_b_bits_resp", top->io_mem_b_bits_resp);
  report_output("in:  io_mem_b_bits_id", top->io_mem_b_bits_id);
  report_output("in:  io_mem_b_bits_user", top->io_mem_b_bits_user);
  report_output("in:  io_mem_ar_ready", top->io_mem_ar_ready);
  report_output("in:  io_mem_r_valid", top->io_mem_r_valid);
  report_output("in:  io_mem_r_bits_data", top->io_mem_r_bits_data);
  report_output("in:  io_mem_r_bits_resp", top->io_mem_r_bits_resp);
  report_output("in:  io_mem_r_bits_last", top->io_mem_r_bits_last);
  report_output("in:  io_mem_r_bits_id", top->io_mem_r_bits_id);
  report_output("in:  io_mem_r_bits_user", top->io_mem_r_bits_user);
}

struct MMIOOp {
  uint64_t id;
  uint64_t addr;
  uint64_t value;
  size_t len;
  bool isWrite;
};

static void mmio_done(MMIOOp *op);

class MMIOInterface {
 protected:
  enum OpState {
    AddrIssued,
    AddrAcked,
    AddrDone,
  };

  VVTAShell &top;
  std::deque<MMIOOp *> queue;
  MMIOOp *rCur, *wCur;
  enum OpState rState, wState;

 public:
  MMIOInterface(VVTAShell &top_)
      : top(top_), rCur(0), wCur(0) {
  }

  void step() {
    if (rCur) {
      /* work on active read operation */

      if (rState == AddrIssued && top.io_host_ar_ready) {
        /* read handshake is complete */
        top.io_host_ar_valid = 0;
        rState = AddrAcked;
      }
      if (rState == AddrAcked && top.io_host_r_valid) {
        /* read data received */
        top.io_host_r_ready = 0;
        rCur->value = top.io_host_r_bits_data;
        mmio_done(rCur);
#ifdef MMIO_DEBUG
        std::cout << main_time << " MMIO: completed AXI read op=" << rCur
                  << " val=" << rCur->value << std::endl;
#endif
        rCur = 0;
      }
    } else if (wCur) {
      /* work on active write operation */

      if (wState == AddrIssued && top.io_host_aw_ready) {
        /* write addr handshake is complete */
        top.io_host_aw_valid = 0;
        wState = AddrAcked;
      }
      if (wState == AddrAcked && top.io_host_w_ready) {
        /* write data handshake is complete */
        top.io_host_w_valid = 0;
        top.io_host_b_ready = 1;
        wState = AddrDone;
      }
      if (wState == AddrDone && top.io_host_b_valid) {
        /* write complete */
        top.io_host_b_ready = 0;
        // TODO(antoinek): check top.io_host_b_resp
#ifdef MMIO_DEBUG
        std::cout << main_time << " MMIO: completed AXI write op=" << wCur
                  << std::endl;
#endif
        mmio_done(wCur);
        wCur = 0;
      }
    } else if (/*!top.clk &&*/ !queue.empty()) {
      /* issue new operation */

      MMIOOp *op = queue.front();
      queue.pop_front();
      if (!op->isWrite) {
        /* issue new read */
        rCur = op;

        rState = AddrIssued;

        top.io_host_ar_bits_addr = rCur->addr;
        top.io_host_ar_valid = 1;
        top.io_host_r_ready = 1;
      } else {
        /* issue new write */
        wCur = op;

        wState = AddrIssued;

        top.io_host_aw_bits_addr = wCur->addr;
        top.io_host_aw_valid = 1;

        top.io_host_w_bits_data = wCur->value;
        top.io_host_w_bits_strb = 0xf;
        top.io_host_w_valid = 1;
      }
    }
  }

  void issueRead(uint64_t id, uint64_t addr, size_t len) {
    MMIOOp *op = new MMIOOp;
#ifdef MMIO_DEBUG
    std::cout << main_time << " MMIO: read id=" << id << " addr=" << std::hex
              << addr << " len=" << len << " op=" << op << std::endl;
#endif
    op->id = id;
    op->addr = addr;
    op->len = len;
    op->isWrite = false;
    queue.push_back(op);
  }

  void issueWrite(uint64_t id, uint64_t addr, size_t len, uint64_t val) {
    MMIOOp *op = new MMIOOp;
#ifdef MMIO_DEBUG
    std::cout << main_time << " MMIO: write id=" << id << " addr=" << std::hex
              << addr << " len=" << len << " val=" << val << " op=" << op
              << std::endl;
#endif
    op->id = id;
    op->addr = addr;
    op->len = len;
    op->value = val;
    op->isWrite = true;
    queue.push_back(op);
  }
};

class MemReader : public AXIReader {
 protected:
  static AXIChannelReadAddr *addrPort(VVTAShell &top) {
    AXIChannelReadAddr *cra = new AXIChannelReadAddr;
    cra->addr_bits = 64;
    cra->id_bits = 8;
    cra->user_bits = 1;

    cra->ready = &top.io_mem_ar_ready;
    cra->valid = &top.io_mem_ar_valid;
    cra->addr = &top.io_mem_ar_bits_addr;
    cra->id = &top.io_mem_ar_bits_id;
    cra->user = &top.io_mem_ar_bits_user;
    cra->len = &top.io_mem_ar_bits_len;
    cra->size = &top.io_mem_ar_bits_size;
    cra->burst = &top.io_mem_ar_bits_burst;
    cra->lock = &top.io_mem_ar_bits_lock;
    cra->cache = &top.io_mem_ar_bits_cache;
    cra->prot = &top.io_mem_ar_bits_prot;
    cra->qos = &top.io_mem_ar_bits_qos;
    cra->region = &top.io_mem_ar_bits_region;
    return cra;
  }

  static AXIChannelReadData *dataPort(VVTAShell &top) {
    AXIChannelReadData *crd = new AXIChannelReadData;
    crd->data_bits = 64;
    crd->id_bits = 8;
    crd->user_bits = 1;

    crd->ready = &top.io_mem_r_ready;
    crd->valid = &top.io_mem_r_valid;
    crd->data = &top.io_mem_r_bits_data;
    crd->resp = &top.io_mem_r_bits_resp;
    crd->last = &top.io_mem_r_bits_last;
    crd->id = &top.io_mem_r_bits_id;
    crd->user = &top.io_mem_r_bits_user;
    return crd;
  }

  virtual void do_read(uint64_t addr, void *buf, size_t len) override {
    if (addr + len > dev_mem_size) {
      throw "mem address out of range";
    }

    memcpy(buf, (const uint8_t *) dev_mem + addr, len);
  }

 public:
  MemReader(VVTAShell &top)
    : AXIReader(*addrPort(top), *dataPort(top))
  {
  }
};

class MemWriter : public AXIWriter {
 protected:
  static AXIChannelWriteAddr *addrPort(VVTAShell &top) {
    AXIChannelWriteAddr *cwa = new AXIChannelWriteAddr;
    cwa->addr_bits = 64;
    cwa->id_bits = 8;
    cwa->user_bits = 1;

    cwa->ready = &top.io_mem_aw_ready;
    cwa->valid = &top.io_mem_aw_valid;
    cwa->addr = &top.io_mem_aw_bits_addr;
    cwa->id = &top.io_mem_aw_bits_id;
    cwa->user = &top.io_mem_aw_bits_user;
    cwa->len = &top.io_mem_aw_bits_len;
    cwa->size = &top.io_mem_aw_bits_size;
    cwa->burst = &top.io_mem_aw_bits_burst;
    cwa->lock = &top.io_mem_aw_bits_lock;
    cwa->cache = &top.io_mem_aw_bits_cache;
    cwa->prot = &top.io_mem_aw_bits_prot;
    cwa->qos = &top.io_mem_aw_bits_qos;
    cwa->region = &top.io_mem_aw_bits_region;
    return cwa;
  }

  static AXIChannelWriteData *dataPort(VVTAShell &top) {
    AXIChannelWriteData *cwd = new AXIChannelWriteData;
    cwd->id_bits = 8;
    cwd->user_bits = 1;

    cwd->ready = &top.io_mem_w_ready;
    cwd->valid = &top.io_mem_w_valid;
    cwd->data = &top.io_mem_w_bits_data;
    cwd->strb = &top.io_mem_w_bits_strb;
    cwd->last = &top.io_mem_w_bits_last;
    cwd->id = &top.io_mem_w_bits_id;
    cwd->user = &top.io_mem_w_bits_user;
    return cwd;
  }

  static AXIChannelWriteResp *respPort(VVTAShell &top) {
    AXIChannelWriteResp *cwr = new AXIChannelWriteResp;
    cwr->id_bits = 8;
    cwr->user_bits = 1;

    cwr->ready = &top.io_mem_b_ready;
    cwr->valid = &top.io_mem_b_valid;
    cwr->resp = &top.io_mem_b_bits_resp;
    cwr->id = &top.io_mem_b_bits_id;
    cwr->user = &top.io_mem_b_bits_user;

    return cwr;
  }

  virtual void do_write(uint64_t addr, const void *buf, size_t len) override {
    if (addr + len > dev_mem_size) {
      throw "mem address out of range";
    }

    memcpy((uint8_t *) dev_mem + addr, buf, len);
  }
 public:
  MemWriter(VVTAShell &top)
    : AXIWriter(*addrPort(top), *dataPort(top), *respPort(top))
  {
  }
};


static void mmio_done(MMIOOp *op) {
  volatile union SimbricksProtoPcieD2H *msg = d2h_alloc();
  volatile struct SimbricksProtoPcieD2HReadcomp *rc;
  volatile struct SimbricksProtoPcieD2HWritecomp *wc;

  if (!msg)
    throw "completion alloc failed";

  if (op->isWrite) {
    wc = &msg->writecomp;
    wc->req_id = op->id;

    SimbricksPcieIfD2HOutSend(&nicif.pcie, msg,
                              SIMBRICKS_PROTO_PCIE_D2H_MSG_WRITECOMP);
  } else {
    rc = &msg->readcomp;
    memcpy((void *)rc->data, &op->value, op->len);
    rc->req_id = op->id;

    SimbricksPcieIfD2HOutSend(&nicif.pcie, msg,
                              SIMBRICKS_PROTO_PCIE_D2H_MSG_READCOMP);
  }

  delete op;
}

#if 0
std::set<DMAOp *> pci_dma_pending;

void pci_dma_issue(DMAOp *op) {
  volatile union SimbricksProtoPcieD2H *msg = d2h_alloc();
  uint8_t ty;

  if (!msg)
    throw "completion alloc failed";

  if (op->write) {
    volatile struct SimbricksProtoPcieD2HWrite *write = &msg->write;
    write->req_id = (uintptr_t)op;
    write->offset = op->dma_addr;
    write->len = op->len;

    // TODO(antoinek): check DMA length
    memcpy((void *)write->data, op->data, op->len);

    SimbricksPcieIfD2HOutSend(&nicif.pcie, msg,
                              SIMBRICKS_PROTO_PCIE_D2H_MSG_WRITE);
  } else {
    volatile struct SimbricksProtoPcieD2HRead *read = &msg->read;
    read->req_id = (uintptr_t)op;
    read->offset = op->dma_addr;
    read->len = op->len;

    SimbricksPcieIfD2HOutSend(&nicif.pcie, msg,
                              SIMBRICKS_PROTO_PCIE_D2H_MSG_READ);
  }

  pci_dma_pending.insert(op);
}

static void h2d_readcomp(volatile struct SimbricksProtoPcieH2DReadcomp *rc) {
  DMAOp *op = (DMAOp *)(uintptr_t)rc->req_id;
  if (pci_dma_pending.find(op) == pci_dma_pending.end())
    throw "unexpected completion";
  pci_dma_pending.erase(op);

  memcpy(op->data, (void *)rc->data, op->len);

#if 0
  std::cerr << "dma read comp: ";
  for (size_t i = 0; i < op->len; i++)
    std::cerr << (unsigned) op->data[i] << " ";
  std::cerr << std::endl;
#endif

  op->engine->pci_op_complete(op);
}

static void h2d_writecomp(volatile struct SimbricksProtoPcieH2DWritecomp *wc) {
  DMAOp *op = (DMAOp *)(uintptr_t)wc->req_id;
  if (pci_dma_pending.find(op) == pci_dma_pending.end())
    throw "unexpected completion";
  pci_dma_pending.erase(op);

  op->engine->pci_op_complete(op);
}
#endif

static void h2d_read(MMIOInterface &mmio,
                     volatile struct SimbricksProtoPcieH2DRead *read) {
  // std::cout << "got read " << read->offset << std::endl;
  if (read->bar == 0) {
    /*printf("read(bar=%u, off=%lu, len=%u) = %lu\n", read->bar, read->offset,
            read->len, val);*/
    mmio.issueRead(read->req_id, read->offset, read->len);
  } else if (read->bar == 2) {
    volatile union SimbricksProtoPcieD2H *msg = d2h_alloc();
    volatile struct SimbricksProtoPcieD2HReadcomp *rc;

    if (!msg)
      throw "completion alloc failed";

    rc = &msg->readcomp;
    memcpy((void *)rc->data, (uint8_t *) dev_mem + read->offset, read->len);
    rc->req_id = read->req_id;

    SimbricksPcieIfD2HOutSend(&nicif.pcie, msg,
                              SIMBRICKS_PROTO_PCIE_D2H_MSG_READCOMP);
  } else {
    throw "unexpected bar";
  }
}

static void h2d_write(MMIOInterface &mmio,
                      volatile struct SimbricksProtoPcieH2DWrite *write) {
  

  // std::cout << "got write " << write->offset << " = " << val << std::endl;

  if (write->bar == 0) {
    uint64_t val = 0;
    memcpy(&val, (void *)write->data, write->len);
    mmio.issueWrite(write->req_id, write->offset, write->len, val);
  } else if (write->bar == 2) {
    volatile union SimbricksProtoPcieD2H *msg = d2h_alloc();
    volatile struct SimbricksProtoPcieD2HWritecomp *wc;

    memcpy((uint8_t *) dev_mem + write->offset, (void *) write->data,
      write->len);

    wc = &msg->writecomp;
    wc->req_id = write->req_id;

    SimbricksPcieIfD2HOutSend(&nicif.pcie, msg,
                              SIMBRICKS_PROTO_PCIE_D2H_MSG_WRITECOMP);
  } else {
    throw "unexpected bar";
  }
}

static void poll_h2d(MMIOInterface &mmio) {
  volatile union SimbricksProtoPcieH2D *msg =
      SimbricksPcieIfH2DInPoll(&nicif.pcie, main_time);
  uint8_t t;

  if (msg == NULL)
    return;

  t = SimbricksPcieIfH2DInType(&nicif.pcie, msg);

  // std::cerr << "poll_h2d: polled type=" << (int) t << std::endl;
  switch (t) {
    case SIMBRICKS_PROTO_PCIE_H2D_MSG_READ:
      h2d_read(mmio, &msg->read);
      break;

    case SIMBRICKS_PROTO_PCIE_H2D_MSG_WRITE:
      h2d_write(mmio, &msg->write);
      break;

    /*case SIMBRICKS_PROTO_PCIE_H2D_MSG_READCOMP:
      h2d_readcomp(&msg->readcomp);
      break;

    case SIMBRICKS_PROTO_PCIE_H2D_MSG_WRITECOMP:
      h2d_writecomp(&msg->writecomp);
      break;*/

    case SIMBRICKS_PROTO_PCIE_H2D_MSG_DEVCTRL:
      break;

    case SIMBRICKS_PROTO_MSG_TYPE_SYNC:
      break;

    case SIMBRICKS_PROTO_MSG_TYPE_TERMINATE:
      std::cerr << "poll_h2d: peer terminated" << std::endl;
      terminated = true;
      break;

    default:
      std::cerr << "poll_h2d: unsupported type=" << t << std::endl;
  }

  SimbricksPcieIfH2DInDone(&nicif.pcie, msg);
}

static volatile union SimbricksProtoPcieD2H *d2h_alloc(void) {
  return SimbricksPcieIfD2HOutAlloc(&nicif.pcie, main_time);
}


int main(int argc, char *argv[]) {
  char *vargs[2] = {argv[0], NULL};
  Verilated::commandArgs(1, vargs);
  struct SimbricksBaseIfParams pcieParams;
#ifdef TRACE_ENABLED
  Verilated::traceEverOn(true);
#endif

  SimbricksPcieIfDefaultParams(&pcieParams);

  if (argc < 3 && argc > 8) {
    fprintf(stderr,
            "Usage: vta_simbricks PCI-SOCKET SHM [START-TICK] "
            "[SYNC-PERIOD] [PCI-LATENCY] [CLOCK-FREQ-MHZ] [DEV-MEM-SIZE-MB]\n");
    return EXIT_FAILURE;
  }
  if (argc >= 4)
    main_time = strtoull(argv[3], NULL, 0);
  if (argc >= 5)
    pcieParams.sync_interval = strtoull(argv[4], NULL, 0) * 1000ULL;
  if (argc >= 6)
    pcieParams.link_latency = strtoull(argv[5], NULL, 0) * 1000ULL;
  if (argc >= 7)
    clock_period = 1000000ULL / strtoull(argv[6], NULL, 0);
  if (argc >= 8)
    dev_mem_size = strtoull(argv[7], NULL, 0) * 1024 * 1024;

  struct SimbricksProtoPcieDevIntro di;
  memset(&di, 0, sizeof(di));

  di.bars[0].len = 1 << 24;
  di.bars[0].flags = SIMBRICKS_PROTO_PCIE_BAR_64;

  di.bars[2].len = dev_mem_size;
  di.bars[2].flags = SIMBRICKS_PROTO_PCIE_BAR_64;

  di.pci_vendor_id = 0xdead;
  di.pci_device_id = 0xbeef;
  di.pci_class = 0x40;
  di.pci_subclass = 0x00;
  di.pci_revision = 0x00;
  di.pci_msi_nvecs = 32;

  pcieParams.sock_path = argv[1];

  if (SimbricksNicIfInit(&nicif, argv[2], nullptr, &pcieParams, &di)) {
    return EXIT_FAILURE;
  }
  int sync_pci = SimbricksBaseIfSyncEnabled(&nicif.pcie.base);

  signal(SIGINT, sigint_handler);
  signal(SIGUSR1, sigusr1_handler);

  dev_mem = new uint8_t[dev_mem_size];

  VVTAShell *top = new VVTAShell;
#ifdef TRACE_ENABLED
  trace = new VerilatedVcdC;
  top->trace(trace, 99);
  trace->open("debug.vcd");
#endif

  MMIOInterface mmio(*top);
  MemReader mem_control_reader(*top);
  MemWriter mem_control_writer(*top);

  reset_inputs(top);
  top->reset = 1;
  top->eval();

  /* raising edge */
  top->clock = !top->clock;
  top->eval();

  top->reset = 0;

  while (!exiting) {
    int done;
    do {
      done = 1;
      if (SimbricksPcieIfD2HOutSync(&nicif.pcie, main_time) < 0) {
        std::cerr << "warn: SimbricksPcieIfD2HOutSync failed (t=" << main_time
                  << ")" << std::endl;
        done = 0;
      }
      if (SimbricksNetIfOutSync(&nicif.net, main_time) < 0) {
        std::cerr << "warn: SimbricksNetIfOutSync failed (t=" << main_time
                  << ")" << std::endl;
        done = 0;
      }
    } while (!done);

    do {
      poll_h2d(mmio);
    } while (
        !exiting &&
        ((sync_pci &&
          SimbricksPcieIfH2DInTimestamp(&nicif.pcie) <= main_time)));

    /* falling edge */
    top->clock = !top->clock;
    main_time += clock_period / 2;
    top->eval();
#ifdef TRACE_ENABLED
    trace->dump(main_time);
#endif

    mmio.step();
    mem_control_writer.step();
    mem_control_reader.step();

    /* raising edge */
    top->clock = !top->clock;
    main_time += clock_period / 2;

    top->eval();
#ifdef TRACE_ENABLED
    trace->dump(main_time);
#endif
  }
  report_outputs(top);
  std::cout << std::endl << std::endl << "main_time:" << main_time << std::endl;

#ifdef TRACE_ENABLED
  trace->dump(main_time + 1);
  trace->close();
#endif
  top->final();
  delete top;
  return 0;
}
