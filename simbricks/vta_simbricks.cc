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

#include <signal.h>
#include <verilated.h>

#include <cassert>

#include "VVTAShell.h"
#include "simbricks/pcie/proto.h"

#define TRACE_ENABLED
// #define MMIO_DEBUG

#include <iostream>
#ifdef TRACE_ENABLED
#include <verilated_vcd_c.h>
#endif

#include <simbricks/base/cxxatomicfix.h>
extern "C" {
#include <simbricks/nicif/nicif.h>
}

#include "vta_simbricks.hh"

// #pragma GCC push_options
// #pragma GCC optimize("O0")

namespace {
uint64_t clock_period = 10 * 1000ULL;  // 10ns -> 100MHz
size_t dev_mem_size = 1024UL * 1024 * 1024;

volatile int exiting = 0;
bool terminated = false;
uint64_t main_time = 0;
struct SimbricksNicIf nicif {};

VTAMemReader *mem_control_reader;
void *dev_mem;

// VerilatedContext *vcontext;
VVTAShell *shell;
#ifdef TRACE_ENABLED
VerilatedVcdC *trace;
#endif

volatile union SimbricksProtoPcieD2H *d2h_alloc();

void sigint_handler(int dummy) {
  exiting = 1;
}

void reset_inputs(VVTAShell &top) {
  top.clock = 0;
  top.reset = 0;

  top.io_host_aw_valid = 0;
  top.io_host_aw_bits_addr = 0;
  top.io_host_w_valid = 0;
  top.io_host_w_bits_data = 0;
  top.io_host_w_bits_strb = 0;
  top.io_host_b_ready = 0;
  top.io_host_ar_valid = 0;
  top.io_host_ar_bits_addr = 0;
  top.io_host_r_ready = 0;

  top.io_mem_aw_ready = 0;
  top.io_mem_w_ready = 0;
  top.io_mem_b_valid = 0;
  top.io_mem_b_bits_resp = 0;
  top.io_mem_b_bits_id = 0;
  top.io_mem_b_bits_user = 0;
  top.io_mem_ar_ready = 0;
  top.io_mem_r_valid = 0;
  top.io_mem_r_bits_data = 0;
  top.io_mem_r_bits_resp = 0;
  top.io_mem_r_bits_last = 0;
  top.io_mem_r_bits_id = 0;
  top.io_mem_r_bits_user = 0;
}

void report_output(const char *label, uint64_t val) {
  if (val == 0)
    return;

  std::cout << "    " << label << " = " << val << "\n";
}

void report_outputs(const VVTAShell &top) {
  report_output("in:  clock", top.clock);
  report_output("in:  reset", top.reset);

  report_output("in:  io_host_aw_valid", top.io_host_aw_valid);
  report_output("out: io_host_aw_ready", top.io_host_aw_ready);
  report_output("in:  io_host_aw_bits_addr", top.io_host_aw_bits_addr);
  report_output("in:  io_host_w_valid", top.io_host_w_valid);
  report_output("out: io_host_w_ready", top.io_host_w_ready);
  report_output("in:  io_host_w_bits_data", top.io_host_w_bits_data);
  report_output("in:  io_host_w_bits_strb", top.io_host_w_bits_strb);
  report_output("in:  io_host_b_ready", top.io_host_b_ready);
  report_output("out: io_host_b_valid", top.io_host_b_valid);
  report_output("out: io_host_b_bits_resp", top.io_host_b_bits_resp);
  report_output("in:  io_host_ar_valid", top.io_host_ar_valid);
  report_output("out: io_host_ar_ready", top.io_host_ar_ready);
  report_output("in:  io_host_ar_bits_addr", top.io_host_ar_bits_addr);
  report_output("in:  io_host_r_ready", top.io_host_r_ready);
  report_output("out: io_host_r_valid", top.io_host_r_valid);
  report_output("out: io_host_r_bits_data", top.io_host_r_bits_data);
  report_output("out: io_host_r_bits_resp", top.io_host_r_bits_resp);

  report_output("in:  io_mem_aw_ready", top.io_mem_aw_ready);
  report_output("out: io_mem_aw_valid", top.io_mem_aw_valid);
  report_output("out: io_mem_aw_bits_addr", top.io_mem_aw_bits_addr);
  report_output("out: io_mem_aw_bits_id", top.io_mem_aw_bits_id);
  report_output("out: io_mem_aw_bits_user", top.io_mem_aw_bits_user);
  report_output("out: io_mem_aw_bits_len", top.io_mem_aw_bits_len);
  report_output("out: io_mem_aw_bits_size", top.io_mem_aw_bits_size);
  report_output("out: io_mem_aw_bits_burst", top.io_mem_aw_bits_burst);
  report_output("out: io_mem_aw_bits_lock", top.io_mem_aw_bits_lock);
  report_output("out: io_mem_aw_bits_cache", top.io_mem_aw_bits_cache);
  report_output("out: io_mem_aw_bits_prot", top.io_mem_aw_bits_prot);
  report_output("out: io_mem_aw_bits_qos", top.io_mem_aw_bits_qos);
  report_output("out: io_mem_aw_bits_region", top.io_mem_aw_bits_region);
  report_output("in:  io_mem_w_ready", top.io_mem_w_ready);
  report_output("out: io_mem_w_valid", top.io_mem_w_valid);
  report_output("out: io_mem_w_bits_data", top.io_mem_w_bits_data);
  report_output("out: io_mem_w_bits_strb", top.io_mem_w_bits_strb);
  report_output("out: io_mem_w_bits_last", top.io_mem_w_bits_last);
  report_output("out: io_mem_w_bits_id", top.io_mem_w_bits_id);
  report_output("out: io_mem_w_bits_user", top.io_mem_w_bits_user);
  report_output("in:  io_mem_b_valid", top.io_mem_b_valid);
  report_output("in:  io_mem_b_bits_resp", top.io_mem_b_bits_resp);
  report_output("in:  io_mem_b_bits_id", top.io_mem_b_bits_id);
  report_output("in:  io_mem_b_bits_user", top.io_mem_b_bits_user);
  report_output("in:  io_mem_ar_ready", top.io_mem_ar_ready);
  report_output("in:  io_mem_r_valid", top.io_mem_r_valid);
  report_output("in:  io_mem_r_bits_data", top.io_mem_r_bits_data);
  report_output("in:  io_mem_r_bits_resp", top.io_mem_r_bits_resp);
  report_output("in:  io_mem_r_bits_last", top.io_mem_r_bits_last);
  report_output("in:  io_mem_r_bits_id", top.io_mem_r_bits_id);
  report_output("in:  io_mem_r_bits_user", top.io_mem_r_bits_user);
}

void sigusr1_handler(int dummy) {
  fprintf(stderr, "main_time = %lu\n", main_time);
  report_outputs(*shell);
}

struct MMIOOp {
  uint64_t id = 0;
  uint64_t addr = 0;
  uint64_t value = 0;
  size_t len = 0;
  bool isWrite = false;
  bool isPosted = false;
};

void mmio_done(MMIOOp *mmio_op);

class MMIOInterface {
 protected:
  VVTAShell &top_;
  std::deque<MMIOOp *> queue_{};
  MMIOOp *rCur_ = nullptr;
  MMIOOp *wCur_ = nullptr;

  bool rAAck_ = false;
  bool rDAck_ = false;
  bool wAAck_ = false;
  bool wDAck_ = false;
  bool wBAck_ = false;

 public:
  explicit MMIOInterface(VVTAShell &top) : top_(top) {
  }

  void step() {
    if (rCur_) {
      /* work on active read operation */
      if (rDAck_) {
        /* read fully completed */
#ifdef MMIO_DEBUG
        std::cout << main_time << " MMIO: completed AXI read op=" << rCur_
                  << " val=" << rCur_->value << "\n";
        // report_outputs(&top);
#endif
        top_.io_host_r_ready = 0;
        mmio_done(rCur_);
        rCur_ = nullptr;
      } else if (top_.io_host_r_valid) {
        assert(rAAck_);
        rCur_->value = top_.io_host_r_bits_data;
        rDAck_ = true;  // need to delay with ready high for a full cycle for
                        // chisel code to fully register
      }

      if (top_.io_host_ar_valid && (top_.io_host_ar_ready || rAAck_)) {
        /* read addr handshake is complete */
#ifdef MMIO_DEBUG
        std::cout << main_time
                  << " MMIO: AXI read addr handshake done op=" << rCur_ << "\n";
        // report_outputs(&top);
#endif
        top_.io_host_ar_valid = 0;
        rAAck_ = true;
      }
    } else if (wCur_) {
      /* work on active write operation */

      if (wBAck_) {
        /* write fully completed */
#ifdef MMIO_DEBUG
        std::cout << main_time << " MMIO: completed AXI wriste op=" << wCur_
                  << "\n";
        // report_outputs(&top);
#endif
        top_.io_host_b_ready = 0;
        mmio_done(wCur_);
        wCur_ = nullptr;
      } else if (top_.io_host_b_valid) {
        assert(wAAck_ && wDAck_);
        wBAck_ = true;  // need to delay with ready high for a full cycle for
                        // chisel code to fully register
      }

      if (top_.io_host_w_valid && (top_.io_host_w_ready || wDAck_)) {
        /* write data handshake is complete */
#ifdef MMIO_DEBUG
        std::cout << main_time
                  << " MMIO: AXI write data handshake done op=" << wCur_
                  << "\n";
        // report_outputs(&top);
#endif
        top_.io_host_w_valid = 0;
        wDAck_ = true;
      }

      if (top_.io_host_aw_valid && (top_.io_host_aw_ready || wAAck_)) {
        /* write addr handshake is complete */
#ifdef MMIO_DEBUG
        std::cout << main_time
                  << " MMIO: AXI write addr handshake done op=" << wCur_
                  << "\n";
        // report_outputs(&top);
#endif
        top_.io_host_aw_valid = 0;
        wAAck_ = true;

        wDAck_ = top_.io_host_w_ready;
        top_.io_host_w_valid = 1;
      }

    } else if (/*!top.clk &&*/ !queue_.empty()) {
      /* issue new operation */
      MMIOOp *mmio_op = queue_.front();
#ifdef MMIO_DEBUG
      std::cout << main_time << " MMIO: issuing new op on axi op=" << mmio_op
                << "\n";
      // report_outputs(&top);
#endif
      queue_.pop_front();
      if (!mmio_op->isWrite) {
        /* issue new read */
        rCur_ = mmio_op;

        top_.io_host_ar_bits_addr = rCur_->addr;
        rAAck_ = top_.io_host_ar_ready;
        top_.io_host_ar_valid = 1;
        rDAck_ = false;
        top_.io_host_r_ready = 1;
      } else {
        /* issue new write */
        wCur_ = mmio_op;

        top_.io_host_aw_bits_addr = wCur_->addr;
        wAAck_ = top_.io_host_aw_ready;
        top_.io_host_aw_valid = 1;

        top_.io_host_w_bits_data = wCur_->value;
        top_.io_host_w_bits_strb = 0xf;
        wDAck_ = false;
        top_.io_host_w_valid = 0;

        wBAck_ = false;
        top_.io_host_b_ready = 1;
      }
    }
  }

  void issueRead(uint64_t req_id, uint64_t addr, size_t len) {
    MMIOOp *mmio_op = new MMIOOp{};
#ifdef MMIO_DEBUG
    std::cout << main_time << " MMIO: read id=" << req_id
              << " addr=" << std::hex << addr << " len=" << len
              << " op=" << mmio_op << "\n";
#endif
    mmio_op->id = req_id;
    mmio_op->addr = addr;
    mmio_op->len = len;
    mmio_op->isWrite = false;
    queue_.push_back(mmio_op);
  }

  void issueWrite(uint64_t req_id, uint64_t addr, size_t len, uint64_t val,
                  bool isPosted) {
    MMIOOp *mmio_op = new MMIOOp{};
#ifdef MMIO_DEBUG
    std::cout << main_time << " MMIO: write id=" << req_id
              << " addr=" << std::hex << addr << " len=" << len
              << " val=" << val << " op=" << mmio_op << "\n";
#endif
    mmio_op->id = req_id;
    mmio_op->addr = addr;
    mmio_op->len = len;
    mmio_op->value = val;
    mmio_op->isWrite = true;
    mmio_op->isPosted = isPosted;
    queue_.push_back(mmio_op);
  }
};

void mmio_done(MMIOOp *mmio_op) {
  if (!mmio_op->isWrite || !mmio_op->isPosted) {
    volatile union SimbricksProtoPcieD2H *msg = d2h_alloc();
    volatile struct SimbricksProtoPcieD2HReadcomp *readcomp;
    volatile struct SimbricksProtoPcieD2HWritecomp *writecomp;

    if (!msg)
      throw "completion alloc failed";

    if (mmio_op->isWrite) {
      writecomp = &msg->writecomp;
      writecomp->req_id = mmio_op->id;

      SimbricksPcieIfD2HOutSend(&nicif.pcie, msg,
                                SIMBRICKS_PROTO_PCIE_D2H_MSG_WRITECOMP);
    } else if (!mmio_op->isWrite) {
      readcomp = &msg->readcomp;
      // NOLINTNEXTLINE(google-readability-casting)
      memcpy((void *)readcomp->data, &mmio_op->value, mmio_op->len);
      readcomp->req_id = mmio_op->id;

      SimbricksPcieIfD2HOutSend(&nicif.pcie, msg,
                                SIMBRICKS_PROTO_PCIE_D2H_MSG_READCOMP);
    }
  }

  delete mmio_op;
}

void h2d_read(MMIOInterface &mmio,
              volatile struct SimbricksProtoPcieH2DRead *read) {
  // std::cout << "got read " << read->offset << "\n";
  if (read->bar == 0) {
    /*printf("read(bar=%u, off=%lu, len=%u) = %lu\n", read->bar, read->offset,
            read->len, val);*/
    mmio.issueRead(read->req_id, read->offset, read->len);
  } else if (read->bar == 2) {
    volatile union SimbricksProtoPcieD2H *msg = d2h_alloc();
    volatile struct SimbricksProtoPcieD2HReadcomp *readcomp;

    if (!msg)
      throw "completion alloc failed";

    readcomp = &msg->readcomp;
    // NOLINTNEXTLINE(google-readability-casting)
    memcpy((void *)readcomp->data,
           static_cast<uint8_t *>(dev_mem) + read->offset, read->len);
    readcomp->req_id = read->req_id;

    SimbricksPcieIfD2HOutSend(&nicif.pcie, msg,
                              SIMBRICKS_PROTO_PCIE_D2H_MSG_READCOMP);
  } else {
    throw "unexpected bar";
  }
}

void h2d_write(MMIOInterface &mmio,
               volatile struct SimbricksProtoPcieH2DWrite *write,
               bool isPosted) {
  // std::cout << "got write " << write->offset << " = " << val << "\n";

  if (write->bar == 0) {
    uint64_t val = 0;
    // NOLINTNEXTLINE(google-readability-casting)
    memcpy(&val, (void *)write->data, write->len);
    mmio.issueWrite(write->req_id, write->offset, write->len, val, isPosted);
  } else if (write->bar == 2) {
    // NOLINTNEXTLINE(google-readability-casting)
    memcpy(static_cast<uint8_t *>(dev_mem) + write->offset, (void *)write->data,
           write->len);

    if (isPosted) {
      return;
    }

    volatile union SimbricksProtoPcieD2H *msg = d2h_alloc();
    volatile struct SimbricksProtoPcieD2HWritecomp *writecomp;

    writecomp = &msg->writecomp;
    writecomp->req_id = write->req_id;

    SimbricksPcieIfD2HOutSend(&nicif.pcie, msg,
                              SIMBRICKS_PROTO_PCIE_D2H_MSG_WRITECOMP);
  } else {
    throw "unexpected bar";
  }
}

void h2d_readcomp(volatile struct SimbricksProtoPcieH2DReadcomp *readcomp) {
  VTAMemReader::AXIOperationT *axi_op =
      // NOLINTNEXTLINE(performance-no-int-to-ptr)
      reinterpret_cast<VTAMemReader::AXIOperationT *>(readcomp->req_id);
  memcpy(axi_op->buf, const_cast<uint8_t *>(readcomp->data), axi_op->len);

  mem_control_reader->readDone(axi_op);
}

void h2d_writecomp(volatile struct SimbricksProtoPcieH2DWritecomp *writecomp) {
  // std::cout << "dma write completed" << "\n";
}

void poll_h2d(MMIOInterface &mmio) {
  volatile union SimbricksProtoPcieH2D *msg =
      SimbricksPcieIfH2DInPoll(&nicif.pcie, main_time);
  uint16_t type;

  if (msg == nullptr)
    return;

  type = SimbricksPcieIfH2DInType(&nicif.pcie, msg);

  // std::cerr << "poll_h2d: polled type=" << (int) t << "\n";
  switch (type) {
    case SIMBRICKS_PROTO_PCIE_H2D_MSG_READ:
      h2d_read(mmio, &msg->read);
      break;

    case SIMBRICKS_PROTO_PCIE_H2D_MSG_WRITE:
      h2d_write(mmio, &msg->write, false);
      break;

    case SIMBRICKS_PROTO_PCIE_H2D_MSG_WRITE_POSTED:
      h2d_write(mmio, &msg->write, true);
      break;

    case SIMBRICKS_PROTO_PCIE_H2D_MSG_READCOMP:
      h2d_readcomp(&msg->readcomp);
      break;

    case SIMBRICKS_PROTO_PCIE_H2D_MSG_WRITECOMP:
      h2d_writecomp(&msg->writecomp);
      break;

    case SIMBRICKS_PROTO_PCIE_H2D_MSG_DEVCTRL:
    case SIMBRICKS_PROTO_MSG_TYPE_SYNC:
      break;

    case SIMBRICKS_PROTO_MSG_TYPE_TERMINATE:
      std::cerr << "poll_h2d: peer terminated"
                << "\n";
      terminated = true;
      break;

    default:
      std::cerr << "poll_h2d: unsupported type=" << type << "\n";
  }

  SimbricksPcieIfH2DInDone(&nicif.pcie, msg);
}

volatile union SimbricksProtoPcieD2H *d2h_alloc() {
  return SimbricksPcieIfD2HOutAlloc(&nicif.pcie, main_time);
}

}  // namespace

void VTAMemReader::doRead(AXIOperationT *axi_op) {
  volatile union SimbricksProtoPcieD2H *msg = d2h_alloc();
  if (!msg)
    throw "dma read alloc failed";

  volatile struct SimbricksProtoPcieD2HRead *read = &msg->read;
  // NOLINTNEXTLINE(google-readability-casting)
  read->req_id = (uintptr_t)axi_op;
  read->offset = axi_op->addr;
  read->len = axi_op->len;

  assert(SimbricksPcieIfH2DOutMsgLen(&nicif.pcie) -
                 sizeof(SimbricksProtoPcieH2DReadcomp) >=
             axi_op->len &&
         "Read response can't fit the required number of bytes");

  SimbricksPcieIfD2HOutSend(&nicif.pcie, msg,
                            SIMBRICKS_PROTO_PCIE_D2H_MSG_READ);
}

void VTAMemWriter::doWrite(AXIOperationT *axi_op) {
  volatile union SimbricksProtoPcieD2H *msg = d2h_alloc();
  if (!msg)
    throw "dma read alloc failed";

  volatile struct SimbricksProtoPcieD2HWrite *write = &msg->write;
  // NOLINTNEXTLINE(google-readability-casting)
  write->req_id = (uintptr_t)axi_op;
  write->offset = axi_op->addr;
  write->len = axi_op->len;

  assert(SimbricksPcieIfD2HOutMsgLen(&nicif.pcie) -
                 sizeof(SimbricksProtoPcieD2HWrite) >=
             axi_op->len &&
         "Write message can't fit the required number of bytes");

  // NOLINTNEXTLINE(google-readability-casting)
  memcpy((void *)write->data, axi_op->buf, axi_op->len);
  SimbricksPcieIfD2HOutSend(&nicif.pcie, msg,
                            SIMBRICKS_PROTO_PCIE_D2H_MSG_WRITE);

  writeDone(axi_op);
}

int main(int argc, char *argv[]) {
  struct SimbricksBaseIfParams pcie_params;

  SimbricksPcieIfDefaultParams(&pcie_params);

  if (argc < 3 || argc > 8) {
    fprintf(stderr,
            "Usage: vta_simbricks PCI-SOCKET SHM [START-TICK] "
            "[SYNC-PERIOD] [PCI-LATENCY] [CLOCK-FREQ-MHZ] [DEV-MEM-SIZE-MB]\n");
    return EXIT_FAILURE;
  }
  if (argc >= 4)
    main_time = strtoull(argv[3], NULL, 0);
  if (argc >= 5)
    pcie_params.sync_interval = strtoull(argv[4], NULL, 0) * 1000ULL;
  if (argc >= 6)
    pcie_params.link_latency = strtoull(argv[5], NULL, 0) * 1000ULL;
  if (argc >= 7)
    clock_period = 1000000ULL / strtoull(argv[6], NULL, 0);
  if (argc >= 8)
    dev_mem_size = strtoull(argv[7], NULL, 0) * 1024 * 1024;

  struct SimbricksProtoPcieDevIntro dev_intro;
  memset(&dev_intro, 0, sizeof(dev_intro));

  dev_intro.bars[0].len = 1 << 24;
  dev_intro.bars[0].flags = SIMBRICKS_PROTO_PCIE_BAR_64;

  dev_intro.bars[2].len = dev_mem_size;
  dev_intro.bars[2].flags = SIMBRICKS_PROTO_PCIE_BAR_64;

  dev_intro.pci_vendor_id = 0xdead;
  dev_intro.pci_device_id = 0xbeef;
  dev_intro.pci_class = 0x40;
  dev_intro.pci_subclass = 0x00;
  dev_intro.pci_revision = 0x00;
  dev_intro.pci_msi_nvecs = 32;

  pcie_params.sock_path = argv[1];

  if (SimbricksNicIfInit(&nicif, argv[2], nullptr, &pcie_params, &dev_intro)) {
    return EXIT_FAILURE;
  }
  int sync_pci = SimbricksBaseIfSyncEnabled(&nicif.pcie.base);

  signal(SIGINT, sigint_handler);
  signal(SIGUSR1, sigusr1_handler);

  dev_mem = new uint8_t[dev_mem_size];

  /* initialize verilated model */
  // vcontext = new VerilatedContext{};
  shell = new VVTAShell;

#ifdef TRACE_ENABLED
  Verilated::traceEverOn(true);
  // vcontext->traceEverOn(true);
  trace = new VerilatedVcdC;
  shell->trace(trace, 99);
  trace->open("debug.vcd");
#endif

  MMIOInterface mmio{*shell};
  mem_control_reader = new VTAMemReader{*shell};
  VTAMemWriter mem_control_writer{*shell};

  reset_inputs(*shell);
  shell->reset = 1;
  shell->clock = 0;
  shell->eval();
  // vcontext->timeInc(1);

  /* raising edge */
  shell->clock = !shell->clock;
  shell->eval();
  // vcontext->timeInc(1);

  shell->reset = 0;

  /* main simulation loop */
  while (!exiting) {
    int done;
    do {
      done = 1;
      if (SimbricksPcieIfD2HOutSync(&nicif.pcie, main_time) < 0) {
        std::cerr << "warn: SimbricksPcieIfD2HOutSync failed (t=" << main_time
                  << ")"
                  << "\n";
        done = 0;
      }
      if (SimbricksNetIfOutSync(&nicif.net, main_time) < 0) {
        std::cerr << "warn: SimbricksNetIfOutSync failed (t=" << main_time
                  << ")"
                  << "\n";
        done = 0;
      }
    } while (!done);

    do {
      poll_h2d(mmio);
    } while (!exiting && ((sync_pci && SimbricksPcieIfH2DInTimestamp(
                                           &nicif.pcie) <= main_time)));

    /* falling edge */
    shell->clock = !shell->clock;
    main_time += clock_period / 2;
    shell->eval();
    // vcontext->timeInc(1);
#ifdef TRACE_ENABLED
    trace->dump(main_time);
#endif

    mmio.step();
    mem_control_writer.step(main_time);
    mem_control_reader->step(main_time);

    /* raising edge */
    shell->clock = !shell->clock;
    main_time += clock_period / 2;

    shell->eval();
    // vcontext->timeInc(1);
#ifdef TRACE_ENABLED
    trace->dump(main_time);
#endif
  }
  report_outputs(*shell);
  std::cout << "\n"
            << "\n"
            << "main_time:" << main_time << "\n";

#ifdef TRACE_ENABLED
  trace->dump(main_time + 1);
  trace->close();
  delete trace;
#endif
  shell->final();
  delete shell;
  // delete vcontext;
  return 0;
}

// #pragma GCC pop_options
