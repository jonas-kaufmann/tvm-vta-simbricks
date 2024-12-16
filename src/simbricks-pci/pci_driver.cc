/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 *
 * \file pci_driver.c
 * \brief VTA driver for SimBricks simulated PCI boards support.
 */

#include "pci_driver.h"

#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <vta/driver.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <thread>
#include <unordered_map>
#include <utility>

#include "vfio.h"

extern "C" {
#include <contiguousMalloc.h>
}

static void* reg_bar = nullptr;
#if SIM_CTRL
static void* sim_ctrl_bar = nullptr;
#endif
static int vfio_fd = -1;
static std::unordered_map<void*, std::pair<size_t, uintptr_t>> cma_map{};
static std::chrono::steady_clock::time_point begin;
static std::chrono::steady_clock::time_point end;
static bool running = false;

void* VTAMemAlloc(size_t size, int cached) {
  std::cout << __func__ << "(size=" << size << ")" << std::endl;
  // round up to next page size
  size_t remainder = size % 4096;
  if (remainder != 0) {
    size = size + 4096 - remainder;
  }
  uintptr_t phys_addr;
  void* addr = mallocContiguous(size, &phys_addr);
  cma_map.emplace(addr, std::make_pair(size, phys_addr));
  return addr;
}

void VTAMemFree(void* buf) {
  auto entry = cma_map.find(buf);
  if (entry == cma_map.end()) {
    return;
  }
  std::cout << __func__ << "(buf=" << buf << ")" << std::endl;
  freeContiguous(entry->second.second, buf, entry->second.first);
  cma_map.erase(buf);
}

vta_phy_addr_t VTAMemGetPhyAddr(void* buf) { return cma_map.at(buf).second; }

void VTAMemCopyFromHost(void* dst, const void* src, size_t size) {
  // For SoC-based FPGAs that used shared memory with the CPU, use memcopy()
  memcpy(dst, src, size);
}

void VTAMemCopyToHost(void* dst, const void* src, size_t size) {
  // For SoC-based FPGAs that used shared memory with the CPU, use memcopy()
  memcpy(dst, src, size);
}

void VTAFlushCache(void* vir_addr, vta_phy_addr_t phy_addr, int size) {
  // Call the cma_flush_cache on the CMA buffer
  // so that the FPGA can read the buffer data.
  // cma_flush_cache(vir_addr, phy_addr, size);
}

void VTAInvalidateCache(void* vir_addr, vta_phy_addr_t phy_addr, int size) {
  // Call the cma_invalidate_cache on the CMA buffer
  // so that the host needs to read the buffer data.
  // cma_invalidate_cache(vir_addr, phy_addr, size);
}

void* VTAMapRegister(uint32_t addr) {
  if (!reg_bar) {
#if !ULTRA96V2
    char* device = std::getenv("VTA_DEVICE");
    if (device == nullptr) {
      std::cerr << "VTA_DEVICE is not set" << std::endl;
      abort();
    }
    if ((vfio_fd = vfio_init(device)) < 0) {
      std::cerr << "vfio init failed" << std::endl;
      abort();
    }

    size_t reg_len = 0;
    if (vfio_map_region(vfio_fd, 0, &reg_bar, &reg_len)) {
      std::cerr << "vfio map region failed" << std::endl;
      abort();
    }

    if (vfio_busmaster_enable(vfio_fd)) {
      std::cerr << "vfio busmaster enable failed" << std::endl;
      abort();
    }
#else
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
      std::cerr << "opening devmem failed" << std::endl;
      abort();
    }

    size_t alloc_size = 4 * 1024;
    off_t alloc_phys_base = 0xA0000000;
    reg_bar = mmap(NULL, alloc_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, alloc_phys_base);
    if (reg_bar == MAP_FAILED) {
      std::cerr << "mmap devmem failed" << std::endl;
      abort();
    }
#endif

#if SIM_CTRL
    reg_len = 0;
    if (vfio_map_region(vfio_fd, 1, &sim_ctrl_bar, &reg_len)) {
      std::cerr << "vfio map region failed for simulation control bar failed" << std::endl;
      abort();
    }
#endif

    // std::cerr << "vfio registers mapped (len = " << reg_len << ")" << std::endl;
  }

  return (uint8_t*)reg_bar + addr;
}

void VTAUnmapRegister(void* vta) {
  // Unmap memory
  // TODO
}

void VTAWriteMappedReg(void* base_addr, uint32_t offset, uint32_t val) {
  *((volatile uint32_t*)(reinterpret_cast<char*>(base_addr) + offset)) = val;
}

uint32_t VTAReadMappedReg(void* base_addr, uint32_t offset) {
  return *((volatile uint32_t*)(reinterpret_cast<char*>(base_addr) + offset));
}

class VTADevice {
 public:
  VTADevice() {
    // VTA stage handles
    vta_host_handle_ = VTAMapRegister(0);
    dry_run_file_ = std::getenv("VTA_DRY_RUN_FILE");
  }

  ~VTADevice() {
    // Close VTA stage handle
    VTAUnmapRegister(vta_host_handle_);
  }

  int Run(vta_phy_addr_t insn_phy_addr, uint32_t insn_count, uint32_t wait_cycles) {
    // Skip invoking the accelerator if the VTA_DRY_RUN_FILE exists
    if (dry_run_file_ != nullptr && std::filesystem::exists(dry_run_file_)) {
      std::cout << "VTADevice::" << __func__ << "() dry run, skipping invoking the accelerator\n";
      return 0;
    }

    std::cout << "VTADevice::" << __func__ << "() invoking the accelerator\n";

    if (!running) {
      begin = std::chrono::steady_clock::now();
      running = true;
#if SIM_CTRL
      VTAWriteMappedReg(sim_ctrl_bar, 0, 1);
#endif
    }
    VTAWriteMappedReg(vta_host_handle_, 0x04, 0);
    VTAWriteMappedReg(vta_host_handle_, 0x08, insn_count);
    VTAWriteMappedReg(vta_host_handle_, 0x0c, insn_phy_addr);
    VTAWriteMappedReg(vta_host_handle_, 0x10, insn_phy_addr >> 32);

    // VTA start
    VTAWriteMappedReg(vta_host_handle_, 0x0, VTA_START);

    // Loop until the VTA is done
    unsigned t, flag = 0;
    for (t = 0; t < wait_cycles; ++t) {
      flag = VTAReadMappedReg(vta_host_handle_, 0x00);
      flag &= 0x2;
      if (flag == 0x2) break;
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    // Report error if timeout
    return t < wait_cycles ? 0 : 1;
  }

 private:
  // VTA handles (register maps)
  void* vta_host_handle_{nullptr};
  char* dry_run_file_{nullptr};
};

VTADeviceHandle VTADeviceAlloc() { return new VTADevice(); }

void VTADeviceFree(VTADeviceHandle handle) {
#if SIM_CTRL
  VTAWriteMappedReg(sim_ctrl_bar, 0, 0);
#endif
  end = std::chrono::steady_clock::now();
  running = false;
  std::cout << "Accelerator latency "
            << std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() << " ns\n";
  std::cout << __func__ << "(" << handle << ")\n";
  delete static_cast<VTADevice*>(handle);

  // Free allocated memory
  for (auto& entry : cma_map) {
    freeContiguous(entry.second.second, entry.first, entry.second.first);
  }
  cma_map.clear();
}

int VTADeviceRun(VTADeviceHandle handle, vta_phy_addr_t insn_phy_addr, uint32_t insn_count,
                 uint32_t wait_cycles) {
  return static_cast<VTADevice*>(handle)->Run(insn_phy_addr, insn_count, wait_cycles);
}
