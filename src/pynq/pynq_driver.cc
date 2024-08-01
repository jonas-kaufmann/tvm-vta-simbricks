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
 * \file pynq_driver.c
 * \brief VTA driver for Zynq SoC boards with Pynq support (see pynq.io).
 */

#include <vta/driver.h>
#include <iostream>
#include <thread>
#include <time.h>
#include "pynq_driver.h"

#define DEBUG_PRINTS


void* VTAMemAlloc(size_t size, int cached) {
  #ifdef DEBUG_PRINTS
  std::cout << "VTAMemAlloc() enter" << std::endl;
  #endif
  assert(size <= VTA_MAX_XFER);
  // Rely on the pynq-specific cma library
  void *a = cma_alloc(size, cached);
  #ifdef DEBUG_PRINTS
  std::cout << "VTAMemAlloc() leave" << std::endl;
  #endif
  return a;
}

void VTAMemFree(void* buf) {
  // Rely on the pynq-specific cma library
  cma_free(buf);
}

vta_phy_addr_t VTAMemGetPhyAddr(void* buf) {
  return cma_get_phy_addr(buf);
}

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
  cma_flush_cache(vir_addr, phy_addr, size);
}

void VTAInvalidateCache(void* vir_addr, vta_phy_addr_t phy_addr, int size) {
  // Call the cma_invalidate_cache on the CMA buffer
  // so that the host needs to read the buffer data.
  cma_invalidate_cache(vir_addr, phy_addr, size);
}

void *VTAMapRegister(uint32_t addr) {
  // Align the base address with the pages
  uint32_t virt_base = addr & ~(getpagesize() - 1);
  // Calculate base address offset w.r.t the base address
  uint32_t virt_offset = addr - virt_base;
  // Open file and mmap
  uint32_t mmap_file = open("/dev/mem", O_RDWR|O_SYNC);
#ifdef DEBUG_PRINTS
  std::cout << "VTAMapRegister(0x" << std::hex << addr << ") addr=0x" << addr
            << " virt_base=0x" << virt_base << " virt_offset=0x" << virt_offset
            << " VTA_IP_REG_MAP_RANGE=0x" << VTA_IP_REG_MAP_RANGE << std::endl;
#endif
  return mmap(NULL,
              (VTA_IP_REG_MAP_RANGE + virt_offset),
              PROT_READ|PROT_WRITE,
              MAP_SHARED,
              mmap_file,
              virt_base);
}

void VTAUnmapRegister(void *vta) {
  // Unmap memory
  int status = munmap(vta, VTA_IP_REG_MAP_RANGE);
  assert(status == 0);
}

void VTAWriteMappedReg(void* base_addr, uint32_t offset, uint32_t val) {
  *((volatile uint32_t *) (reinterpret_cast<char *>(base_addr) + offset)) = val;
}

uint32_t VTAReadMappedReg(void* base_addr, uint32_t offset) {
  return *((volatile uint32_t *) (reinterpret_cast<char *>(base_addr) + offset));
}

class VTADevice {
 public:
  VTADevice() {
    // VTA stage handles
    vta_handle_ = VTAMapRegister(VTA_FETCH_ADDR);
  }

  ~VTADevice() {
    // Close VTA stage handle
    VTAUnmapRegister(vta_handle_);
  }

  int Run(vta_phy_addr_t insn_phy_addr,
          uint32_t insn_count,
          uint32_t wait_cycles) {
    VTAWriteMappedReg(vta_handle_, 0x08, insn_count);
    VTAWriteMappedReg(vta_handle_, 0x0c, insn_phy_addr);
    VTAWriteMappedReg(vta_handle_, 0x10, insn_phy_addr >> 32);
    VTAWriteMappedReg(vta_handle_, 0x14, 0);
    VTAWriteMappedReg(vta_handle_, 0x18, 0);
    VTAWriteMappedReg(vta_handle_, 0x1c, 0);
    VTAWriteMappedReg(vta_handle_, 0x20, 0);

    // VTA start
    VTAWriteMappedReg(vta_handle_, 0x0, VTA_START);

    // Allow device to respond
    struct timespec ts = { .tv_sec = 0, .tv_nsec = 1000 };
    nanosleep(&ts, &ts);

    // Loop until the VTA is done
    std::cout << "Run() now waiting for VTA to finish..." << std::endl;
    unsigned t, flag = 0;
    for (t = 0; t < wait_cycles; ++t) {
      flag = VTAReadMappedReg(vta_handle_, 0x00);
      if (flag == 0x2) break;
      std::this_thread::yield();
    }
    // Report error if timeout
    return t < wait_cycles ? 0 : 1;
  }

 private:
  // VTA handles (register maps)
  void* vta_handle_{nullptr};
};

VTADeviceHandle VTADeviceAlloc() {
  return new VTADevice();
}

void VTADeviceFree(VTADeviceHandle handle) {
  delete static_cast<VTADevice*>(handle);
}

int VTADeviceRun(VTADeviceHandle handle,
                 vta_phy_addr_t insn_phy_addr,
                 uint32_t insn_count,
                 uint32_t wait_cycles) {
  #ifdef DEBUG_PRINTS
  std::cout << "VTADeviceRun()" << std::endl;
  #endif
  return static_cast<VTADevice*>(handle)->Run(
      insn_phy_addr, insn_count, wait_cycles);
}
