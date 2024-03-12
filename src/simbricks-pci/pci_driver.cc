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

#include <vta/driver.h>
#include <cstdlib>
#include <thread>
#include <iostream>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>
#include <stdlib.h>
#include "pci_driver.h"
#include "vfio.h"

static void *reg_bar = nullptr;
static int vfio_fd = -1;

static void *alloc_base = nullptr;
static uint64_t alloc_phys_base = 1ULL * 1024 * 1024 * 1024;
static size_t alloc_size = 512 * 1024 * 1024;
static size_t alloc_off = 0;

static void alloc_init()
{
  if (alloc_base)
    return;

  std::cerr << "simbricks-pci: initializing allocator" << std::endl;
  int fd = open("/dev/mem", O_RDWR | O_SYNC);
  if (fd < 0) {
    std::cerr << "opening devmem failed" << std::endl;
    abort();
  }

  void *mem = mmap(NULL, alloc_size, PROT_READ | PROT_WRITE, MAP_SHARED,
			fd, alloc_phys_base);
  if (mem == MAP_FAILED) {
    std::cerr << "mmap devmem failed" << std::endl;
    abort();
  }
  alloc_base = mem;

  std::cerr << "simbricks-pci: allocator initialized" << std::endl;
}

void* VTAMemAlloc(size_t size, int cached) {
  //std::cerr << "simbricks-pci: VTAMemAlloc(" << size << ")" << std::endl;
  alloc_init();

  assert (alloc_off + size <= alloc_size);
  void *addr = (void *) ((uint8_t *) alloc_base + alloc_off);
  alloc_off += size;
  //std::cerr << "simbricks-pci:    = " << addr << std::endl;
  return addr;
}

void VTAMemFree(void* buf) {
  //std::cerr << "simbricks-pci: VTAMemFree(" << buf << ")" << std::endl;
  // TODO
}

vta_phy_addr_t VTAMemGetPhyAddr(void* buf) {
  return alloc_phys_base + ((uintptr_t) buf - (uintptr_t) alloc_base);
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
  //cma_flush_cache(vir_addr, phy_addr, size);
}

void VTAInvalidateCache(void* vir_addr, vta_phy_addr_t phy_addr, int size) {
  // Call the cma_invalidate_cache on the CMA buffer
  // so that the host needs to read the buffer data.
  //cma_invalidate_cache(vir_addr, phy_addr, size);
}

void *VTAMapRegister(uint32_t addr) {
  if (!reg_bar) {
    char *device = std::getenv("VTA_DEVICE");
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

    //std::cerr << "vfio registers mapped (len = " << reg_len << ")" << std::endl;
  }

  return (uint8_t *) reg_bar + addr;
}

void VTAUnmapRegister(void *vta) {
  // Unmap memory
  // TODO
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
    vta_host_handle_ = VTAMapRegister(0);
  }

  ~VTADevice() {
    // Close VTA stage handle
    VTAUnmapRegister(vta_host_handle_);
  }

  int Run(vta_phy_addr_t insn_phy_addr,
          uint32_t insn_count,
          uint32_t wait_cycles) {
    VTAWriteMappedReg(vta_host_handle_, 0x08, insn_count);
    VTAWriteMappedReg(vta_host_handle_, 0x0c, insn_phy_addr);
    VTAWriteMappedReg(vta_host_handle_, 0x10, insn_phy_addr >> 32);
    VTAWriteMappedReg(vta_host_handle_, 0x14, 0);
    VTAWriteMappedReg(vta_host_handle_, 0x18, 0);
    VTAWriteMappedReg(vta_host_handle_, 0x1c, 0);
    VTAWriteMappedReg(vta_host_handle_, 0x20, 0);

    // VTA start
    VTAWriteMappedReg(vta_host_handle_, 0x0, VTA_START);

    // Loop until the VTA is done
    unsigned t, flag = 0;
    for (;;) {
      flag = VTAReadMappedReg(vta_host_handle_, 0x00);
      flag &= 0x2;
      if (flag == 0x2) break;
      std::this_thread::yield();
    }
    // Report error if timeout
    // return t < wait_cycles ? 0 : 1;
    return 0;
  }

 private:
  // VTA handles (register maps)
  void* vta_host_handle_{nullptr};
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
  return static_cast<VTADevice*>(handle)->Run(
      insn_phy_addr, insn_count, wait_cycles);
}
