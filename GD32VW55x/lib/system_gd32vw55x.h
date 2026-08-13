/*!
    \file    system_gd32vw55x.h
    \brief   RISC-V Device Peripheral Access Layer Header File for
             GD32VW55x Device Series

    \version 2026-02-04, V1.6.0, firmware for GD32VW55x
*/

/*
 * Copyright (c) 2009-2018 Arm Limited. All rights reserved.
 * Copyright (c) 2019 Nuclei Limited. All rights reserved.
 * Copyright (c) 2026, GigaDevice Semiconductor Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* This file refers the RISC-V standard, some adjustments are made according to GigaDevice chips */

#ifndef SYSTEM_GD32VW55x_H
#define SYSTEM_GD32VW55x_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#if   defined (__ICCRISCV__)
    #include "compiler.h"
#endif

/* firmware version can be aquired by uncommenting the macro */
#define __FIRMWARE_VERSION_DEFINE

typedef struct EXC_Frame {
  unsigned long ra;                /* ra: x1, return address for jump */
  unsigned long tp;                /* tp: x4, thread pointer */
  unsigned long t0;                /* t0: x5, temporary register 0 */
  unsigned long t1;                /* t1: x6, temporary register 1 */
  unsigned long t2;                /* t2: x7, temporary register 2 */
  unsigned long a0;                /* a0: x10, return value or function argument 0 */
  unsigned long a1;                /* a1: x11, return value or function argument 1 */
  unsigned long a2;                /* a2: x12, function argument 2 */
  unsigned long a3;                /* a3: x13, function argument 3 */
  unsigned long a4;                /* a4: x14, function argument 4 */
  unsigned long a5;                /* a5: x15, function argument 5 */
  unsigned long mcause;            /* mcause: machine cause csr register */
  unsigned long mepc;              /* mepc: machine exception program counter csr register */
  unsigned long msubm;             /* msubm: machine sub-mode csr register, nuclei customized */
#ifndef __riscv_32e
  unsigned long a6;                /* a6: x16, function argument 6 */
  unsigned long a7;                /* a7: x17, function argument 7 */
  unsigned long t3;                /* t3: x28, temporary register 3 */
  unsigned long t4;                /* t4: x29, temporary register 4 */
  unsigned long t5;                /* t5: x30, temporary register 5 */
  unsigned long t6;                /* t6: x31, temporary register 6 */
#endif
} EXC_Frame_Type;

/* system clock frequency (core clock) */
extern uint32_t SystemCoreClock;

/* function declarations */
/* initialize the system and update the SystemCoreClock variable */
extern void SystemInit (void);
/* update the SystemCoreClock with current core clock retrieved from cpu registers */
extern void SystemCoreClockUpdate (void);
#ifdef __FIRMWARE_VERSION_DEFINE
/* get firmware version */
extern uint32_t gd32vw55x_firmware_version_get(void);
#endif /* __FIRMWARE_VERSION_DEFINE */

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_GD32VW55x_H */
