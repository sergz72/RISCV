#include "board.h"
#include "exceptions.h"
#include <common_printf.h>
//#include <stdio.h>
#include "sys_timer.h"

typedef struct
{
  unsigned int mcause;
  unsigned int registers[31];
  unsigned int mepc;
  unsigned int mstatus;
  bool is_active;
  unsigned long long int sleep_to;
  pmp_config pmp_config_code, pmp_config_data;
} task_data;

static task_data tasks[MAX_TASKS];
task_data *current_task_data = tasks;

unsigned int exception_stack[EXCEPTION_STACK_SIZE] __attribute__((aligned(16)));

static void task_switch(void)
{
  delayms(1000);
  PUTS("Task switch\n");
}

void exc_handler(void)
{
  unsigned int mcause = current_task_data->mcause & MCAUSE_CAUSE;

  const char *cause;
  switch (mcause) {
  case InsUnalign_EXCn:
    cause = "Instruction address misaligned";
    break;
  case Break_EXCn:
    cause = "Breakpoint";
    break;
  case LdAddrUnalign_EXCn:
    cause = "Load address misaligned";
    break;
  case StAddrUnalign_EXCn:
    cause = "Store address misaligned";
    break;
  case NMI_EXCn:
    cause = "Non-maskable interrupt";
    break;
  case IlleIns_EXCn:
    cause = "Illegal instruction fault";
    break;
  case UmodeEcall_EXCn:
    task_switch();
    return;
  case MmodeEcall_EXCn:
    if (!current_task_data->registers[16]) // a7
    {
      task_switch();
      return;
    }
    cause = "Environment call from M-mode";
    break;
  case InsAccFault_EXCn:
    cause = "Instruction access fault";
    break;
  case LdFault_EXCn:
    cause = "Load access fault";
    break;
  case StAccessFault_EXCn:
    cause = "Store access fault";
    break;
  default:
    cause = "Unknown exception";
    break;
  }

  CSR_MSTATUS_Type mstatus;
  mstatus.d = __RV_CSR_READ(CSR_MSTATUS);
  PRINTF("\n%s exception from mode %d, mcause: 0x%x, mepc: 0x%x. Rebooting...\n", cause, mstatus.b.mpp, current_task_data->mcause, current_task_data->mepc);
  reboot();
}

void ecall_handler(unsigned int a0, unsigned int a1, unsigned int a2, unsigned int a3, unsigned int a4, unsigned int a5,
                   unsigned int a6, unsigned int a7)
{
  switch (a7)
  {
    case 1:
      PUTS("Task switch complete. Rebooting...\n");
      reboot();
      break;
    default:
      PRINTF("Unknown environment call %x from U-mode. Rebooting...\n", a7);
      reboot();
  }
}

void nmi_handler(unsigned long mcause, unsigned long sp)
{
}

void Exception_Init(void)
{
  //asm volatile ("csrw mscratch, %0" : : "r" (&exception_stack[EXCEPTION_STACK_SIZE]));
}

void Exception_Register_EXC(uint32_t EXCn, unsigned long exc_handler)
{
}
