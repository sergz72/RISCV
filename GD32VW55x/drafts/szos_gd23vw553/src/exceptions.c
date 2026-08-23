#include "exceptions.h"
#include <common_printf.h>
#include "os.h"

task_data tasks[MAX_TASKS];
task_data *current_task_data;

unsigned int exception_stack[EXCEPTION_STACK_SIZE] __attribute__((aligned(16)));

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
    os_delay();
    return;
  case MmodeEcall_EXCn:
    if (!current_task_data->registers[16]) // a7
    {
      os_delay();
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

void Exception_Init(void)
{
  current_task_data = tasks;
  current_task_data->is_active = true;
}

void Exception_Register_EXC(uint32_t EXCn, unsigned long exc_handler)
{
}
