#include "board.h"
#include "exceptions.h"
#include <common_printf.h>

void nmi_handler(unsigned long mcause, unsigned long sp)
{
  EXC_Frame_Type *exc_frame = (EXC_Frame_Type *)sp;
  common_printf("NMI Exception, mepc: 0x%lx. Rebooting...\r\n", exc_frame->mepc);
  reboot();
}

static void exception_handler(unsigned long mcause, unsigned long sp)
{
  EXC_Frame_Type *exc_frame = (EXC_Frame_Type *)sp;
  mcause &= MCAUSE_CAUSE;

  if (mcause == UmodeEcall_EXCn)
  {
    switch (exc_frame->a7)
    {
      default:
        common_printf("Unknown environment call %x from U-mode. Rebooting...\r\n", exc_frame->a7);
        reboot();
    }
  }

  const char *cause;
  switch (mcause & MCAUSE_CAUSE) {
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
    case MmodeEcall_EXCn:
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
  common_printf("\r\n%s exception from mode %d, mcause: 0x%lx, mepc: 0x%lx\r\n", cause, mstatus.b.mpp, exc_frame->mcause, exc_frame->mepc);
  switch (mstatus.b.mpp)
  {
    case PRV_U:
      puts_("Rebooting...\r\n");
      reboot();
    default:
      puts_("Rebooting...\r\n");
      reboot();
  }
}

uint32_t core_exception_handler(unsigned long mcause, unsigned long sp)
{
  uint32_t EXCn = mcause & 0X00000fff;

  if (EXCn == NMI_EXCn)
    nmi_handler(mcause, sp);
  else
    exception_handler(mcause, sp);
  return 0;
}

void Exception_Init(void)
{
}

void Exception_Register_EXC(uint32_t EXCn, unsigned long exc_handler)
{
}

