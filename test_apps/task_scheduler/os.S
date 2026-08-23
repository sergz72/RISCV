.global osTaskSwitch

.extern current_task

/*typedef struct
{
  unsigned long long int RSP;
  unsigned long long int sleep_to;
  bool is_active;
  void *stack;
} task_t;*/

.text

// new current task value is in rdi
osTaskSwitch:
  pushq %rbx
  pushq %rbp
  pushq %r12
  pushq %r13
  pushq %r14
  pushq %r15
  movq  current_task(%rip), %rbx
  movq  %rsp, (%rbx)
  movq  %rdi, current_task(%rip)
  movq  (%rdi), %rsp
  popq  %r15
  popq  %r14
  popq  %r13
  popq  %r12
  popq  %rbp
  popq  %rbx
  ret
