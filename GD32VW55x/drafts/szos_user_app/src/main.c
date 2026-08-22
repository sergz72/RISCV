#include <syscalls.h>

static bool led_state = true;

void __attribute__((naked)) main(int argc, const char **argv)
{
  printf("argc=%d\n", argc);
  for (int i = 0; i < argc; i++)
    printf("argv[%d]=%s\n", i, argv[i]);
  for (int i = 0; i < 10; i++)
  {
    led_state = !led_state;
    osLeds(led_state, 1);
    osDelay(500);
  }
  osExit(0);
}
