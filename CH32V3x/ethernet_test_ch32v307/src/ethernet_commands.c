#include "ethernet_commands.h"
#include <shell.h>
#include <eth_udp.h>
#include <eth_icmpv6.h>
#include <stdlib.h>
#include <string.h>

#include "eth_queue.h"

static int udp_send_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem udp_send_command_items[] = {
  {nullptr, param_handler, nullptr},
  {nullptr, param_handler, nullptr},
  {nullptr, param_handler, nullptr},
  {nullptr, nullptr, udp_send_handler}
};
static const ShellCommand udp_send_command = {
  udp_send_command_items,
  "udp_send",
  "udp_send address port data"
};

static int ns_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data);
static const ShellCommandItem ns_command_items[] = {
  {nullptr, param_handler, nullptr},
  {nullptr, nullptr, ns_handler}
};
static const ShellCommand ns_command = {
  ns_command_items,
  "ns",
  "ns address"
};

static int udp_send_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  unsigned char address[16];
  int port;

  int rc = ETH_Parse_IPV6(argv[0], address);
  if (rc)
  {
    pfunc("Incorrect address\n");
    return rc;
  }
  port = atoi(argv[1]);
  if (port <= 0 || port > 65535)
  {
    pfunc("Incorrect port\n");
    return 1;
  }

  rc = ETH_UDP_Send(1024, address, port, argv[2], strlen(argv[2]), &eth_user_queue);
  pfunc("ETH_UDP_Send rc=%d\n", rc);
  return rc;
}

static int ns_handler(printf_func pfunc, gets_func gfunc, int argc, char **argv, void *data)
{
  unsigned char address[16];

  int rc = ETH_Parse_IPV6(argv[0], address);
  if (rc)
  {
    pfunc("Incorrect address\n");
    return rc;
  }
  ETH_NS_Send(address);
  return 0;
}

void register_ethernet_commands(void)
{
  shell_register_command(&udp_send_command);
  shell_register_command(&ns_command);
}