#include <stdio.h>
#include "freertps/freertps.h"
#include <string.h>

int talker_main(int argc, char **argv)
{
  printf("hello, world!\r\n");
  freertps_system_init();
  frudp_pub_t *pub = freertps_create_pub
                       ("chatter",
                        "std_msgs::msg::dds_::String_");
  int pub_count = 0;
  frudp_disco_start();
  while (freertps_system_ok())
  {
    frudp_listen(50000);
    frudp_disco_tick();
    char msg[256] = {0};
    snprintf(&msg[4], sizeof(msg) - 4, "Hello World: %d", pub_count++);
    uint32_t rtps_string_len = strlen(&msg[4]) + 1;
    *((uint32_t *)msg) = rtps_string_len;
    freertps_publish(pub, (uint8_t *)msg, rtps_string_len + 4);
    os_printf("sending: [%s]\n", &msg[4]);
    vTaskDelay(1000);
  }
  frudp_fini();
  return 0;
}

