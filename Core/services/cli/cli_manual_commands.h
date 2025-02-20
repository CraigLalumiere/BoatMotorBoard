#ifndef APP_CLI_MANUAL_COMMANDS_H
#define APP_CLI_MANUAL_COMMANDS_H

#include "embedded_cli.h"

void on_cli_digital_out_set(EmbeddedCli *cli, char *args, void *context);
void on_cli_digital_in_read(EmbeddedCli *cli, char *args, void *context);
void on_cli_spi(EmbeddedCli *cli, char *args, void *context);
void on_cli_i2c(EmbeddedCli *cli, char *args, void *context);
void on_print_reset_reason(EmbeddedCli *cli, char *args, void *context);
void on_do_q_assert(EmbeddedCli *cli, char *args, void *context);
void on_print_version(EmbeddedCli *cli, char *args, void *context);
void on_cli_adc_sample(EmbeddedCli *cli, char *args, void *context);

#endif // APP_CLI_MANUAL_COMMANDS_H