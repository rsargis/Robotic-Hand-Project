/*Function prototypes for command parsing*/
#ifndef COMMAND_INTERFACE_H
#define COMMAND_INTERFACE_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "NuMicro.h"
#include "motion_control.h"

#define RXBUFSIZE   1024

void read_line(void);
void command_parser(char* line);

#endif