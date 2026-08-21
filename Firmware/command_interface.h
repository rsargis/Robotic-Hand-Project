/*Function prototypes for command parsing*/
#ifndef COMMAND_INTERFACE_H
#define COMMAND_INTERFACE_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "NuMicro.h"

#define RXBUFSIZE   1024
char g_acRXBuffer[RXBUFSIZE];
int index = 0;

void read_line();
void command_parser(char* line);

#endif