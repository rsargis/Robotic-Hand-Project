#include "command_interface.h"

void read_line(){
    uint8_t c = 0xFF;
    while(UART_GET_RX_EMPTY(UART0) == 0)
    {
        /* Get the character from UART Buffer */
        c = UART_READ(UART0);

        if (c == '\r' || c == '\n'){
            g_acRXBuffer[index] = '\0';
            command_parser(g_acRXBuffer);
            index = 0; 
        } else if (index < RXBUFSIZE - 1) {
            g_acRXBuffer[index] = c;
            index++;
        }
    }
}

void command_parser(char* line){
    
}