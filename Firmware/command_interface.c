#include "command_interface.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdint.h>
#include "NuMicro.h"
#include "motion_control.h"

char g_acRXBuffer[RXBUFSIZE];
int index = 0;

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
    if (strcmp("OPEN", line) == 0){
        motion_set_pose(&POSE_OPEN_HAND);
        return;
    }
    else if(strcmp("CLOSE", line) == 0){
        motion_set_pose(&POSE_CLOSED_HAND);
        return;
    }
    else if(strcmp("NEUTRAL", line) == 0){
        motion_set_pose(&POSE_NEUTRAL);
        return;
    }
    else if(strcmp("MIDDLE", line) == 0){
        motion_set_pose(&POSE_MIDDLE);
        return;
    }
    else if (strcmp("OK", line) == 0){
        motion_set_pose(&POSE_OK);
        return;
    }
    else if (strcmp("VICTORY", line) == 0){
        motion_set_pose(&POSE_VICTORY);
        return;
    }
    else if(strncmp("STATUS", line, 6) == 0){
        int id = 0;

        if (sscanf(line, "STATUS %d", &id) != 1 || id < 1 || id > 8) {
            printf("ERROR, INVALID ID\n");
            return;
        }

        SCS_Status_t status;
        int res = scs_get_status((uint8_t)id, &status);
        if (res == SCS_OK){
            printf("  -> position: %u speed: %u voltage: %u temperature: %u\n", 
                   status.position, status.speed, status.voltage, status.temperature);
        }
        else{
            printf("  -> status read failed (error code: %d)\n", res);
        }
    }
    else if(strncmp("POSE", line, 4) == 0){
        hand_pose_t pose;
        if (sscanf(line, "POSE %hd %hd %hd %hd %hd %hd %hd %hd", 
                   &pose.angle[0], &pose.angle[1], &pose.angle[2], &pose.angle[3],
                   &pose.angle[4], &pose.angle[5], &pose.angle[6], &pose.angle[7]) == 8) {
            clamp_pose(&pose);
            motion_set_pose(&pose);
        } 
        else {
            printf("ERROR, INVALID POSE COMMAND\n");
        }
    }
    else if(strcmp("HELP", line) == 0){
        printf("Valid commands: OPEN, CLOSE, NEUTRAL, MIDDLE, OK, VICTORY, STATUS <id>, POSE <angle0> ... <angle7>\n");
    }
    else{
        printf("ERROR, INVALID COMMAND\n");
        printf("Valid commands: OPEN, CLOSE, NEUTRAL, MIDDLE, OK, VICTORY, STATUS <id>, POSE <angle0> ... <angle7>\n");
    }
}