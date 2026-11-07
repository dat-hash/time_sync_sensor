#include "debug_print.h"
#include <stdio.h>
#include "usart.h"

void debug_print_init(void) {
    setvbuf(stdout, NULL, _IONBF, 0); // whyyyyyyy :( this just to make printf actually print right away
}

// weird step but now this hijack the printf so trick it to so uart transmit
// file 0 = stdin input (scanf, getchar) aka dont care
// file 1 = stdout normal output: printf, puts, fwrite(..., stdout), aka i do care
// file 2 = stderr error output aka i do care to know what gone wrong
// file 3+ = files anything opened with open()/fopen() aka these dont even exist here as there no system files
// must return lenght somehow
int _write(int file, char *data, int length)
{
    if (file == 1 || file == 2) {
        HAL_UART_Transmit(&huart3, (uint8_t *)data, (uint16_t)length, 100u);
    }
    return length;
}