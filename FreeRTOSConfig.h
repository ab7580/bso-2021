
/* We sleep a lot, so cooperative multitasking is fine. */
#define configUSE_PREEMPTION 0

/* Blink doesn't really need a lot of stack space! */
#define configMINIMAL_STACK_SIZE 128

/* Funkcijo vTaskList() omogocite tako, da v FreeRTOSConfig.h nastavite configUSE_TRACE_FACILITY in configUSE_STATS_FORMATTING_FUNCTIONS na 1. */
#define configUSE_TRACE_FACILITY 1
#define configUSE_STATS_FORMATTING_FUNCTIONS 1

/* Use the defaults for everything else */
#include_next<FreeRTOSConfig.h>

