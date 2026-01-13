/*
 * shell.c
 *
 *  Created on: 7 juin 2019
 *      Author: Laurent Fiack
 */

#include "shell.h"
#include "FreeRTOS.h"
#include "queue.h"

#include <stdio.h>

#include "usart.h"
#include "gpio.h"
#include "cmsis_os.h"	// FreeRTOS

typedef struct{
	char c;
	int (* func)(int argc, char ** argv);  // Syntaxe pour un pointeur de fct
	char * description;
} shell_func_t;

// Déclaration du sémaphore
SemaphoreHandle_t sem_uart_rx;

static int shell_func_list_size = 0;
static shell_func_t shell_func_list[SHELL_FUNC_LIST_MAX_SIZE];  // Tableau de liste de fct crées (64 max)

static char print_buffer[BUFFER_SIZE];  // Buffer de taille 40

static char uart_read() {
	char c;

	//HAL_UART_Receive(&UART_DEVICE, (uint8_t*)(&c), 1, HAL_MAX_DELAY);
	HAL_UART_Receive_IT(&UART_DEVICE, (uint8_t*)(&c), 1);

	// bloquer la tâche jusqu'à reception de caractère
	// On utilise un sémaphore
	xSemaphoreTake(sem_uart_rx, portMAX_DELAY);

	return c;
}

static int uart_write(char * s, uint16_t size) {
	HAL_UART_Transmit(&UART_DEVICE, (uint8_t*)s, size, HAL_MAX_DELAY);
	return size;
}

static int sh_help(int argc, char ** argv) {
	int i;
	for(i = 0 ; i < shell_func_list_size ; i++) {
		int size;
		size = snprintf (print_buffer, BUFFER_SIZE, "%c: %s\r\n", shell_func_list[i].c, shell_func_list[i].description);  // Affiche le carractère : description
		uart_write(print_buffer, size);
	}

	return 0;
}

void shell_init() {
	int size = 0;

	// Création du sémaphore
	sem_uart_rx = xSemaphoreCreateBinary();
	vQueueAddToRegistry(sem_uart_rx, "UART_RX_SEM");  // Question 9 (Partie 3.3) pour nommer mon semaphore pour l'affichage de la mémoire dans le debugger

	size = snprintf (print_buffer, BUFFER_SIZE, "\r\n\r\n===== Monsieur Shell v0.2 =====\r\n");
	uart_write(print_buffer, size);

	shell_add('h', sh_help, "Help");
}

int shell_add(char c, int (* pfunc)(int argc, char ** argv), char * description) {
	if (shell_func_list_size < SHELL_FUNC_LIST_MAX_SIZE) {	// shell_func_list_size = 0 au début (= nombre de fct enregistré dans le shell)
		shell_func_list[shell_func_list_size].c = c;
		shell_func_list[shell_func_list_size].func = pfunc;
		shell_func_list[shell_func_list_size].description = description;
		shell_func_list_size++;
		return 0;
	}

	return -1;
}

static int shell_exec(char * buf) {
	int i;

	char c = buf[0];

	int argc;
	char * argv[ARGC_MAX];
	char *p;

	for(i = 0 ; i < shell_func_list_size ; i++) {
		if (shell_func_list[i].c == c) {
			argc = 1;
			argv[0] = buf;

			for(p = buf ; *p != '\0' && argc < ARGC_MAX ; p++){
				if(*p == ' ') {
					*p = '\0';
					argv[argc++] = p+1;
				}
			}

			return shell_func_list[i].func(argc, argv);
		}
	}

	int size;
	size = snprintf (print_buffer, BUFFER_SIZE, "%c: no such command\r\n", c);
	uart_write(print_buffer, size);
	return -1;
}


int shell_run() {
	int reading = 0;
	int pos = 0;

	static char cmd_buffer[BUFFER_SIZE];	// Variable pas dans la pile pour gagner de la memoire

	while (1) {
		uart_write("> ", 2);
		reading = 1;

		while(reading) {
			char c = uart_read();	// Attend de recevoir le carractère et return dans char c le caractère lu
			int size;

			switch (c) {
			//process RETURN key
			case '\r':
				//case '\n':
				size = snprintf (print_buffer, BUFFER_SIZE, "\r\n");
				uart_write(print_buffer, size);
				cmd_buffer[pos++] = 0;     //add \0 char at end of string
				size = snprintf (print_buffer, BUFFER_SIZE, ":%s\r\n", cmd_buffer);
				uart_write(print_buffer, size);
				reading = 0;        //exit read loop
				pos = 0;            //reset buffer
				break;
				//backspace
			case '\b':
				if (pos > 0) {      //is there a char to delete?
					pos--;          //remove it in buffer

					uart_write("\b \b", 3);	// delete the char on the terminal
				}
				break;
				//other characters
			default:
				//only store characters if buffer has space
				if (pos < BUFFER_SIZE) {
					uart_write(&c, 1);
					cmd_buffer[pos++] = c; //store
				}
			}
		}
		shell_exec(cmd_buffer);
	}
	return 0;
}

void shell_uart_rx_callback(void)
{
	BaseType_t hptw;
	xSemaphoreGiveFromISR(sem_uart_rx, &hptw);
	portYIELD_FROM_ISR(hptw);
}
