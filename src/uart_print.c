#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <driverlib/rom.h>
#include <driverlib/uart.h>
#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>
#include <inc/hw_uart.h>
#include <inc/hw_sysctl.h>
#include <inc/hw_memmap.h>
#include <common/tm4c123gh6pm.h>

#define INFINITY 1.0f/0.f

void setup_uart_printer(void){
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)){};
	
	ROM_GPIOPadConfigSet(GPIO_PORTA_BASE, 3, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
	ROM_GPIODirModeSet(GPIO_PORTA_BASE, 3, GPIO_DIR_MODE_HW);
	ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 921600,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_TWO | UART_CONFIG_PAR_NONE));
}

void putchar(char c) {
	static int chars_sent_recently = 0;
	ROM_UARTCharPut(UART0_BASE, c);
	while(ROM_UARTBusy(UART0_BASE)) {};
	
	//Really dumb, but avoid overwhelming ICDI
	if(++chars_sent_recently > 7) {
		chars_sent_recently = 0;
		for(int i = 0; i < 150; i++) {
			__asm("mov r1,r1\n");
		}
	}
}

void print_string(const char * const str) {
	for(int i = 0; str[i] != '\0'; i++) {
		putchar( str[i]);
	}
}

void print_unsigned_decimal(uint32_t num){
	
	char buf[11]; //Large enough to fit any value of num
	
	int places = 0;
	
	do {
		buf[places++] = (char)('0' + (num % 10));
		num /= 10;
	} while (num > 0);
	
	for(; places; places--) {
		putchar( buf[places-1]);
	}
}

void print_decimal(int32_t num){
	
	if (num < 0) {
		putchar( '-');
		num = 0 - num;
	}
	
	print_unsigned_decimal((uint32_t)num);
}

void print_float(float number) {
	
	if(number != number) { //NaN is not equal to anything, including NaN
		print_string("NaN");
		return;
	}
	
	if(number == INFINITY || number == -INFINITY) {
		print_string("inf");
		return;
	}
	
	if(number > UINT32_MAX || (0-number) > UINT32_MAX) {
		print_string("[out of range]");
		return;
	}
	
	char buf[15]; // large enough to fit any value
	
	if(number < 0) {
		putchar( '-');
		number = 0 - number;
	}
	
	uint32_t integerPart = (uint32_t)number;
	uint32_t decimalPart = (uint32_t)((number - integerPart) * 1000);
	
	int places = 0;
	
	for(int i = 0; i < 3; i++) {
		buf[places++] = (char)('0' + decimalPart % 10);
		decimalPart /= 10;
	};
	
	buf[places++] = '.';
	
	print_unsigned_decimal(integerPart);
	
	for(; places; places--) {
		putchar( buf[places-1]);
	}
}

void printlf(char format[], ...) {
	va_list args;
	va_start(args, format);
	
	char *str;
	int32_t num;
	uint32_t numu;
	float *numf;
	
	for(int i=0; format[i] != '\0'; i++) {
		switch(format[i]) {
			case '%' :
				i++;
				switch(format[i]) {
					case 'u': //unsigned decimal number
						numu = va_arg(args, uint32_t);
						print_unsigned_decimal(numu);
					break;
					
					case 'd': //signed decimal number
						num = va_arg(args, int32_t);
						print_decimal(num);
					break;
					
					case '\0': // End of string
						putchar( '%');
						i--; //let the for loop catch this
					break;
					
					case 's': //string
						str = va_arg(args, char*);
						print_string(str);
					break;
					
					case 'f': //float
						numf = va_arg(args, float *);
						print_float(*numf);
					break;
					
					default: //Not recognized
					putchar( '%');
					putchar( format[i]);
					break;
				}
			break;
				
			default:
				putchar( format[i]);
			break;
		}
	}
	
	va_end(args);
}
