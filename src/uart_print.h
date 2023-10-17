void setup_uart_printer(void);

/*! 
 * 	Supports %s (string), %d (signed decimal), %u (unsigned decimal), and %f (float)
 * FLOATS SHOULD BE PASSED AS A POINTER TO FLOAT - THIS IS DUE TO A C LANGUAGE LIMITATION 
 * COMBINED WITH NO HARDWARE (double)
 * Floats are limited in magnitude to UINT32_MAX
 * Floats only print a fixed 3 decimal places
 * */
void printlf(char format[], ...);

void print_string(const char * const str);