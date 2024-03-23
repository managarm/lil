#include <lil/imports.h>

#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

const bool logLilVerbose = true;

void lil_log(enum LilLogType type, const char *fmt, ...) {
	va_list args;
	va_start(args, fmt);

	switch(type) {
		case ERROR:
			printf("\e[31mlil: ");
			break;
		case WARNING:
			printf("\e[33mlil: ");
			break;
		case INFO:
			if(logLilVerbose) {
				printf("lil: ");
			} else {
				va_end(args);
				return;
			}
			break;
		case DEBUG:
		case VERBOSE:
			if(logLilVerbose) {
				printf("\e[37mlil: ");
			} else {
				va_end(args);
				return;
			}
			break;
	}

	vprintf(fmt, args);
	printf("\e[39m");

	va_end(args);
}

void lil_sleep(uint64_t ms) {
	usleep(ms * 1000);
}

void lil_usleep(uint64_t us) {
	usleep(us);
}

void lil_panic(const char* msg) {
	printf("PANIC: %s\n", msg);
	exit(1);
}
