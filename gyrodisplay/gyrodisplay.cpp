#include <stdio.h>
#include <windows.h>
#include <conio.h>
#include <stdlib.h>
#include "serial.h"

int main(void)
{
	serial_t obj = serial_create("COM1", 9600);
	unsigned char buf[128], len;

	if (obj == NULL) {
		fprintf(stderr, "オブジェクト生成に失敗");
		return EXIT_FAILURE;
	}

	while (1) {
		len = serial_recv(obj, buf, sizeof(buf));
		if (len) serial_send(obj, buf, len);
		Sleep(1);
		if (_kbhit())  break;
	}

	serial_delete(obj);

	return EXIT_SUCCESS;
}