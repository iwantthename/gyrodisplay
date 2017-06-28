#include <stdio.h>
#include <windows.h>
#include <conio.h>
#include <stdlib.h>
#include "serial.h"

int main(void)
{
	serial_t obj = serial_create("COM7", 9600);
	unsigned char buf[512], send_str[1] = {'a'}, len;

	

	if (obj == NULL) {
		fprintf(stderr, "オブジェクト生成に失敗");
		return EXIT_FAILURE;
	}

	while (1) {
		len = serial_recv(obj, buf, sizeof(buf));
		printf("%s", buf);
		for (int i = 0; buf[i] != '\0'; i++)printf("%x ", buf[i]);
		printf("\n\n");
		if (len) {
			serial_send(obj, send_str, sizeof(send_str));
		}
		Sleep(100);
	}

	serial_delete(obj);

	return EXIT_SUCCESS;
}