#ifndef _SERIAL_H_
#define _SERIAL_H_

typedef struct _TAG_SERIAL* serial_t;

/* 関数プロトタイプ */
serial_t serial_create(char *pname, unsigned int baud);
void serial_delete(serial_t obj);
unsigned int serial_send(serial_t obj, unsigned char *buf, unsigned int size);
unsigned int serial_recv(serial_t obj, unsigned char *buf, unsigned int size);
unsigned int serial_recv_length(serial_t obj);

#endif /* _SERIAL_H_ */