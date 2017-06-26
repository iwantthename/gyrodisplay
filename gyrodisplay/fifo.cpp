#include <stdlib.h>
#include "fifo.h"

/* インスタンス生成 */
fifo_t *fifo_create(void)
{
	fifo_t *obj = (fifo_t *)malloc(sizeof(fifo_t));
	if (obj == NULL)	return NULL;

	obj->read = obj->write = 0;
	obj->size = FIFO_BUFSIZE;

	return obj;
}

/* インスタンス消去 */
void fifo_delete(fifo_t *obj)
{
	free(obj);
}

/* データを書き込む. 戻り値：書き込めたバイト数 */
unsigned int fifo_write(fifo_t *obj, unsigned char *buf, unsigned int size)
{
	unsigned int ret = 0;
	unsigned int next = (obj->write + 1) % obj->size;

	while (next != obj->read && ret < size) {
		obj->buf[obj->write] = buf[ret];
		obj->write = next;
		next = (obj->write + 1) % obj->size;
		ret++;
	}

	return ret;
}

/* データを読み込む．戻り値：読み込めたバイト数 */
unsigned int fifo_read(fifo_t *obj, unsigned char *buf, unsigned int size)
{
	unsigned int ret = 0;

	while (obj->read != obj->write && ret < size) {
		buf[ret] = obj->buf[obj->read];
		obj->read = (obj->read + 1) % obj->size;
		ret++;
	}

	return ret;
}

/* 格納されているデータ数を取得する */
unsigned int fifo_length(fifo_t *obj)
{
	return (obj->size + obj->write - obj->read) % obj->size;
}