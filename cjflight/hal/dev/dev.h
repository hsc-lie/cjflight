#ifndef __DEV_H_
#define __DEV_H_


#include "common.h"
#include "doubly_list.h"


typedef struct
{
    DoublyListItem_t ListItem;

	uint32_t ID;

	void (*Init)();
	void (*DeInit)();
}Dev_t;

extern void DevInit(Dev_t *dev);
extern void DevDeInit(Dev_t *dev);
extern void DevInitAll(DoublyListItem_t *devList);
extern void DevDeInitAll(DoublyListItem_t *devList);
extern Dev_t *DevGet(DoublyListItem_t *devList, uint32_t id);
extern Dev_t *DevGet(DoublyListItem_t *devList, uint32_t id);
extern void DevUnregister(DoublyListItem_t *devList, Dev_t *dev);
extern void DevRegister(DoublyListItem_t *devList, Dev_t *dev);


#endif /*__DEV_H_*/