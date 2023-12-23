#include "dev.h"


static void DevInitInList(DoublyListItem_t *item, void *params)
{
	Dev_t *dev;

	dev = LIST_ENTRY(item, Dev_t, ListItem);
	DevInit(dev);
}

static void DevDeInitInList(DoublyListItem_t *item, void *params)
{
	Dev_t *dev;

	dev = LIST_ENTRY(item,Dev_t,ListItem);
	DevDeInit(dev);
}

static void *DevFindID(DoublyListItem_t *item, void *params)
{
	uint8_t id = *(uint32_t *)params;
	Dev_t *dev = LIST_ENTRY(item,Dev_t,ListItem);
	void *ret = NULL;

	if(dev->ID == id)
	{
		ret = dev;
	}

	return ret;
}

void DevInit(Dev_t *dev)
{
	if(NULL != dev->Init)
	{
		dev->Init();
	}
}

void DevDeInit(Dev_t *dev)
{
	if(NULL != dev->DeInit)
	{
		dev->DeInit();
	}
}

void DevInitAll(DoublyListItem_t *devList)
{
	DoublyListItemForeach(devList, DevInitInList, NULL);
}

void DevDeInitAll(DoublyListItem_t *devList)
{
	DoublyListItemForeach(devList, DevDeInitInList, NULL);
}


Dev_t *DevGet(DoublyListItem_t *devList, uint32_t id)
{
	return (Dev_t *)DoublyListItemForeach(devList, DevFindID, &id);
}


void DevRegister(DoublyListItem_t *devList, Dev_t *dev)
{
	DoublyListItemInsertAfter(devList, &dev->ListItem);
}

void DevUnregister(DoublyListItem_t *devList, Dev_t *dev)
{
	DoublyListItemRemove(&dev->ListItem);
}



