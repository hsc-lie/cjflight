#include "doubly_list.h"

void DoublyListItemInit(DoublyListItem_t *item)
{
    item->Next = item; 
    item->Prev = item;
}

void DoublyListItemInsertAfter(DoublyListItem_t *item, DoublyListItem_t *new)
{
    new->Next = item->Next;
    new->Prev = item;

    item->Next = new;
    item->Next->Prev = new;
}

void DoublyListItemInsertBefore(DoublyListItem_t *item, DoublyListItem_t *new)
{
    new->Next = item;
    new->Prev = item->Prev;

    item->Prev->Next = new;
    item->Prev = new;
}

void DoublyListItemRemove(DoublyListItem_t *item)
{
    item->Next->Prev = item->Prev;
    item->Prev->Next = item->Next;

    item->Next = item;
    item->Prev = item;
}

uint32_t DoublyListItemGetLen(DoublyListItem_t *item)
{
    uint32_t len = 0;
    DoublyListItem_t *p = item;

    while (p->Next != item)
    {
        p = p->Next;
        ++len;
    }

    return len;
}

void *DoublyListItemForeach(DoublyListItem_t *item, DoublyListHandleFunc_t func, void *params)
{
	DoublyListItem_t *p = item;
	void *ret;

	while(p->Next != item)
	{
		ret = func(p->Next, params);
		if(NULL != ret)
		{
			return ret;
		}
		p = p->Next;
	}

	return NULL;
}