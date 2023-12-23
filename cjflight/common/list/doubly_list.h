#ifndef __DOUBLY_LIST_H_
#define __DOUBLY_LIST_H_

#include "list_common.h"

typedef struct DoublyListItem_t
{
    struct DoublyListItem_t *Next;  
    struct DoublyListItem_t *Prev;
}DoublyListItem_t;

typedef void *(*DoublyListHandleFunc_t)(DoublyListItem_t *item, void *params);

extern void DoublyListItemInit(DoublyListItem_t *list);

extern void DoublyListItemInsertAfter(DoublyListItem_t *list, DoublyListItem_t *item);
extern void DoublyListItemInsertBefore(DoublyListItem_t *list, DoublyListItem_t *item);

extern void DoublyListItemRemove(DoublyListItem_t *item);

extern uint32_t DoublyListItemGetLen(DoublyListItem_t *list);

extern void *DoublyListItemForeach(DoublyListItem_t *list, DoublyListHandleFunc_t func, void *params);

#endif /*__DOUBLY_LIST_H_*/