
#ifndef __CIRDLINKEDLIST_H
#define __CIRDLINKEDLIST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

//private struct
typedef struct DLinkList_node{
    struct DLinkList_node* prev;
    struct DLinkList_node* next;

    void* data;

}DLinkList_node;

typedef struct CirLinkList{
    DLinkList_node * head;
    DLinkList_node * cur; //used for inc and dec 
    uint64_t size;
}CirLinkList;


CirLinkList* CirLinkList_init();
void CirLinkList_push(CirLinkList* lst, void *data);

void* CirLinkList_get(CirLinkList* lst, uint64_t idx);
void* CirLinkList_del(CirLinkList* lst, uint64_t idx);

void* CirLinkList_peek(CirLinkList* lst);
void* CirLinkList_inc(CirLinkList* lst);
void* CirLinkList_dec(CirLinkList* lst);



#ifdef __cplusplus
}
#endif

#endif /* __CIRDLINKEDLIST_H */
