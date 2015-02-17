

//TODO: implement deletion
//TODO: implement lock functions using mutex

#include "CirDLinkedList.h"



CirLinkList* CirLinkList_init(){
    CirLinkList* lst = malloc(sizeof(CirLinkList));
    lst->head=NULL;
    lst->cur=NULL;
    lst->size=0;
    return lst;
}

//Add to Top
void CirLinkList_push(CirLinkList* lst, void *data){
    if(lst && data){
        DLinkList_node* node = malloc(sizeof(DLinkList_node));
        node->data=data;
        lst->size++;
        
        //list empty
        if(lst->head==NULL){
            lst->head=node;
            node->prev=node;
            node->next=node;
            return;
        }

        //only one item in list
        if(lst->head->next==lst->head){
            lst->head->prev=node;
            node->next=lst->head;
            lst->head->next=node;
            node->prev=lst->head;
            lst->head=node;
            return;
        }

        //Two or more nodes in list
        node->next=lst->head;
        node->prev=lst->head->prev;
        lst->head->prev=node;
        node->prev->next=node;
        lst->head=node;
        return;
    }
}

void* CirLinkList_peek(CirLinkList* lst){
    //empty
    if(lst->head==NULL)
        return NULL;

    return lst->head->data;

}

//Increment Head to next node -- Used for scanning etc
void* CirLinkList_inc(CirLinkList* lst){
    
    //empty
    if(lst->head==NULL)
        return NULL;
        
    //first inc or dec
    if(lst->cur==NULL){
        lst->cur=lst->head;
        return lst->cur->data;
    }
    
    lst->cur = lst->cur->next; //increment to next element
    
    return lst->cur->data;
}

//Decrement Head to prev node -- Used for scanning etc
void* CirLinkList_dec(CirLinkList* lst){
    
    //empty
    if(lst->head==NULL)
        return NULL;

    //first inc or dec
    if(lst->cur==NULL){
        lst->cur=lst->head;
        return lst->cur->data;
    }

    lst->cur = lst->cur->prev; //increment to next element
    
    return lst->cur->data;
}



void* CirLinkList_get(CirLinkList* lst, uint64_t idx){
    DLinkList_node* n;
    uint64_t x;
    
    n=lst->head;

    //empty
    if(lst->head==NULL)
        return NULL;
    
    idx%=lst->size;//stop exccesive looping around list
    
    for(x=0;x<idx;x++){
        n=n->next;
    }
    return n->data;
}

void* CirLinkList_del(CirLinkList* lst, uint64_t idx){
    DLinkList_node* n;
    void* data;
    uint64_t x;
    
    n=lst->head;

    //empty
    if(lst->head==NULL)
        return NULL;
    
    idx%=lst->size;//stop exccesive looping around lis
    
    for(x=0;x<idx;x++){
        n=n->next;
    }
    //last node
    if(n->next == n && n->prev == n){
        lst->head=NULL;
        lst->cur=NULL;
    }else{
    //relink list
        n->prev->next = n->next;
        n->next->prev = n->prev;
    }
    if(lst->head == n) lst->head=n->next;
    if(lst->cur  == n)  lst->cur=n->next;
    
    data = n->data;

    lst->size--;
    free(n);
    
    return data;
}
