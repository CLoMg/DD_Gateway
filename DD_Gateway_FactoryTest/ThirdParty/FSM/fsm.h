#ifndef FSM_H
#define FSM_H

typedef unsigned char States_T ;
typedef unsigned char EventsID_T;

typedef struct Event_Queue{
	unsigned char size;
	unsigned char event;
	struct Event_Queue* next;
}Event_Queue_T;

typedef struct{
	States_T CurState;
	EventsID_T EventID;
	States_T NextState;
	void (*Actfunc)();
}StateTransform_T;

typedef struct{
	unsigned char	state;
	int transform_num;
	StateTransform_T *pTransTable;
}FSM_T;


void FSM_Regist(FSM_T* pFSM,StateTransform_T* pTable,unsigned char trans_num,States_T init_state);
void FSM_StateTransfer(FSM_T*,States_T);
void FSM_EventHandle(FSM_T* pFSM,const EventsID_T);
void LED_Process(void);

#endif

