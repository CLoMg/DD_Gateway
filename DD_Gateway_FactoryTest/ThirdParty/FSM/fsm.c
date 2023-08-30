#include "fsm.h"
#include "stdlib.h"

void FSM_Regist(FSM_T* pFSM,StateTransform_T* pTable)
{
	pFSM->pTransTable = pTable;
}
void FSM_StateTransfer(FSM_T* pFSM,States_T next_state)
{
	pFSM->state = next_state;
}


StateTransform_T* FindTrans(FSM_T* pFSM, const EventsID_T evt){
	unsigned char i=0;
	for(;i< pFSM->transform_num;++i){
		if((pFSM->state == pFSM->pTransTable[i].CurState) && (evt == pFSM->pTransTable[i].EventID))
			return &pFSM->pTransTable[i];
	}
	return NULL;
}

void FSM_EventHandle(FSM_T* pFSM,EventsID_T evt){
	StateTransform_T* pActTrans = NULL;
	pActTrans = FindTrans(pFSM,evt);
	if(pActTrans !=NULL)
	{
		FSM_StateTransfer(pFSM,pActTrans->NextState);
		pActTrans->Actfunc();
	}
}
