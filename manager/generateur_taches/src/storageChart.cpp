#include "storage.h"
#include "storageChart.h"

using namespace std;

bool productInStock(Storage tabStock[]){
	bool tmp = false;
	for(int i=0;i<SIZE;i++)
	{
		if((tabStock[i].getBeginningDelivery() != 0) || (tabStock[i].getEndDelivery() != 0))
		{
			tmp = true;
		}
	}
	return tmp;
}


int findId(Storage tabStock[],int debut, int fin){
	int id = -1;
	for(int i=0; i<SIZE; i++)
	{
		if((tabStock[i].getBeginningDelivery() == debut) && (tabStock[i].getEndDelivery() == fin))
		{
			id = i;
		}
	}
	return id;
}
