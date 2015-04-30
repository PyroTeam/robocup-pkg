#include "storage.h"
#include "storageChart.h"

using namespace std;

bool productInStock(Storage tabStock[]){
	bool tmp = false;
	for(int i=0;i<SIZE;i++)
	{
		if((tabStock[i].getProduct() != 0) && (tabStock[i].getProduct() != 10))
		{
			tmp = true;
		}
	}
	return tmp;
}


int findId(Storage tabStock[],int debut, int fin){
	for(int i=0; i<SIZE; i++)
	{
		if((tabStock[i].getBeginningDelivery() == debut) && (tabStock[i].getEndDelivery() == fin))
		{
			return tabStock[i].getId();
		}
	}
	return -3;
}
