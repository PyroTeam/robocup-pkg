#ifndef WORK_H
#define WORK_H

#include "task.h"
#include "order.h"

bool alreadyInWork(std::list<std::list<Task> > work, Order order);
std::list<std::list<Task> >::iterator maxRatio(std::list<std::list<Task> > &work);
bool positiveRatio(std::list<std::list<Task> > work);
void addInWork(std::list<std::list<Task> > &work, Order &order, int &availableCap);
void ratioCalculus(std::list<std::list<Task> > &work,double time,int robot,bool take[3]);
void getInfoWork(std::list<std::list<Task> > work);
void cleanWork(std::list<std::list<Task> > &work,std::list<std::list<Task> >::iterator &it, double time);
void particularTasksInWork(std::list<std::list<Task> > ::iterator &it, int &availableCap, int &storage, double time);
void finishedTasks(std::list<std::list<Task> > &work, int robot, double time);

#endif
