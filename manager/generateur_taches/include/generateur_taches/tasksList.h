#ifndef TASKSLIST_H
#define TASKSLIST_H

#include <list>
#include "task.h"
#include "product.h"

std::list<Task> creationListTasksProduct(Product product,int beginningDelivery,int endDelivery);
std::list<Task> creationListTasksAction(int action,Product product,int beginningDelivery,int endDelivery);
bool uncapInWork(std::list<Task> list);

#endif
