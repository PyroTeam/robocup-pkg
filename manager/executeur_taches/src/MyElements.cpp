#include "MyElements.h"

/* Constructeur */
MyElements::MyElements(){
}

/* Destructeur */
MyElements::~MyElements(){}

/* Méthodes */
BaseStation MyElements::getBS(){
  return this->bs;
}
RingStation MyElements::getRS1(){
  return this->rs1;
}
RingStation MyElements::getRS2(){
  return this->rs2;
}
CapStation MyElements::getCS1(){
  return this->cs1;
}
CapStation MyElements::getCS2(){
  return this->cs2;
}
DeliveryStation MyElements::getDS(){
  return this->ds;
}