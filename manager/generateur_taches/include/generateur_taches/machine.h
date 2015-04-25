#ifndef MACHINE_H
#define MACHINE_H

#include "robot.h"

class Machine{

        public:
                Machine();
                
                int get_x(){return m_x;}
                void set_x(int x){m_x=x;}
                int get_y(){return m_y;}
                void set_y(int y){m_y=y;}
                bool get_traite(){return m_traite;}
                void set_traite(bool traite){m_traite=traite;}
                int get_robot(){return m_robot;}
                
                void correspondance_zone();
                void update_machine(Robot tab_robot[]);

        private:
                float m_x;
                float m_y;
                int m_robot;
                bool m_traite;
};

#endif
