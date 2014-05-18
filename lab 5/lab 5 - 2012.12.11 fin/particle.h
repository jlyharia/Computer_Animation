#ifndef PARTICLE_H_INCLUDED
#define PARTICLE_H_INCLUDED
#include <vector>
#include "trivial.h"
using namespace std;
class particle
{
    public:
        particle();

        virtual ~particle();
        vector <float> p;//position
        vector <float> v;//velocity
        vector <float> a;//acceleration

        void setPos(float,float,float);
        void setPos(vector <float> );
        void setVel(float,float,float);
        void setVel(vector <float>);

        void draw();
        //draw particle
        void addLife(float);
        //add particle's life
        float getLife(){return life;}
        //return life time
        float getLifeLimit(){return life_limit;}
        //return life limit
        void m_life_time();
        void m_firework_life_time_limit();
        void setParType(bool);
        //set particle type//true -> shooting particle//false -> fireworks
        bool getParType();
        void setParColor(vector <float>);
    private:
        float life;
        float life_limit;
        bool par_Type;
        vector <float> par_color;
};
#endif // PARTICLE_H_INCLUDED

