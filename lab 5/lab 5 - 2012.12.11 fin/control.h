#ifndef CONTROL_H
#define CONTROL_H
#include "particle.h"
#include <vector>
#include <GL/glut.h>
using namespace std;

class control
{
    public:
        control(float);
        virtual ~control();
        float del_t;//time frame of particle system
        void update(float);
        //update
        vector<particle> par;
        void m_Rand_fireworks_color(vector< vector<float> >);
        //put some color sets
        void m_setRand_fireVel(vector< vector<float> >);
        //put random initial fireworks velocity
        void m_preference(int, int);
        //user control preference
    private:

        void m_gravity(float);
        //process gravity law
        vector<float> m_Rand_Par_Vel();
        //random particle initial velocity
        void m_newParticle(int);
        //create new particle for shooting
        void m_NewParticle_Time_limit(int);
        //set shooting time interval for shooting particle
        bool TimeFunc_NewParticle();
        //time function, return if it is time to generate next shooting
        vector< vector<float> > m_check_par_life();
        //check particle life time, if reach its limit, then die
        void m_newFireworks(vector< vector<float> >, int);
        //generate particle for fireworks
        float par_radius;//particle radius
        float time_interval;
        float interval_time_limit;
        int Num_fireworks;
        vector< vector<float> > Rand_firework_vel;
        vector< vector<float> > Rand_firework_col;
};

#endif // CONTROL_H
