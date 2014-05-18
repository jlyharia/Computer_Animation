#include "control.h"
#include "particle.h"
#include "trivial.h"
#include <vector>
#include <cmath>
#include <iostream>
#include <ctime>
#include <stdlib.h>
using namespace std;
control::control(float t)
{
    srand ( time(0) );
    par.clear();
    time_interval = 0;
    del_t = t;
}
void control::m_preference(int par_Time_limit, int fireworks_num)
{//preference configure

    //new particle time interval limit
    m_NewParticle_Time_limit(par_Time_limit);
    //fireworks' number
    Num_fireworks = fireworks_num;
}
void control::m_setRand_fireVel(vector< vector<float> > input)
{//pass the fireworks' velocity
    Rand_firework_vel = input;
}
vector<float> control::m_Rand_Par_Vel()
{//return a ramdon velocity limited by room size and particle length
    vector<float> output(3);
    //x
    output[0] = rand() % 20;
    output[0] -= 10;
    //y
    output[1] = rand() % 20;
    output[1] += 10;
    //z
    output[2] = rand() % 20;
    output[2] -= 10;
    return output;
}

void control::m_newParticle(int Num_fireworks)
{//create new particle
    if(TimeFunc_NewParticle())
    {
        for(int i =0 ; i< Num_fireworks; i++)
        {
            particle newPar;
            newPar.setParColor(Rand_firework_col[i]);
            //set particle color
            //set particle type to particle -> true
            newPar.setParType(true);
            //add particle
            newPar.setPos(0 , -15 , -50);
            //set initial particle position
            newPar.setVel(m_Rand_Par_Vel());
            //set particle velocity
            par.push_back(newPar);
        }
    }
}
void control::m_newFireworks(vector< vector<float> > firework_pos,
                             int Num_fireworks)
{//create new Fireworks on the position where particle was dead
    if((int)firework_pos.size()==0)
        return;
    int pos = (int)firework_pos.size();
    for(int i =0 ; i< pos; i++)
    {//position where fireworks should explode
        for(int j = 0; j <Num_fireworks; j++ )
        {//number of fireworks blow up a time
            float index = (float)j/Num_fireworks;
            index *= 10;
            particle newFirework;
            //set particle color
            newFirework.setParColor(Rand_firework_col[(int)index]);
            //set particle type to fireworks -> false
            newFirework.setParType(false);
            //add firework_pos
            newFirework.setPos(firework_pos[i]);
            //set initial firework_pos position
            newFirework.setVel(Rand_firework_vel[j]);
            //set firework_pos velocity
            par.push_back(newFirework);
        }
    }

}


vector< vector<float> > control::m_check_par_life()
{//check life of particle, if exceed its life limit then die
    //then create fireworks

    vector< vector<float> > firework_pos;
    if((int)par.size() == 0)
        return firework_pos;

    for(int i = 0; i<(int)par.size(); i++)
    {

        par[i].addLife(del_t);
        if(par[i].getLife() > par[i].getLifeLimit())
        {//this particle reach life limit then explode
            if(par[i].getParType() == true)
            {//this particle will blow up
                firework_pos.push_back(par[i].p);
                //collect explosion position
            }
            par.erase(par.begin()+i);
            //erase the particle
            par[i].addLife(del_t);


            par[i].draw();

        }
        else
        {
            par[i].draw();
        }
    }
    return firework_pos;
}


void control::update(float t)
{//up date particle system
    ///1. create new particle
    m_newParticle(1);
    ///2. gravity law
    m_gravity(t);
    ///3. check life of particle and fireworks
    vector< vector<float> > firework_pos = m_check_par_life();
    ///4. create new fireworks
    m_newFireworks(firework_pos, Num_fireworks);


}
void control::m_gravity(float t)
{//t ->delta t // gravity law
    vector <float> p1(3);
    vector <float> v1(3);
    for(int i = 0; i<(int)par.size(); i++)
    {
        vector <float> p0 = par[i].p;//current position
        vector <float> v0 = par[i].v;//current velocity
        vector <float> a = par[i].a; //acceleration
        for(int j =0 ; j < 3; j++)
        {
            //v' = v + at
            v1[j] = v0[j] + a[j]*t;
            //p' = p + vt + 1/2 at^2;
            p1[j] = p0[j] + (v0[j]+v1[j])*t/2  ;
        }
        par[i].v = v1;
        par[i].p = p1;
    }
}
void control::m_NewParticle_Time_limit(int input)
{//set time interval for generating next particle

    interval_time_limit = input;
}

void control::m_Rand_fireworks_color(vector< vector<float> >input)
{//random fireworks material diffuse color
    Rand_firework_col = input;
}
bool control::TimeFunc_NewParticle()
{//determine if time is good for generate new particle
    time_interval += del_t;
    bool output = NULL;
    if(time_interval > interval_time_limit)
    {
        time_interval = 0;
        output = true;
    }
    else
        output = false;
    return output;
}
control::~control()
{
    //dtor
}
