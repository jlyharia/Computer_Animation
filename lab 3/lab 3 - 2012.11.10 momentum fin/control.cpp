#include "control.h"
#include "particle.h"
#include "trivial.h"
#include <vector>
#include <cmath>
#include <iostream>
#include <ctime>
#include <stdlib.h>
using namespace std;
control::control(float ballSize)
{
    par_radius = ballSize;//set particle
    setEnergyEmis(0.9);
}
vector<float> control::m_RandomPar()
{//return a ramdon position limited by room size and particle length
    vector<float> pos(3);
    pos[2] = rand() % (int)(room_length - 2 * par_radius);
    pos[2] = -pos[2] - par_radius;
    pos[0] = rand() % (int)(room_width - 2 * par_radius);
    pos[0] = pos[0] - room_width/2 + par_radius;
    pos[1] = rand() % (int)(room_heght - 2 * par_radius);
    pos[1] = pos[1] + floor_trans + par_radius;

    return pos;
}
void control::setParticleInfo()
{//set particle information 1)Position 2) velocity
    //spread seed
    srand ( time(0) );
    float vx,vy,vz;
    vector<float> pos(3);
    float diameter = 2 * par_radius;
    for(int i = 0 ; i < massNum ; i++)
    {//ramdon position limited in room
        //set ramson position
        pos = m_RandomPar();
        for(int j = 0 ; j< i; j++)
        {//check if current particle inside other particles
            if(pointLength(pos, par[j].p) < diameter)
            {
                pos = m_RandomPar();//reset position
                j = 0;
            }
        }
        par[i].setPos(pos[0],pos[1],pos[2]);
        //ramdon velocity limited between -20 and 20
        vx = (rand() % 40) -20;
        vy = (rand() % 40) -20;
        vz = (rand() % 40) -20;
        par[i].setVel(vx,vy,vz);
        //set particle mass
        par[i].mass = par_radius;
    }

}
void control::update(float t)
{
    //step 1 find next position
    nextPos(t);
    //step 2 put in the matrix form
    objMatrix();
}
void control::objMatrix()
{//put particle translate matrix to 2d vector
    parMartix.clear();
    vector <float> uni;
    for(int i = 0; i < (int)par.size(); i++)
    {
        uni = Translate1D(par[i].p);
        parMartix.push_back(uni);
    }
}
vector<float> control::m_reflect_vec(vector<float> input, vector<float> normal)
{//output reflection velocity//input 1)incoming velocity 2)collision normal
    //r = 2n(n x L) - L
    vector<float> output(3);
    for(int i = 0; i < 3; i++)
    {
        input[i] = -input[i];//reverse velocity
        output[i] = 2*normal[i]*(normal[i] * input[i]) - input[i];
    }
    return output;
}
void control::m_gravity(vector <float> &p1, vector <float> &v1,float t,
                        particle &ball)
{//modify p1,v1 by gravity law//p1 -> next position, v1 ->next velocity
    //t ->delta t
    vector <float> p0 = ball.p;//current position
    vector <float> v0 = ball.v;//current velocity
    vector <float> a = ball.a; //acceleration
    for(int i =0 ; i < (int)p0.size(); i++)
    {
        //v' = v + at
        v1[i] = v0[i] + a[i]*t;
        //p' = p + vt + 1/2 at^2;
        p1[i] = p0[i] + (v0[i]+v1[i])*t/2  ;


    }
}
void control::m_collision_walls(vector <float> &p1, vector <float> &v1
                                ,particle &ball)
{//modify p1,v1 by colliding with walls
    vector <float> v0 = ball.v;//current velocity
    for(int i =0 ; i<(int)room_bound.size(); i++)
    {//i = 5 walls
        int temp = 0;
        for(int j =0 ; j<3; j++)
        {//find index of non zero element
            if(room_normal[i][j] != 0)
                temp = j;
        }
        //check position in case stick on wall
        if(abs(p1[temp] - room_bound[i][temp]) < par_radius)
        {//walls collision
            //move particle out of wall
            /*
              since walls' normal is either positive or negative
              by multiply room_normal[i][temp], we can determine
              where particle should move
            */
            p1[temp] = room_bound[i][temp] + par_radius * room_normal[i][temp];
            //find reflection velocity
            vector <float> refl = m_reflect_vec(v0,room_normal[i]);
            //energy emission
            for(int j = 0; j < 3; j++)
                v1[j] = refl[j] * energyEmis;
        }
    }
}

void control::momentum(particle fir_ball, particle &sec_ball,vector <float> &v1)
{//compute velocity by conservation of momentum
    vector <float> va(3);//next position current
    vector <float> vb(3);//next position compared
    vector <float> ub = sec_ball.v;
    vector <float> ua = fir_ball.v;
    float mb = sec_ball.mass;
    float ma = fir_ball.mass;
    float Cr = 1.0;
    for(int i = 0 ; i < 3 ; i++)
    {
        va[i] = (Cr*mb*(ub[i]-ua[i]) + ma*ua[i] + mb*ub[i])/(ma + mb);
        vb[i] = (Cr*ma*(ua[i]-ub[i]) + ma*ua[i] + mb*ub[i])/(ma + mb);
        va[i] *= energyEmis; //energy emission
        vb[i] *= energyEmis; //energy emission
    }
    //update velocity - current
    v1 = va;
    //update velocity - compared
    sec_ball.setVel(vb[0],vb[1],vb[2]);
}

void control::m_collision_par(vector <float> &p1, vector <float> &v1,
                              particle ball , int cur_index)
{//modify p1,v1 by colliding with particles
    //ball -> current particle
    //p1 -> next position, v1 ->next velocity
    float diameter = 2 * par_radius;
    for(int i = cur_index + 1 ; i< massNum; i++)
    {//i ->compared particles
        //determine if two particle will collide or leave each other
        if(pointLength(p1, par[i].p) < diameter)
        {//p1 -> next particles postion//par[i] -> compared particles
            if(pointLength(p1, par[i].p) >= pointLength(ball.p, par[i].p))
            {//if two particles' distance greater than before
                // leave apart
                continue;
            }
            else
            {//collision calculate both velocity
                momentum(ball,par[i],v1);
                p1 = ball.p;
            }

        }
    }
}
void control::nextPos(float t)
{//find next position
    vector <float> p1(3);//next position
    vector <float> v1(3);//next velocity
    for(int i = 0; i< massNum;i++)
    {//i -> current particle

        ///gravity law
        m_gravity(p1 , v1 , t, par[i]);


        ///check wall collision
        m_collision_walls(p1 , v1 , par[i]);

        ///check particle collision
        m_collision_par(p1 , v1 , par[i], i);

        ///update particle information
        //update position
        par[i].setPos(p1[0],p1[1],p1[2]);
        //update velocity
        par[i].setVel(v1[0],v1[1],v1[2]);
    }
}
vector < vector <float> > control::getMatrix()
{
    return parMartix;
}
void control::setRoomBoundary(float x,float y, float z, float Room_flo)
{//set room boundary
    room_width = x;
    room_heght = y;
    room_length = z;
    floor_trans = Room_flo;
    room_bound.clear();
    vector<float> uni(3);
    //front
    uni[2] = -z;
    room_bound.push_back(uni);
    //back
    uni[2] = 0;
    room_bound.push_back(uni);
    //right
    uni[0] = x/2;
    room_bound.push_back(uni);
    //left
    uni[0] = -x/2;
    room_bound.push_back(uni);
    //floor
    uni[0] = 0;
    uni[1] = Room_flo;
    room_bound.push_back(uni);
    ///------------------
    room_normal.clear();
    vector<float> nor(3);
    //front
    nor[2] = 1;
    room_normal.push_back(nor);
    //back
    nor[2] = -1;
    room_normal.push_back(nor);
    //right
    nor[2] = 0;
    nor[0] = -1;
    room_normal.push_back(nor);
    //left
    nor[0] = 1;
    room_normal.push_back(nor);
    //floor
    nor[0] = 0;
    nor[1] = 1;
    room_normal.push_back(nor);
    //----------------
    setParticleInfo();
}
void control::setEnergyEmis(float input)
{//set energy emission
    energyEmis = input;
}
void control::setParticleNumber(int num)
{//set number of particle
    massNum = num;
    vector<particle> patern(num);
    par = patern;
}
control::~control()
{
    //dtor
}
