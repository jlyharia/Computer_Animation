#include "particle.h"
#include <vector>
#include <iostream>
#include "trivial.h"
#include <GL/glut.h>
#include <ctime>
#include <stdlib.h>
using namespace std;
particle::particle()
{
    srand ( time(0) );
    vector <float> temp(3);
    p = temp;
    v = temp;
    a = temp;
    a[1] = -9.8;//gravity toward negtive y
    life = 0;

}
void particle::setParType(bool type)
{//if type = true -> particle
    //type = false -> fireworks
    if(type == true)
    {//type = true -> particle
        par_Type = type;
        m_life_time();
    }
    else
    {//type = false -> fireworks
        par_Type = type;
        m_firework_life_time_limit();
    }
}

void particle::m_life_time()
{//set particle life time
    life_limit = 2;
}
void particle::m_firework_life_time_limit()
{//set fire work life time limit
    life_limit = 6;
}
//int particle::life
void particle::addLife(float t)
{//add life time
    life += t;
}
void particle::setParColor(vector <float> input)
{
    par_color = input;
}
void particle::draw()
{//draw firework
    vector <float>  uni = Translate1D(p);
    ///------------------------
    GLfloat material_diffuse[3];
    copy(par_color.begin(),par_color.end(),material_diffuse);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, material_diffuse);

    glPushMatrix();
    GLfloat glfM_Move[16];
    copy(uni.begin(),uni.end(),glfM_Move);
    glLoadMatrixf(glfM_Move);
    glutSolidSphere(0.5,50,20);
    glPopMatrix();

}
bool particle::getParType()
{
    return par_Type;
}
void particle::setVel(vector <float> input)
{//initialze position
    v[0] = input[0];
    v[1] = input[1];
    v[2] = input[2];
}
void particle::setVel(float x,float y,float z)
{//initialze position
    v[0] = x;
    v[1] = y;
    v[2] = z;
}
void particle::setPos(vector <float> input)
{//initialze position
    p[0] = input[0];
    p[1] = input[1];
    p[2] = input[2];
}
void particle::setPos(float x,float y,float z)
{//initialze position
    p[0] = x;
    p[1] = y;
    p[2] = z;
}

particle::~particle()
{

}
