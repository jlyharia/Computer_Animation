/****
    Fall 2012
    Computer Animation
    Lab 5 : Particle System - Fireworks Simulation
    Student Name : YiHung Lee
    Gw ID        : G42957795
    Approach :
    https://sites.google.com/site/breezeverywhere/project/computer-animation/lab-5

****/
#include "SHADER.h"
#include "trivial.h"
#include <GL/glew.h>
#include <GL/glut.h>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include "particle.h"
#include "control.h"
#include <ctime>
#include <stdlib.h>
using namespace std;

float del_t = 0.03;

GLuint shader_phong;

control obj(del_t);



void updateParticle()
{//update particle position
    obj.update(del_t);
}
void setRand(int Num_fireworks)
{//random fireworks initial velocity
    srand ( time(0) );
    vector< vector<float> > velocity;
    for(int i = 0 ; i< Num_fireworks; i++)
    {
        vector<float> uni(3);
        //x
        uni[0] = rand() % (int)30;
        uni[0] -= 15;
        //y
        uni[1] = rand() % (int)20;
        uni[1] += 10;
        //z

        uni[2] = rand() % (int)30;
        uni[2] -= 15;
        velocity.push_back(uni);
    }
    obj.m_setRand_fireVel(velocity);
}
void setRandColor()
{//random fireworks color
    vector< vector<float> > color;
    for(int i = 0 ; i< 11; i++)
    {
        vector<float> uni(3);
        for(int j = 0 ; j<3; j++)
        {
            uni[j] = (float)rand()/RAND_MAX;
        }
        color.push_back(uni);
    }
    obj.m_Rand_fireworks_color(color);
}
void iniPSystem()
{//initialize particle system
    //setting preference
    int Num_fireworks = 30;//set number of fireworks would blow up
    int par_Time_limit = 3;//set time interval between each shoot
    obj.m_preference(par_Time_limit,Num_fireworks);
    setRand(Num_fireworks);
    setRandColor();
}

void drawScene()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    // modelview matrix
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();
    //set light 0
    setLight_0();
    glUseProgram(shader_phong);

    //update particle system
    updateParticle();

    // swap back and front buffers
    glutSwapBuffers();
}

int main( int argc, char** argv )
{
    iniPSystem();
    // create GL window
    glutInit(&argc, argv);
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGBA |GLUT_DEPTH);
    glutInitWindowSize(936, 528);
    glutInitWindowPosition(150, 100);
    glutCreateWindow(argv[0]);

    // user initialization
    init();

    // set callback functions
    glutDisplayFunc(drawScene);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(handleKeypress);
    glutTimerFunc(16, timer, 0);
    //-----------glew--------------

    glewInit();
    shader_phong = CreateShaders("phong");
    // main loop
    glutMainLoop();

    return 0;
}
