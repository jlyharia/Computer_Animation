/****
    Fall 2012
    Computer Animation
    Lab 4 : Behavioral Motion Control System
    Student Name : YiHung Lee
    Gw ID        : G42957795
    Approach:
    https://sites.google.com/site/breezeverywhere/project/computer-animation/lab-4

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
#include "leader.h"
#include "control.h"
#define PI 3.14159265
using namespace std;
GLfloat lightPos0[4] = {-3, 19.5, -31.5, 1.0f};
//timer
float del_t = 0.01;

const float ballSize = 2;
GLuint shader_phong;


float roomScale_x = 100;
float roomScale_y = 50;
float roomScale_z = 70;
float floor_pos = -20;
float Obstacle_pos[3] = {10,0, -70};
float Obstacle_size = 7;


control obj(ballSize);
leader boss;
void setLight_0()
{
    GLfloat light0_specular[] = {0.3f, 0.3f, 0.3f, 1.0f}; //White Specular light
    GLfloat light0_diffuse[] = {0.6f, 0.6f, 0.6f, 1.0f}; //red diffuse light
    GLfloat light0_ambien[] = {0.2f, 0.2f, 0.2f, 1.0f}; //red diffuse light

	glLightfv(GL_LIGHT0, GL_POSITION, lightPos0);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light0_specular);
	glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambien);

    GLfloat shininess = 50;
    GLfloat material_diffuse[] = {1.0f, 0.55f, 0.52f, 1.0f};
    GLfloat material_Specular[] = {0.5f, 0.5f, 0.5f, 1.0f};//light yellow

	glMaterialfv(GL_FRONT, GL_SPECULAR, material_Specular);
	glMaterialf(GL_FRONT, GL_SHININESS, shininess); //The shininess parameter
	glMaterialfv(GL_FRONT, GL_DIFFUSE, material_diffuse);
    glEnable(GL_LIGHT0);
}

void drawRoom()
{//draw Room
    glPushMatrix();
    glTranslatef(-roomScale_x/2, floor_pos , -roomScale_z-30);
    glScalef(roomScale_x, roomScale_y, roomScale_z);
    glColor4f(0.67f, 1.0f, 0.18f, 1.0f);
    //-----------------------------------------------------
    glBegin(GL_LINE_STRIP);
        glVertex3f(0,0,0);
        glVertex3f(1,0,0);
        glVertex3f(1,1,0);
        glVertex3f(0,1,0);
        glVertex3f(0,0,0);
    glEnd();
    glBegin(GL_LINE_STRIP);
        glVertex3f(0,0,1);
        glVertex3f(1,0,1);
        glVertex3f(1,1,1);
        glVertex3f(0,1,1);
        glVertex3f(0,0,1);
    glEnd();
    glBegin(GL_LINES);
        glVertex3f(0,0,1);
        glVertex3f(0,0,0);

        glVertex3f(0,1,1);
        glVertex3f(0,1,0);

        glVertex3f(1,0,1);
        glVertex3f(1,0,0);

        glVertex3f(1,1,1);
        glVertex3f(1,1,0);
    glEnd();
    glPopMatrix();
}

// OpenGL initialization
void init()
{
    glClearColor (0.0, 0.0, 0.0, 0.0);
    glClearDepth (1.0);

    // render state
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_LIGHTING);
    glCullFace(GL_BACK);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);
}

// timer
void timer( int value )
{
    // render
    glutPostRedisplay();
    // 16 ms per frame ( about 60 frames per second )FPS = 60
    glutTimerFunc( 16, timer, 0 );
}

// keyboard input
void handleKeypress( unsigned char key, int x, int y )
{
    switch( key )
    {
        case VK_ESCAPE:
            exit(0);
            break;
        default:
            break;
    }
}

void reshape( int w, int h )
{// update viewport and projection matrix when the window is resized
    // viewport
    glViewport(0, 0, (GLsizei) w, (GLsizei) h);

    // projection matrix
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    gluPerspective(65.0, (GLfloat) w/(GLfloat) h, 1.0, 200.0);
}

void updateParticle()
{//update boids information
    obj.update(del_t,boss.getPosition());
    vector < vector <float> > massMatrix = obj.getMatrix();
    ///------------------------
    GLfloat material_diffuse[] = {0.8f, 0.8f, 1.0f, 1.0f};
    glMaterialfv(GL_FRONT, GL_DIFFUSE, material_diffuse);
    for(int i = 0 ; i< (int)massMatrix.size(); i++ )
    {
        glPushMatrix();
        GLfloat glfM_Move[16];
        copy(massMatrix[i].begin(),massMatrix[i].end(),glfM_Move);
        glLoadMatrixf(glfM_Move);
        glutSolidSphere(ballSize,50,20);
        glPopMatrix();
    }
}
void updateLeader()
{//update leader's position
    boss.update(del_t);
    vector <float>  massMatrix = boss.getMatrix();
    ///------------------------

    GLfloat material_diffuse[] = {0.5f, 0.3f, 1.0f, 1.0f};
    glMaterialfv(GL_FRONT, GL_DIFFUSE, material_diffuse);
    glPushMatrix();
        GLfloat glfM_Move[16];
        copy(massMatrix.begin(),massMatrix.end(),glfM_Move);
        glLoadMatrixf(glfM_Move);
        glutSolidSphere(ballSize,50,20);
    glPopMatrix();
}
void updateObstacle()
{
    GLfloat material_diffuse[] = {0.5f, 1.0f, 0.5f, 1.0f};
    glMaterialfv(GL_FRONT, GL_DIFFUSE, material_diffuse);
    glPushMatrix();
        glTranslatef(Obstacle_pos[0],Obstacle_pos[1],Obstacle_pos[2]);
        glutSolidSphere(Obstacle_size,50,20);
    glPopMatrix();
}
void drawScene()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    // modelview matrix
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();
    //set light 0
    setLight_0();
    //draw room
    glUseProgram(0);
    drawRoom();

    glUseProgram(shader_phong);
    //update Leader's information
    updateLeader();

    updateObstacle();

    //update particle system
    updateParticle();


    // swap back and front buffers
    glutSwapBuffers();
}

void iniPSystem()
{
    obj.setParticleNumber(20);
    obj.setRoomScale(roomScale_x,roomScale_y,roomScale_z);
    obj.setRoomTranslate(-roomScale_x/2, floor_pos , -roomScale_z-30);
    obj.setRoomBoundary();
    obj.setObstacleInfo(Obstacle_pos[0],Obstacle_pos[1],Obstacle_pos[2],
                        Obstacle_size);
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
