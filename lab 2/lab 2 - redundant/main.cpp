/****
    Fall 2012
    Computer Animation
    Lab 2 : Hierarchical Object Motion Control System
    Student Name : YiHung Lee
    Gw ID        : G42957795
    Programming flow:
        step 1  convert array to vector
        step 2  convert 6 number fixed angle coordinate system to quaternion
        step 3  cubic interpolation and compute tangent vector for each point
        step 4  derive body matrix that face tangent direction by replace
                rotation part in interpolation matrix by tangent matrix
        step 5  derive leg matrix by combining leg's matrix inlocal coordinate
                system and body matrix
        Fin     render body,left leg and right leg by scale glSolidCube
****/
#include "trivial.h"
#include <GL/glut.h>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include "CONTROL.h"

#define PI 3.14159265

#define B_splines 0
#define CatmullRom 1
using namespace std;

// screen width and height
int screenWidth;
int screenHeight;
//declare a storage for a series position information by interpolating
//input position information
vector< vector<float> > Mat_body;
vector< vector<float> > Mat_R_Leg;
vector< vector<float> > Mat_L_Leg;
vector< vector<float> > vec_PosInfo;

int count = 0;
float Power_T = 0.005;

/*
input position information_2 (6 number fixed angle form)
(x,y,z, apha, beta, theta)
alpha  - angle of rotattion about x axis
beta   - angle of rotattion about y axis
gammar - angle of rotattion about z axis
*/
float PosInfo[7][6] = {{6, 0, -20  ,0  ,0 ,0},
                       {-6,0 ,-20  ,0  ,0 ,0},
                       {-6,0, -8   ,0  ,0 ,0},
                       {6, 0, -8   ,0  ,0 ,0},
                       {6, 0, -20  ,0  ,0 ,0},
                       {-6, 0,-20  ,0  ,0 ,0},
                       {-6, 0,-8   ,0  ,0 ,0}};
GLfloat legScale[16]={1,0,0,0,0,3,0,0,0,0,1,0,0,0,0,1};
//body scale (2,3,1)
GLfloat bodyScale[16]={2,0,0,0,0,3,0,0,0,0,1,0,0,0,0,1};
//leg scale (1,3,1)

void drawTeapot()
{
    ///------------body
    glPushMatrix();
    GLfloat glfM_Move[16];
    copy(Mat_body[count].begin(),Mat_body[count].end(),glfM_Move);
    glLoadMatrixf(glfM_Move);
    //body scale 2,3,1
    glMultMatrixf(bodyScale);
    glutSolidCube(1);
    glPopMatrix();
    ///---------right leg------------
    glPushMatrix();
    GLfloat glfM_RLeg[16];
    copy(Mat_R_Leg[count].begin(),Mat_R_Leg[count].end(),glfM_RLeg);
    glLoadMatrixf(glfM_RLeg);
    //leg scale 1,3,1
    glMultMatrixf(legScale);
    glutSolidCube(1);
    glPopMatrix();
    ///---------left leg------------
    glPushMatrix();
    GLfloat glfM_LLeg[16];
    copy(Mat_L_Leg[count].begin(),Mat_L_Leg[count].end(),glfM_LLeg);
    glLoadMatrixf(glfM_LLeg);
    //leg scale 1,3,1
    glMultMatrixf(legScale);
    glutSolidCube(1);
    glPopMatrix();
}
vector< vector<float> > TransArrToVec()
{
    int array_size,array_long;
    array_long = sizeof(PosInfo[0])/sizeof(float);
    array_size = sizeof(PosInfo);

    vector < vector<float> > vec;
    for(int i = 0 ; i < array_size ; i++)
    {
        vector <float> vec_temp;
        vec_temp.assign(PosInfo[i],PosInfo[i] + array_long);
        vec.push_back(vec_temp);
    }
    return vec;
}

void CreateBody()
{

    //create an interface to control body and legs
    CONTROL obj;
    //set right leg position relative to body
    obj.set_iniTrans_R_leg(-0.5,-3,0);
    obj.set_iniTrans_L_leg(0.5,-3,0);
    //set leg move speed
    obj.setlegSpeed(3.2);
    //convert array to vectors
    vec_PosInfo = TransArrToVec();
    //compute matrix
    obj.iniProcess(Power_T, B_splines,vec_PosInfo);
    //get body's matrix
    Mat_body = obj.getBodyMatrix();
    //get right leg's matrix
    Mat_R_Leg = obj.getRLegMatrix();
    //get left leg's matrix
    Mat_L_Leg = obj.getLLegMatrix();
}
// OpenGL initialization
void init()
{
    glClearColor (0.0, 0.0, 0.0, 0.0);
    glClearDepth (1.0);
}

// timer
void timer( int value )
{

    // frame index

    if(count < (int)Mat_body.size())
        count++;
    else
        count = 0;

    // render
    glutPostRedisplay();

    // reset timer
    // 16 ms per frame ( about 60 frames per second )FPS = 60
    glutTimerFunc( 16, timer, 0 );
}

void drawScene()
{
    // clear buffer
    glClearColor (0.0, 0.0, 0.0, 0.0);
    glClearDepth (1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // render state
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);

    // light source attributes
    GLfloat LightAmbient[]	= { 0.4f, 0.4f, 0.4f, 1.0f };
    GLfloat LightDiffuse[]	= { 0.9f, 0.9f, 0.9f, 1.0f };
    GLfloat LightSpecular[]	= { 0.4f, 0.4f, 0.4f, 1.0f };
    GLfloat LightPosition[] = { 5.0f, 5.0f, 5.0f, 1.0f };

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_AMBIENT , LightAmbient );
    glLightfv(GL_LIGHT0, GL_DIFFUSE , LightDiffuse );
    glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
    glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);

    // surface material attributes
    GLfloat material_Ka[]	= { 0.11f, 0.06f, 0.11f, 1.0f };
    GLfloat material_Kd[]	= { 0.43f, 0.47f, 0.54f, 1.0f };
    GLfloat material_Ks[]	= { 0.33f, 0.33f, 0.52f, 1.0f };
    GLfloat material_Ke[]	= { 0.1f , 0.0f , 0.1f , 1.0f };
    GLfloat material_Se		= 10;

    glMaterialfv(GL_FRONT, GL_AMBIENT	, material_Ka);
    glMaterialfv(GL_FRONT, GL_DIFFUSE	, material_Kd);
    glMaterialfv(GL_FRONT, GL_SPECULAR	, material_Ks);
    glMaterialfv(GL_FRONT, GL_EMISSION	, material_Ke);
    glMaterialf (GL_FRONT, GL_SHININESS	, material_Se);

    // modelview matrix
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();
    drawTeapot();

    // light source
    glDisable(GL_LIGHT0);
    glDisable(GL_LIGHTING);

    // swap back and front buffers
    glutSwapBuffers();


}

// update viewport and projection matrix when the window is resized
void reshape( int w, int h )
{
    // viewport
    glViewport(0, 0, (GLsizei) w, (GLsizei) h);

    // projection matrix
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    gluPerspective(65.0, (GLfloat) w/(GLfloat) h, 1.0, 50.0);

    screenWidth = w;
    screenHeight = h;
}

// keyboard input
void keyboard( unsigned char key, int x, int y )
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



int main( int argc, char** argv )
{
    CreateBody();
    // create GL window
    glutInit(&argc, argv);
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB |GLUT_DEPTH);
    glutInitWindowSize(936, 528);
    glutInitWindowPosition(150, 100);
    glutCreateWindow(argv[0]);

    // user initialization
    init();

    // set callback functions
    glutDisplayFunc(drawScene);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutTimerFunc(16, timer, 0);
    // main loop
    glutMainLoop();

    return 0;
}
