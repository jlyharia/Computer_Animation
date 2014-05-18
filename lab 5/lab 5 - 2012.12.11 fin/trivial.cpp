#include "trivial.h"
#include <GL/glut.h>
#include <math.h>
#include <vector>
#include <iostream>
#define PI 3.14159265
#define B_splines 0
#define CatmullRom 1
using namespace std;

vector <float> cross_product3D(vector<float> vector_1,
                               vector<float> vector_2)
{//compute cross product
    vector <float> normal(3);
    normal[0] = vector_1[1]*vector_2[2] - vector_2[1]*vector_1[2];
    normal[1] = -(vector_1[0]*vector_2[2] - vector_2[0]*vector_1[2]);
    normal[2] = vector_1[0]*vector_2[1] - vector_2[0]*vector_1[1];
    return normal;
}
float dot_product3D(vector<float> vector_1,vector<float> vector_2)
{//input 2 3D vector, output dot product
    float dotProduct = 0;
    for(int i = 0 ; i <(int)vector_1.size(); i++)
        dotProduct += vector_1[i]*vector_2[i];
    return dotProduct;
}

vector<float> Normalize(vector<float> input)
{//intput 1-d vector//output normalized vector
    vector<float> output;
    for(int i=0;i <(int)input.size(); i++)
        output.push_back(input[i]/vecLenth(input));
    return output;
}
float vecLenth(vector<float> input)
{//intput 1-d vector//output length of vector
    float magnitude = 0;
    for(int i=0;i <(int)input.size(); i++)
        magnitude += pow(input[i],2);
    return sqrt(magnitude);
}
float pointLength(vector<float> point_1,vector<float> point_2)
{//output two point length
    vector<float> vec((int)point_1.size());
    for(int i = 0; i < (int)point_1.size();i++)
        vec[i] = point_1[i] - point_2[i];
    return vecLenth(vec);
}


vector<float> MatMuliti(vector < vector<float> > matrix_L,
                                 vector<float> matrix_R)
{//MxN * Nx1 //output 1D vector
    vector<float>  matrix_out ((int)matrix_L.size());
    for(int m3_row = 0 ; m3_row < (int)matrix_L.size() ; m3_row++ )
    {
        matrix_out[m3_row] =0;
        for(int j=0; j<(int)matrix_R.size();j++)
            matrix_out[m3_row] += matrix_L[m3_row][j] * matrix_R[j];
    }
    return matrix_out;
}

float MatMuliti(vector<float> matrix_L,vector<float> matrix_R)
{//1xN * Nx1 // output a value
    float out = 0;
    for(int i = 0 ; i < (int)matrix_L.size() ; i++ )
        out +=  matrix_L[i] * matrix_R[i];
    return out;
}


vector < vector <float> > MatMuliti(vector < vector <float> > mat_1,
                                  vector < vector <float> > mat_2)
{//mat_1 * mat_2 in any dimention larger than 1
    int row = (int)mat_1.size();
    int column = (int)mat_2[0].size();
    vector < vector <float> > output(row, vector<float> (column));
    for(int i = 0 ; i < row ;i++)
        for(int j = 0;j < column ;j++)
            for(int k = 0; k < (int)mat_2.size() ;k++)
                output[j][i] += mat_1[j][k] * mat_2[k][i];
    return output;
}

vector <float> Translate1D(vector <float> input)
{//input x y z//output colume major 1d 1x16 vector
    vector <float> output(16);
    output[0]  = 1;
    output[5]  = 1;
    output[10] = 1;
    output[15] = 1;
    output[12] = input[0];
    output[13] = input[1];
    output[14] = input[2];
    return output;
}

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
// update viewport and projection matrix when the window is resized
void reshape( int w, int h )
{
    // viewport
    glViewport(0, 0, (GLsizei) w, (GLsizei) h);

    // projection matrix
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    gluPerspective(65.0, (GLfloat) w/(GLfloat) h, 1.0, 200.0);
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

void setLight_0()
{
    GLfloat lightPos0[4] = {-3, 19.5, -31.5, 1.0f};
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

// timer
void timer( int value )
{
    // render
    glutPostRedisplay();
    // 16 ms per frame ( about 60 frames per second )FPS = 60
    glutTimerFunc( 16, timer, 0 );
}
