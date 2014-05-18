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
    vector <float> cro_product(3);
    cro_product[0] = vector_1[1]*vector_2[2] - vector_2[1]*vector_1[2];
    cro_product[1] = -(vector_1[0]*vector_2[2] - vector_2[0]*vector_1[2]);
    cro_product[2] = vector_1[0]*vector_2[1] - vector_2[0]*vector_1[1];
    return cro_product;
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
vector<float> pointMinus(vector<float> point_1, vector<float> point_2)
{//output vector v1->v2
    vector<float> vec((int)point_1.size());
    for(int i = 0; i < (int)point_1.size();i++)
        vec[i] = point_2[i] - point_1[i];
    return vec;
}
vector< vector <float> > M_Catmull_Rom()
{//return CatmullRom Spline Matrix
    vector< vector <float> > M_CatmullRom(4, vector<float> (4));
    float a = 0.5; //CatmullRom
    M_CatmullRom [0][0] = -a;
    M_CatmullRom [0][1] = 2-a;
    M_CatmullRom [0][2] = a-2;
    M_CatmullRom [0][3] = a;
    M_CatmullRom [1][0] = 2*a;
    M_CatmullRom [1][1] = a-3;
    M_CatmullRom [1][2] = 3-(2*a);
    M_CatmullRom [1][3] = -a;
    M_CatmullRom [2][0] = -a;
    M_CatmullRom [2][1] = 0;
    M_CatmullRom [2][2] = a;
    M_CatmullRom [2][3] = 0;
    M_CatmullRom [3][0] = 0;
    M_CatmullRom [3][1] = 1;
    M_CatmullRom [3][2] = 0;
    M_CatmullRom [3][3] = 0;
    return M_CatmullRom;
}
vector< vector <float> > M_B_Spline()
{//return B Spline Matrix
    vector< vector <float> > M_BSpline(4, vector<float> (4));
    float a = 0.1666666; //B Spline
    M_BSpline [0][0] = -a;
    M_BSpline [0][1] = 3*a;
    M_BSpline [0][2] = -3*a;
    M_BSpline [0][3] = a;
    M_BSpline [1][0] = 3*a;
    M_BSpline [1][1] = -6*a;
    M_BSpline [1][2] = 3*a;
    M_BSpline [1][3] = 0;
    M_BSpline [2][0] = -3*a;
    M_BSpline [2][1] = 0;
    M_BSpline [2][2] = 3*a;
    M_BSpline [2][3] = 0;
    M_BSpline [3][0] = a;
    M_BSpline [3][1] = 4*a;
    M_BSpline [3][2] = a;
    M_BSpline [3][3] = 0;
    return M_BSpline;
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
