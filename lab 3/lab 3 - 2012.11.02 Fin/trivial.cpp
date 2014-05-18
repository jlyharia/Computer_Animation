#include "trivial.h"
#include <GL/glut.h>
#include <math.h>
#include <vector>
#include <iostream>
#define PI 3.14159265
#define B_splines 0
#define CatmullRom 1
using namespace std;


vector < vector<float> > Rotation_X(float alpha)
{//input angle - output rotation matrix by X axis
    float angle = sin(alpha* PI/180);
    vector < vector<float> > matrix_R = idenMatrix(4);
    matrix_R[1][1]= cos(angle);
    matrix_R[1][2]=  -sin(angle);
    matrix_R[2][1]=  sin(angle);
    matrix_R[2][2]=  cos(angle);
    return matrix_R;
}

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

vector< vector <float> > Spline(float t, int posType, int SplineType,
                                vector < vector<float> > input,
                                vector < vector<float> > &tangentVector)
{//input cubic parameter t, posType
    //Spline type, and position Info
    //Q(t) = TMG
    //posType - number of numbers for a position infomation
    //add one function to compute tangent for lab 2
    vector <vector <float> > interPosInfo;
    //select type of Spline
    vector <vector <float> > M;
    if(SplineType == CatmullRom)
        M = M_Catmull_Rom();
    else if (SplineType == B_splines)
        M = M_B_Spline();

    vector < vector<float> > G;
    //Calculate Interpolate point

    int numOfCurve = -3 + input.size()/(sizeof(float)*input[0].size());
    for(int n = 0; n < numOfCurve ; n++)
    {//number of curve produce by control point
        for(int dim = 0; dim < posType ; dim++)
        {
            vector<float> temp_G;
            for(int i = n ; i < n + 4 ; i++)//put 4 number in G
                temp_G.push_back(input[i][dim]);
            G.push_back(temp_G);
        }
    }
    for(int i = 0; i < numOfCurve ; i++)
    {
        for(float s = 0; s < 1; s += t)
        {
            vector <float> interPoint;
            vector <float> tempTangent;
            vector <float> T(4,1);
            T[0] = pow(s,3);
            T[1] = pow(s,2);
            T[2] = s;

            vector <float> T_tan(3,1);
            T_tan[0] = 3 * pow(s,2);
            T_tan[1] = 2 * s;
            T_tan[2] = 1;
            for(int dim = 0; dim < posType ; dim++)
            { //(x,y,z)
                //M * G  4x4 * 4x1
                vector<float> inter = MatMuliti(M, G[i*posType + dim]);
                //T * (M*G) 1x4 * 4x1
                interPoint.push_back(MatMuliti(T,inter));
                //for tangent(x,y,z)
                inter.pop_back();//3x1
                tempTangent.push_back(MatMuliti(T_tan,inter));
            }

            interPosInfo.push_back(interPoint);
            tangentVector.push_back(tempTangent);

        }
    }

    return interPosInfo;
}
vector < vector <float> > Translate(vector <float> intput)
{//input x y z//output 4x4 tanslate matrix
    vector < vector <float> > output = idenMatrix(4);
    for(int i = 0; i<3;i++)//[0 3][1 3][2 3]
        output[i][3] = intput[i];
    return output;
}
vector< vector <float> > TangenToMat(vector<float> input)
{//input tangent vector// output tangent rotation matrix
    vector < vector<float> > output (4,vector<float>(4));
    vector<float> up_vector(3,0);
    up_vector[1] = 1;

    vector <float> N_vector = Normalize(input);
    vector <float> U_vector = Normalize(cross_product3D(N_vector,up_vector));
    vector <float> V_vector = cross_product3D(U_vector,N_vector);

    output[0].assign(U_vector.begin(),U_vector.end());
    output[1].assign(V_vector.begin(),V_vector.end());
    output[2].assign(N_vector.begin(),N_vector.end());
    for(int i = 0; i<3;i++)
        output[i].push_back(0);
    output[3][3] =  1;
    return output;

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

vector <float>RowToColumn(vector < vector <float> >input)
{//input 2-D row major array// output 1-D column major array
    int capacity = input[0].size()*input.size();
    int column = input[0].size();
    vector <float> output (capacity);
    for(int i = 0; i<(int)input.size();i++)
        for(int j = 0; j<(int)input[i].size();j++)
            output[i+j*column] = input[i][j];
    return output;
}
vector < vector <float> >ColumnToRow(vector <float>input)
{//input 2-D column major array// output 1-D row major array
    int capacity = sqrt(input.size());
    vector < vector <float> >output(capacity,vector<float> (capacity));
    for(int i = 0; i< capacity ; i++)
        for(int j = 0; j< capacity ; j++)
            output[i][j] = input[i+j*capacity];
    return output;
}
vector < vector <float> > idenMatrix(int d)
{//input dimention //output identidy matrix
    vector < vector <float> > output(d, vector<float> (d));
    for(int i = 0; i < d ; i++)
        for(int j = 0; j < d ; j++)
            if(i == j)
                output[i][j] = 1;
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