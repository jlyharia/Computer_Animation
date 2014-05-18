#include "trivial.h"
#include "leader.h"
#include <vector>
#include <cmath>
#include <iostream>
#define PI 3.14159265
using namespace std;
leader::leader()
{
    setControlPoint();
    cubic_M = M_Catmull_Rom();
    current_curve = 0;
    parameter_t = 0;
}

void leader::setControlPoint()
{//set leader's control point// key framing system
    Con_number = 11;
    float PosInfo[11][3] =
        {
        { 45,  25,-65},//1
        {-45,  25,-65},//2
        {-45, -15,-65},//3
        { 45, -15,-65},//4
        { 45,  25,-65},//5
        {-30,  25,-65},//6
        {  0,   0,-90},//7
        {  0, -15,-65},//8
        {-20,  10,-35},//9
        {-45,  25,-65},//10
        {-45, -15,-65} //11
        };
    vector< vector<float> > temp(Con_number,vector<float>(3));
    for(int i = 0; i< Con_number ;i++)
        for(int j = 0; j< 3; j++)
            temp[i][j] = PosInfo[i][j];

    controlPoint = temp;
}
vector<float> leader::getMatrix()
{//return matix required by OpenGL
    return leaderMatrix;
}
vector<float> leader::getPosition()
{//return leader's position
    return leaderPosition;
}
void leader::update(float t)
{//update t to find next position on curve
    //determine control points
    t *= 0.5;
    vector< vector<float> > G = find_G();

    //update new position
    leaderPosition = Spline(G,parameter_t);
    leaderMatrix = Translate1D(leaderPosition);

    if(parameter_t > 1)
    {//if reach the end, change to next curve
        current_curve++;
        if(current_curve > 7)
            current_curve = 0;//curve start over
        parameter_t = 0;//new curve start from 0
    }
    else
    {
        parameter_t += t;
    }
}
vector< vector<float> > leader::find_G()
{//find which control point should be used
    vector< vector<float> > output;
    for(int i = current_curve; i< current_curve + 4; i++)
    {
        output.push_back(controlPoint[i]);
    }
    return output;

}
vector <float> leader::Spline(vector< vector<float> > G,float t)
{//output an interpolated position
    //input cubic parameter t, and position Info
    //Q(t) = TMG
    vector <float> interPoint;
    //Calculate Interpolate point
    vector <vector <float> > G_column(3,vector <float>(4));
    for(int i = 0; i < 3 ; i++)
    {
        vector<float> temp_G;
        for(int j = 0 ; j < 4 ; j++)//put x,y,z in G in column form
            G_column[i][j] = G[j][i];
    }
    //find interpolation point
    vector <float> T(4,1);
    T[0] = pow(t,3);
    T[1] = pow(t,2);
    T[2] = t;
    for(int i = 0; i < 3 ; i++)
    {//i<3 for x,y,z only
        //M * G //4x4_4x1
        vector<float> inter = MatMuliti(cubic_M, G_column[i]);
        //T * (M*G) //1x4_4x1
        interPoint.push_back(MatMuliti(T,inter));
    }
    return interPoint;
}
leader::~leader()
{
    //dtor
}


