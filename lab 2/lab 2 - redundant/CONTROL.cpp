#include "CONTROL.h"
#include "HUMAN.h"
#include "trivial.h"
#include <math.h>
#include <vector>
#include <iostream>
#define PI 3.14159265
#define B_splines 0
#define CatmullRom 1
#define FORWARD 1
#define BACKWARD -1
using namespace std;
CONTROL::CONTROL()
{
    setHierarchy();
    setlegSpeed(3.0);
}

void CONTROL::iniProcess(float t, int SplineType,
                          vector < vector<float> > Vec_PosInfo)
{
    //convert to quaternion type - 7 numbes

    vector < vector<float> > QuatVector = fixToQuat(Vec_PosInfo);

    int posType = QuatVector.size();
    vector < vector<float> > tangentVector;
    //Cubic interpolation not only return interpolation point but compute
    //tangent vectorby passing reference
    vector < vector<float> > interPosInfo = Spline(t, posType, SplineType, Vec_PosInfo,
                                                   tangentVector);
    //nomalize
    vector < vector<float> > unit_interPosInfo = QuatNormalize(interPosInfo);

    //number of interpolation point
    frameNumber = (int)tangentVector.size();
    //-----compute body matrix--
    Body.Matrix = combineTangent(unit_interPosInfo,tangentVector);
    //-----compute Right Leg----
    combineLeg(FORWARD, R_leg);
    //-----compute Left Leg----
    combineLeg(BACKWARD, L_leg);
}
void CONTROL::combineLeg(int dir, HUMAN &leg)
{//combine right leg matrix and body matrix
    //dir = 1 if FORWARD, -1 if BACKWARD
    float x = leg.iniTranslate[0];
    float y = leg.iniTranslate[1];
    vector<float> uni;
    vector < vector<float> > output;
    vector < vector<float> > T1 = idenMatrix(4);
    vector < vector<float> > T3 = T1;
    //find movement
    T1[1][3] = y/2;
    T3[0][3] = x;
    T3[1][3] = y/2;
    float t;
    //get parent's matrix
    vector < vector<float> > parentMatrix = leg.parent->Matrix;
    for(int i = 0; i<frameNumber;i++)
    {
        //time function for change leg movement
        t = sin(legSpeed*i*PI/180)*40 *dir;
        vector < vector<float> > T2 = Rotation_X(t);//rotation matrix
        //combine matrix in local coordinate system//T = T3 x T2 x T1
        vector < vector<float> > localMatrix = MatMuliti(MatMuliti(T3,T2),T1);
        //combine local and body to world coordinate system
        vector < vector<float> > worldMatrix =
            MatMuliti(ColumnToRow(parentMatrix[i]),localMatrix);
        //row to column
        uni = RowToColumn(worldMatrix);
        //put back
        output.push_back(uni);
    }
    leg.Matrix = output;
}
void CONTROL::set_iniTrans_R_leg(float x , float y, float z)
{
    R_leg.set_iniTranslate(x , y, z);
}
void CONTROL::set_iniTrans_L_leg(float x , float y, float z)
{
    L_leg.set_iniTranslate(x , y, z);
}
vector < vector<float> > CONTROL::getBodyMatrix()
{
    return Body.Matrix;
}
vector < vector<float> > CONTROL::getRLegMatrix()
{
    return R_leg.Matrix;
}
vector < vector<float> > CONTROL::getLLegMatrix()
{
    return L_leg.Matrix;
}
vector < vector<float> >CONTROL::combineTangent
    (vector < vector<float> >unit_interPosInfo, vector < vector<float> >tangentVector)
{//combine Tangent vector and interpolation vector
    vector < vector<float> > output;
    vector < vector<float> > Mat_Tangent;
    vector < vector<float> > Mat_PosInfo;
    vector < vector<float> > Mat_combine;
    vector<float> uni;
    for(int i = 0; i<(int)tangentVector.size(); i++)
    {
        //compute tangent matrix
        Mat_Tangent = TangenToMat(tangentVector[i]);
        //compute interPosInfo matrix
        Mat_PosInfo = QuatToMatrix(unit_interPosInfo[i]);
        //replace rotation part to Tangent
        Mat_combine = ReplaceTangent(Mat_PosInfo,Mat_Tangent);
        //4x4 row major to 1-D column major
        uni = RowToColumn(Mat_combine);
        //push_back
        output.push_back(uni);
    }
    return output;
}
vector < vector <float> > CONTROL::ReplaceTangent(vector < vector <float> > mat_1,
                                  vector < vector <float> > mat_Tan)
{
    for(int i = 0;i<3;i++)
        for(int j = 0;j<3;j++)
            mat_1[i][j] = mat_Tan[i][j];

    return mat_1;
}
void CONTROL::setlegSpeed(float speed)
{
    legSpeed = speed;
}
void CONTROL::setHierarchy()
{
    //set hierarchy
    Body.parent = NULL;
    R_leg.parent = &Body;
    L_leg.parent = &Body;
}
CONTROL::~CONTROL()
{
    //dtor
}
