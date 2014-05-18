#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <GL/glut.h>
using namespace std;
vector < vector<float> > fixToQuat(vector < vector<float> >);
vector <float> QuatMulti(vector <float>,vector <float>);
vector <float> cross_product3D(vector<float> ,vector<float> );
float dot_product3D(vector<float> ,vector<float> );

///Quaternion Functions
vector < vector<float> > QuatNormalize(vector < vector<float> >);
vector < vector<float> > QuatToMatrix(vector<float>);
void MatrixRowToColumn (GLfloat* , vector<float>);

///Cubic Curve Function
vector< vector <float> > Spline(float, int, int,
                                vector < vector<float> >,
                                vector < vector<float> >&);
//Catmull Rom Spline
vector< vector <float> > M_Catmull_Rom();
//B Spline
vector< vector <float> > M_B_Spline();
//Tangent
vector< vector <float> > TangenToMat(vector<float>);

///Fixed Angle Functions
vector < vector<float> > Rotation_X(float);
vector < vector<float> > Rotation_Y(float);
vector < vector<float> > Rotation_Z(float);
vector < vector<float> > Translation(float x, float y, float z);

vector < vector<float> >  FiexedToMatrix(vector < vector<float> >);
///input process

///Matrix Operation
vector<float> MatMuliti(vector < vector<float> >,vector<float>);
float MatMuliti(vector<float>, vector<float>);
vector < vector <float> > MatMuliti(vector < vector <float> >,
                                  vector < vector <float> >);

vector <float> RowToColumn(vector < vector <float> >);
vector < vector <float> >ColumnToRow(vector <float>);
vector < vector <float> > idenMatrix(int d);
///vector Operation
vector<float> Normalize(vector<float>);
float vecLenth(vector<float>);


