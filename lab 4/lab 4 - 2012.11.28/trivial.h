#include <vector>
using namespace std;
///Cubic Curve Function
//Catmull Rom Spline
vector< vector <float> > M_Catmull_Rom();
//B Spline
vector< vector <float> > M_B_Spline();



///Matrix Operation
vector<float> MatMuliti(vector < vector<float> >,vector<float>);
float MatMuliti(vector<float>, vector<float>);
vector < vector <float> > MatMuliti(vector < vector <float> >,
                                  vector < vector <float> >);

vector < vector <float> > Translate(vector <float>);
vector <float> Translate1D(vector <float>);
///vector Operation
float vecLenth(vector<float>);
float pointLength(vector<float>,vector<float>);
vector <float> cross_product3D(vector<float> ,vector<float> );

float dot_product3D(vector<float> ,vector<float> );
vector <float> pointMinus(vector<float> ,vector<float>);
vector<float> Normalize(vector<float> );
