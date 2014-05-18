#include <vector>
using namespace std;

///Matrix Operation
vector<float> MatMuliti(vector < vector<float> >,vector<float>);
float MatMuliti(vector<float>, vector<float>);
vector < vector <float> > MatMuliti(vector < vector <float> >,
                                  vector < vector <float> >);

vector <float> Translate1D(vector <float>);
///vector Operation
vector<float> Normalize(vector<float>);
float vecLenth(vector<float>);
float pointLength(vector<float>,vector<float>);
vector <float> cross_product3D(vector<float> ,vector<float> );
float dot_product3D(vector<float> ,vector<float> );

///OpenGL
void handleKeypress( unsigned char key, int x, int y );
void reshape( int w, int h );
void init();
void setLight_0();
void timer( int value );
