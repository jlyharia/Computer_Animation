#ifndef LEADER_H
#define LEADER_H
#include <vector>
using namespace std;

class leader
{
    public:
        leader();
        vector<float> getPosition();
        vector<float> getMatrix();
        void update(float);
        virtual ~leader();
    private:
        void setControlPoint();
        vector <float> Spline(vector< vector<float> >,float);

        vector< vector<float> > controlPoint;
        vector< vector<float> > cubic_M;
        int Con_number;//number of control points
        int current_curve;
        float parameter_t;//(0~1)

        vector<float> leaderMatrix;
        vector<float> leaderPosition;

        vector< vector<float> > find_G();
    protected:
};

#endif // LEADER_H
