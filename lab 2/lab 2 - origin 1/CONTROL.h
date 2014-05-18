#ifndef CONTROL_H
#define CONTROL_H
#include "HUMAN.h"
#define B_splines 0
#define CatmullRom 1
class CONTROL
{
    public:
        CONTROL();
        void setHierarchy();

        //initial process
        void iniProcess(float , int,vector < vector<float> > );

        //combine tangent vector and position information
        vector < vector<float> > combineTangent(vector < vector<float> >,
                                               vector < vector<float> >);

        void combineLeg(int, HUMAN &);
        //combine right leg matrix and body matrix
        vector < vector<float> > getBodyMatrix();
        vector < vector<float> > getRLegMatrix();
        vector < vector<float> > getLLegMatrix();

        void set_iniTrans_R_leg(float, float, float);
        void set_iniTrans_L_leg(float, float, float);
        void setlegSpeed(float);
        HUMAN Body,R_leg,L_leg;
        virtual ~CONTROL();

    private:
        float Power_T;
        int Spline_type;
        int frameNumber;
        float legSpeed;
        vector < vector<float> > QuatVector;
};

#endif // CONTROL_H
