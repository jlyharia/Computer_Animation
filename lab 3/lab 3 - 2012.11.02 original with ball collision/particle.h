//copy right
#ifndef PARTICLE_H_INCLUDED
#define PARTICLE_H_INCLUDED
#include <vector>
using namespace std;
class particle
{
    public:
        particle()
        {
            vector <float> temp(3);
            p = temp;
            v = temp;
            a = temp;
            a[1] = -9.8;//gravity toward negtive y
        }
        virtual ~particle(){};

        vector <float> p;//position
        vector <float> v;//velocity
        vector <float> a;//acceleration
        void setPos(float x,float y,float z)
        {//initialze position
            p[0] = x;
            p[1] = y;
            p[2] = z;
        }
        void setVel(float x,float y,float z)
        {//initialze position
            v[0] = x;
            v[1] = y;
            v[2] = z;
        }
    private:
};
#endif // PARTICLE_H_INCLUDED

