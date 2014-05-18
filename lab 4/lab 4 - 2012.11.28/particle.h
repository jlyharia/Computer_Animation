//copy right
#ifndef PARTICLE_H_INCLUDED
#define PARTICLE_H_INCLUDED
#include <vector>
using namespace std;
class particle
{
    public:
        particle()
        {//initialize particle information
            vector <float> temp(3);
            p = temp;
            v = temp;
            a_ca = temp;
            a_cw = temp;
            a_cb = temp;
            a_co = temp;

            a_vm = temp;
            a_fc = temp;
            a_tf = temp;
            a_fin = temp;
            energy = 1000;
            candidate_cb.clear();
        }
        virtual ~particle(){};
        float energy;
        vector <float> p;//position
        vector <float> v;//velocity
        vector <float> a_fin;//final acceleration
        vector <float> a_ca;//acceleration collision avoid

        vector <float> a_cw;//acceleration collision wall
        vector <float> a_cb;//acceleration collision boids
        vector<vector <float> > candidate_cb;
        vector <float> a_co;//acceleration collision obsticle


        vector <float> a_vm;//acceleration velocity matching
        vector <float> a_fc;//acceleration flock centering
        vector <float> a_tf;//acceleration target following
        float mass;//mass of particle

        void setPos(vector<float> input)
        {//initialze position
            p = input;
        }
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
        void setVel(vector<float> input)
        {//set position
            v = input;
        }
    private:
};
#endif // PARTICLE_H_INCLUDED

