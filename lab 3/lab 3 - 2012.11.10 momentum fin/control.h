#ifndef CONTROL_H
#define CONTROL_H
#include "particle.h"
#include <vector>
using namespace std;

class control
{
    public:
        control(float);
        virtual ~control();
        float del_t;//current time of particle system


        void objMatrix();
        //return final martix for particle

        void update(float);
        //update
        vector<float> m_reflect_vec(vector<float>, vector<float>);
        vector < vector <float> > getMatrix();
        //return final matrix
        vector <float> m_check_collision();
        void setRoomBoundary(float,float, float, float);
        //set room boundary
        vector<particle> par;
        //set energy emission
        void setEnergyEmis(float);

        void setParticleNumber(int num);
    private:
        //check particle collide with walls
        void m_collision_walls(vector <float> &, vector <float> &, particle &);
        //process gravity law
        void m_gravity(vector <float> &, vector <float> &,float t, particle &);

        void m_collision_par(vector <float> &, vector <float> &,
                              particle , int);


        void momentum(particle , particle &,vector <float> &);

        void nextPos(float);//find next position
        float floor_pos;//floor translate

        //set particle information(Position & velocity)
        void setParticleInfo();
        vector<float> m_RandomPar();

        vector < vector <float> > parMartix;
        vector < vector <float> > room_bound;
        vector < vector <float> > room_normal;
        float energyEmis;//enegy emission coefficient
        int massNum;//number of particles
        float par_radius;//particle radius
        float room_heght;
        float room_width;
        float room_length;
        float floor_trans;
};

#endif // CONTROL_H
