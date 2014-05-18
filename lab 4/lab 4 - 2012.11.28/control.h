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


        void update(float,vector<float>);
        //update

        vector < vector <float> > getMatrix();
        //return final matrix
        void setRoomTranslate(float x,float y, float z);
        void setRoomScale(float x,float y, float z);
        vector<particle> par;

        void setRoomBoundary();

        void setParticleNumber(int num);
        void setObstacleInfo(float, float, float, float);

    private:
        void objMatrix();
        //return final martix for particle

        void m_collision_walls(particle &);
        vector<float> m_distanceToWall(particle boid);
        vector<float> m_full_weighted_sum(vector< vector<float> > input);

        void m_Target_following(particle &);
        void m_collision_obstacle(particle &);
        void m_collision_boids(void);
        void m_collision(particle &);
        vector<float> m_weighted_sum(vector< vector<float> >,
                                     vector<float> ,int, float);
        void m_priority_force(particle &);
        void m_Fin_position(particle &,float);
        void m_Velocity_matching(particle &, int,float);
        void m_Flock_centering(particle &boid);
        vector<float> m_weighted_v(vector<float> velocity);

        void nextPos(float);//find next position
        float floor_pos;//floor translate

        //set particle information(Position & velocity)
        void setParticleInfo();
        vector<float> m_RandomPar();

        vector < vector <float> > parMartix;
        vector < vector <float> > room_bound;
        vector < vector <float> > room_normal;
        float del_t;//current time frame of particle system
        vector<float> target;
        //set room boundary
        int massNum;//number of particles
        float par_radius;//particle radius
        float room_heght;
        float room_width;
        float room_length;
        float v_limit;
        vector <float> room_trans;
        vector <float> room_scale;

        vector <float> Obstacle_pos;
        float Obstacle_size;
        bool boidsEvade;

        float sc_cw;//scalar for collision wall
        float sc_cb;//scalar for avoid from boids
        float sc_co;//scalar for avoid from obstacle
        float sc_vm;//scalar for velocity maching
        float sc_fc;//scalar for flock centering
        float sc_tf;//scalar for target following

};

#endif // CONTROL_H
