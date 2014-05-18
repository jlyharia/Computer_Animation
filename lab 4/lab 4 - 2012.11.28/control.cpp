#include "control.h"
#include "particle.h"
#include "trivial.h"
#include <vector>
#include <cmath>
#include <iostream>
#include <ctime>
#include <stdlib.h>
#include <iomanip>
using namespace std;
control::control(float ballSize)
{
    par_radius = ballSize;//set particle
    //set velocity limit
    v_limit = 30;
    ///---------set scalar
    sc_cw = 10000;//scalar for avoid from wall
    sc_cb = 10000;//scalar for avoid from boids
    sc_co = 100000;//scalar for avoid from obstacle
    sc_vm = 1;//scalar for velocity maching
    sc_fc = 5;//scalar for flock centering
    sc_tf = 10;//scalar for target following
}
vector<float> control::m_RandomPar()
{//return a ramdon position limited by room size and particle length
    vector<float> scale_info = room_scale;
    vector<float> trans_info = room_trans;
    vector<float> pos(3);
    //z
    pos[2] = rand() % (int)(scale_info[2] - 2 * par_radius);
    pos[2] = -pos[2] - par_radius - 30;
    //x
    pos[0] = rand() % (int)(scale_info[0] - 2 * par_radius);
    pos[0] = pos[0] - scale_info[0]/2 + par_radius;
    //y
    pos[1] = rand() % (int)(scale_info[1] - 2 * par_radius);
    pos[1] = pos[1] + trans_info[1] + par_radius;

    return pos;
}
void control::setParticleInfo()
{//set particle information 1)Position 2) velocity
    //spread seed
    srand ( time(0) );
    float vx,vy,vz;
    vector<float> pos(3);
    float diameter = 2 * par_radius;
    for(int i = 0 ; i < massNum ; i++)
    {//ramdon position limited in room
        //set ramson position
        pos = m_RandomPar();
        for(int j = 0 ; j< i; j++)
        {//check if current particle inside other particles
            if(pointLength(pos, par[j].p) < diameter)
            {
                pos = m_RandomPar();//reset position
                j = 0;
            }
        }
        par[i].setPos(pos[0],pos[1],pos[2]);
        //ramdon velocity limited between -20 and 20
        vx = (rand() % 40) -20;
        vy = (rand() % 40) -20;
        vz = (rand() % 40) -20;
        par[i].setVel(vx,vy,vz);
        //set particle mass
        par[i].mass = par_radius;
    }
}
void control::update(float t,vector<float> input)
{//update particle system
    //update target position
    target = input;
    //step 1 find next position
    nextPos(t);
    //step 2 put in the matrix form
    objMatrix();
}
void control::objMatrix()
{//transform boids' position to OpenGL's matrix form
    parMartix.clear();
    vector <float> uni;
    for(int i = 0; i < (int)par.size(); i++)
    {
        uni = Translate1D(par[i].p);
        parMartix.push_back(uni);
    }
}

void control::m_collision(particle &boid)
{//compute collision avoidance
    //collision avoid from walls
    m_collision_walls(boid);
    //collision avoid from obstacle
    m_collision_obstacle(boid);
    if(boidsEvade)
    {//collision avoid from boids
        m_collision_boids();
    }

    vector< vector<float> > avoidForce;
    avoidForce.push_back(boid.a_cw);
    avoidForce.push_back(boid.a_cb);
    avoidForce.push_back(boid.a_co);
    boid.a_ca = m_full_weighted_sum(avoidForce);
}
void control::m_collision_obstacle(particle &boid)
{//compute collision between boid and obstacle
    vector<float> v1 = boid.v;
    vector<float> p1 = boid.p;
    vector<float> p2 = Obstacle_pos;
    vector<float> output(3);
    float evadeRadius = Obstacle_size * 3;
    //s = P1 -> obstacle
    vector<float> s = pointMinus(p1,p2);
    float s_length = vecLenth(s);
    if(s_length < evadeRadius)
    {//enter avoiding zone
        //k = s dot V/|V|
        float k = dot_product3D(s,Normalize(v1));
        float t = sqrt(pow(s_length,2) - pow(k,2));
        if(t < Obstacle_size)
        {//potentail collision
            //calculate avoidance direction by simplified algorithm
            vector<float> dir = pointMinus(Normalize(s),Normalize(v1));
            if(vecLenth(dir) == 0)
            {//boid is heading the center of the obstacle
                vector<float> temp(3,1);
                dir = temp; //evade in given direction
            }
            dir = Normalize(dir);
            //change
            //scalar r
            float r = sc_co/pow(s_length,2);
            for(int i = 0 ; i<3; i++)
            {
                output[i] = dir[i] * r;
            }
            boid.a_co = output;
        }
    }

}
void control::m_collision_boids(void)
{//evade boids each other
    ///testing for potential collision with a bounding sphere
    float evadeRadius = par_radius * 5;
    for(int i = 0; i< massNum;i++)
    {
        vector<float> v1 = par[i].v;
        vector<float> p1 = par[i].p;
        for(int j = i+1; j< massNum;j++)
        {
            vector<float> p2 = par[j].p;
            //s = P1 -> P2
            vector<float> s = pointMinus(p1,p2);
            float s_length = vecLenth(s);
            if(s_length < evadeRadius)
            {//enter avoiding zone
                //k = s dot V/|V|
                float k = dot_product3D(s,Normalize(v1));
                float t = sqrt(pow(s_length,2) - pow(k,2));
                if(t < par_radius)
                {//potentail collision
                    //calculate avoidance direction by simplified algorithm
                    vector<float> dir = pointMinus(Normalize(s),Normalize(v1));
                    if(vecLenth(dir) == 0)
                    {//two boids have opposite direction
                        vector<float> temp(3,1);
                        dir = temp; //evade in given direction
                    }
                    dir = Normalize(dir);
                    //change
                    vector<float> a_cb1(3);
                    vector<float> a_cb2(3);
                    //scalar r
                    float r = sc_cb/pow(s_length,2);
                    for(int f = 0 ; f<3; f++)
                    {
                        a_cb1[f] = dir[f] * r;
                        a_cb2[f] = a_cb1[f] * -1;
                    }
                    par[i].candidate_cb.push_back(a_cb1);
                    //another boid
                    par[j].candidate_cb.push_back(a_cb2);

                }
            }
        }// end for(int j = i+1; j< massNum;j++)
        //weighted sum of all boids avoidance force for one boid
        par[i].a_cb = m_full_weighted_sum(par[i].candidate_cb);
    }

}

vector<float> control::m_full_weighted_sum(vector< vector<float> > input)
{//output weighted sum in genral
    vector<float> output(3);
    float total = 0;
    int inputSize = (int)input.size();
    for(int i = 0 ; i< inputSize ;i++)
    {//collect total length
        total += vecLenth(input[i]);
    }
    //weighted sum
    for(int i = 0 ; i< inputSize ;i++)
    {
        //find the scale to certain vector
        float scale = vecLenth(input[i])/total;
        for(int j = 0; j < 3; j++)
        {
            output[j] += input[i][j] * scale;
        }
    }
    return output;
}
vector<float> control::m_distanceToWall(particle boid)
{//find distance between boid and walls
    vector<float> p = boid.p;
    vector<float> wallDistance(6);
    //0)front z
    wallDistance[0] = abs(p[2] - room_bound[0][2]);
    //1)back z
    wallDistance[1] = abs(p[2] - room_bound[1][2]);
    //2)right   x
    wallDistance[2] = abs(p[0] - room_bound[2][0]);
    //3)left    x
    wallDistance[3] = abs(p[0] - room_bound[3][0]);
    //4)floor   y
    wallDistance[4] = abs(p[1] - room_bound[4][1]);
    //5)top y
    wallDistance[5] = abs(p[1] - room_bound[5][1]);
    return  wallDistance;
}
void control::m_collision_walls(particle &boid)
{//output - acceleration for boid prevent from collision with walls
    //input - boid's position
    //alg - 1/r^2
    vector<float> output(3);
    vector< vector<float> > wallForce(6,vector<float>(3));
    vector<float> wallDistance = m_distanceToWall(boid);
    //distance between boid and walls
    //0)front 1)back 2)right 3)left 4)floor 5)top
    vector<float> p = boid.p;
    for(int i =0; i<6; i++)
    {//6 walls
        //find the scalar
        float r = wallDistance[i];
        r = sc_cw/pow(r,2);
        for(int j = 0; j<3; j++)
            wallForce[i][j] = room_normal[i][j] * r;
    }
    //weighted sum
    output = m_full_weighted_sum(wallForce);
    ///updata avoidance acceleration collision wall
    boid.a_cw = output;
}
vector<float> control::m_weighted_sum(vector< vector<float> > a,
                                  vector<float> a_length,int num, float limit)
{//compute weighted sum
    float total = 0;
    vector<float> output(3);
    if(num == 3)
    {//all energy consume smaller than limit
        for(int i = 0;i<4;i++)
        {//get all energy consume
            total += a_length[i];
        }
        for(int i = 0;i<4;i++)
        {//find weight for each acceleration
            float weight = a_length[i]/total;
            for(int j = 0;j<3;j++)
            {//weighted sum
                output[j] += weight * a[i][j];
            }
        }
    }
    else if(num == 0)
    {//only one force(top priority) can work and
        //it exceed the energy limit
        //find a scalar to meet energy limit
        float scalar = limit/a_length[num];
        for(int i = 0 ; i < 3 ; i++)
        {
            output[i] += scalar * a[num][i];
        }
    }
    else//num < 3
    {//one force energy consume larger than limit
        //rest of force small than energy consume
        for(int i = 0;i < num ;i++)
        {//weighted sum for those accelerations have full access to energy
            float weight = a_length[i]/limit;
            for(int j = 0 ; j < 3 ; j++)
            {
                output[j] += weight * a[i][j];
            }
        }
        float rest = limit;
        for(int i = 0 ; i < num ; i++)
        {//find the rest available energy
            rest -= a_length[i];
        }
        //weighted sum for the acceleration has part access to energy
        //find scalar to meet the rest energy
        float scalar = rest/vecLenth(a[num]);
        //find the weight of this energy consume
        float weight = rest/limit;
        for(int i = 0 ; i < 3 ; i++)
        {
            output[i] += weight * scalar * a[num][i];
        }
    }
    return output;
}

vector<float> control::m_weighted_v(vector<float> velocity)
{//make velocity smaller than velocity limit
    float scalar = v_limit/vecLenth(velocity);
    for(int i =0 ; i < 3; i++)
    {
        velocity[i] *= scalar;
    }
    return velocity;
}

void control::m_Target_following(particle &seeker)
{//set acceleration for boid target seeking
    //input - 1)target's position 2)seeker's position
    //alg - r^2
    //unit vector point to target//direction from boid to target
    vector<float> dir = Normalize(pointMinus(seeker.p,target));
    //scalar r
    float r = pointLength(target, seeker.p);
    r = pow(r,2)/sc_tf;
    vector<float> output = dir;
    for(int i = 0;i< (int)dir.size(); i++)
        output[i] = dir[i] * r;
    ///set acceleration for target following
    seeker.a_tf = output;
}
void control::m_Velocity_matching(particle &boid, int current, float t)
{//make current boid change acceleration to match nearby boids' velocity
    //find nearby boids and average their velocity
    //set velocity maching zone for certain boid
    vector <float> boid_v = boid.v;
    float vm_zone = 5 * par_radius;
    vector<vector <float> > nearboid;
    for(int i = 0; i < massNum; i++)
    {//search all boids
        if(i == current)
        {//skip comparing to itself
            continue;
        }
        else if(pointLength(boid.p,par[i].p) <= vm_zone)
        {//collect nearby boids' velocity
            nearboid.push_back(par[i].v);
        }
    }
    vector<float> a(3);
    if((int)nearboid.size() != 0)
    {//no neighbor boids
        //get all velocity //weighted sum
        vector<float> goal_v = m_full_weighted_sum(nearboid);

        //find acceleration
        for(int i =0 ; i < 3; i++)
        {
            //a = (v'- v)/t
            a[i] = (sc_vm*(goal_v[i] - boid_v[i]))/t;
        }
    }
    ///set acceleration for Velocity matching
    boid.a_vm = a;
}
void control::m_Flock_centering(particle &boid)
{//find the acceleration for flock centering
    vector<float> pos;
    vector<float> pos_center(3);
    vector<float> output(3);
    if(massNum != 1)
    {

        for(int i = 0; i<massNum; i++)
        {//get all position of boids
            pos = par[i].p;
            for(int j = 0; j<3; j++)
            {
                pos_center[j] += pos[j];
            }
        }
        for(int j = 0; j<3; j++)
        {//find center position
            pos_center[j] /= massNum;
        }
        //find acceleration direction
        vector<float> dir = Normalize(pointMinus(boid.p,pos_center));
        //scalar r
        float r = pointLength(boid.p,pos_center);
        r = pow(r,2)/sc_fc;
        output = dir;
        for(int i = 0;i< 3; i++)
            output[i] = dir[i] * r;
    }
    ///set acceleration for flock centering
    boid.a_fc = output;
}

void control::m_priority_force(particle &boid)
{//determine the force priority
    vector< vector<float> > a;
    vector<float> a_length(4);
    a.push_back(boid.a_ca);//0 acceleration collision avoidance
    a.push_back(boid.a_vm);//1 acceleration velocity matching
    a.push_back(boid.a_fc);//2 acceleration flock centering
    a.push_back(boid.a_tf);//3 acceleration target following

    float Limit = boid.energy;
    float energy = 0;
    for(int i = 0;i<4;i++)
    {
        float length = vecLenth(a[i]);
        energy += length;
        a_length[i] = length;
        if(energy > Limit)
        {//if energy consume larger than limit
            boid.a_fin = m_weighted_sum(a,a_length,i,Limit);
            break;
        }
        else if(energy <= Limit && (i == 3))
        {//if all energy consume smaller than limit
            boid.a_fin = m_weighted_sum(a,a_length,i,Limit);
            break;
        }
    }
}
void control::m_Fin_position(particle &boid,float t)
{//set final position and velocity
    //p1 -> next position, v1 ->next velocity
    //t ->delta t
    vector <float> p0 = boid.p;//current position
    vector <float> v0 = boid.v;//current velocity
    vector <float> p1(3);
    vector <float> v1(3);
    vector <float> a = boid.a_fin; //acceleration
    for(int i =0 ; i < 3; i++)
    {
        //v' = v + at
        v1[i] = v0[i] + a[i]*t;
    }
    if(vecLenth(v1) > v_limit)
    {//keep velocity bound by velocity limit
        v1 = m_weighted_v(v1);
    }
    for(int i =0 ; i < 3; i++)
    {
        //p' = p + (v + v')t/2;
        p1[i] = p0[i] + (v0[i]+v1[i])*t*0.5;
    }
    ///set final position and velocity
    boid.p = p1;
    boid.v = v1;
}
void control::nextPos(float t)
{//find next position
    for(int i = 0; i< massNum;i++)
    {//i -> current particle
        i == 0 ? boidsEvade = true: boidsEvade=false;
        ///1. Collision avoidance
        m_collision(par[i]);
        ///2. Velocity matching
        m_Velocity_matching(par[i],i,t);
        ///3. Flock centering
        m_Flock_centering(par[i]);
        ///4. Target following
        m_Target_following(par[i]);
        ///5. Force Priority
        m_priority_force(par[i]);
        ///6. final position
        m_Fin_position(par[i],t);
    }
}
vector < vector <float> > control::getMatrix()
{//return the final matrix of boids
    return parMartix;
}
void control::setRoomTranslate(float x,float y, float z)
{//set room translation
    vector<float> temp(3);
    temp[0] = x;
    temp[1] = y;
    temp[2] = z;
    room_trans = temp;
}
void control::setRoomScale(float x,float y, float z)
{//set room scalse
    vector<float> temp(3);
    temp[0] = x;
    temp[1] = y;
    temp[2] = z;
    room_scale = temp;
}
void control::setRoomBoundary()
{//set romm boundary
    room_bound.clear();
    vector<float> uni(3);
    //front -100
    uni[2] = room_trans[2];
    room_bound.push_back(uni);
    //back  -30
    uni[2] = room_scale[2] + room_trans[2];
    room_bound.push_back(uni);
    //right 50
    uni[0] = room_trans[0] + room_scale[0];
    room_bound.push_back(uni);
    //left  -50
    uni[0] = room_trans[0];
    room_bound.push_back(uni);
    //floor -20
    uni[0] = 0;
    uni[1] = room_trans[1];
    room_bound.push_back(uni);
    //top   30
    uni[1] = room_trans[1] + room_scale[1];
    room_bound.push_back(uni);
    ///------------------
    room_normal.clear();
    vector<float> nor(3);
    //front
    nor[2] = 1;
    room_normal.push_back(nor);
    //back
    nor[2] = -1;
    room_normal.push_back(nor);
    //right
    nor[2] = 0;
    nor[0] = -1;
    room_normal.push_back(nor);
    //left
    nor[0] = 1;
    room_normal.push_back(nor);
    //floor
    nor[0] = 0;
    nor[1] = 1;
    room_normal.push_back(nor);
    //top
    nor[1] = -1;
    room_normal.push_back(nor);
    ///----------------
    setParticleInfo();
}
void control::setObstacleInfo(float x,float y, float z, float ob_size)
{//set obstacle information
    vector<float> temp(3);
    temp[0] = x;
    temp[1] = y;
    temp[2] = z;
    Obstacle_pos = temp;
    Obstacle_size = ob_size;
}
void control::setParticleNumber(int num)
{//set number of particle
    massNum = num;
    vector<particle> patern(num);
    par = patern;
}
control::~control()
{
    //dtor
}
