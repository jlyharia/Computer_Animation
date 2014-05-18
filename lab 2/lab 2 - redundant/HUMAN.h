#ifndef HUMAN_H_INCLUDED
#define HUMAN_H_INCLUDED
#include <vector>
using namespace std;
class HUMAN
{
    public:
        HUMAN(){};
        virtual ~HUMAN(){};
        vector<float> iniTranslate; //for initialize
        vector < vector<float> > Matrix; //dependent matrix for this object
        HUMAN *parent;
        void set_iniTranslate(float x , float y, float z)
        {
            iniTranslate.push_back(x);
            iniTranslate.push_back(y);
            iniTranslate.push_back(z);
        }
    private:
    protected:
};

#endif // HUMAN_H_INCLUDED
