#include <GL/glew.h>
#include <GL/glut.h>
#include <string>
using namespace std;
//Glew Functions
int printOglError(char *file, int line);
void printShaderInfoLog(GLuint obj);
void printProgramInfoLog(GLuint obj);

GLuint CreateShaders(string file_name);


char *textFileRead(char *fn);
int textFileWrite(char *fn, char *s);
