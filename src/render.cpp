#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>

//include header file for glfw library so that we can use OpenGL
#include <GLFW/glfw3.h>
#include <GL/glut.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "data_structures.cpp"

#include "eigen/Eigen/Dense"

using Eigen::MatrixXf;
using Eigen::Matrix4f;
using Eigen::Matrix3f;
using Eigen::Vector4f;
using Eigen::Vector3f;
using Eigen::VectorXf;
using namespace std;

#ifdef _WIN32
static DWORD lastTime;
#else
static struct timeval lastTime;
#endif

#define PI 3.14159265 // Should be used from mathlib

using namespace std;

/*
For UC Berkeley's CS184 Fall 2016 course, assignment 3 (Bezier surfaces)
*/

//****************************************************
// Global Variables
//****************************************************
GLfloat translation[3] = {0.0f, 0.0f, 0.0f};
GLfloat scale[3] = {1.0f, 1.0f, 1.0f};
vector<Triangle*> triangles;
vector<vector<float> > rotations = *(new vector<vector<float> >());\
bool auto_strech = false;
int Width_global = 400;
int Height_global = 400;
int Z_buffer_bit_depth = 128;
float EPSILON = .001;
float numVertices = 0;
int numTriangles = 0;



//****************************************************
// Simple init function
//****************************************************
void initializeRendering()
{
    glfwInit();
}

//****************************************************
// A routine to set a pixel by drawing a GL point.  This is not a
// general purpose routine as it assumes a lot of stuff specific to
// this example.
//****************************************************
void setPixel(float x, float y, GLfloat r, GLfloat g, GLfloat b) {
    glColor3f(r, g, b);
    glVertex2f(x+0.5, y+0.5);  // The 0.5 is to target pixel centers
    // Note: Need to check for gap bug on inst machines.
}

//****************************************************
// Keyboard inputs. Add things to match the spec! 
//****************************************************
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    switch (key) {
        

        case GLFW_KEY_EQUAL:
          if (action && mods == GLFW_MOD_SHIFT){
            scale[0] += 0.01f;
            scale[1] += 0.01f;
            scale[2] += 0.01f;
          } break;
        case GLFW_KEY_MINUS:
          scale[0] -= 0.01f;
          scale[1] -= 0.01f;
          scale[2] -= 0.01f;
          break;
        case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(window, GLFW_TRUE); break;
        case GLFW_KEY_Q: glfwSetWindowShouldClose(window, GLFW_TRUE); break;
        case GLFW_KEY_LEFT :
          if (action && mods == GLFW_MOD_SHIFT){
            translation[0] -= 0.001f * Width_global;
            break;
          } else{
            vector<float> to_push;
            to_push.push_back(10);
            to_push.push_back(1);
            to_push.push_back(0);
            to_push.push_back(0);
            rotations.push_back(to_push);
            break;
          }
        case GLFW_KEY_RIGHT:
          if (action && mods == GLFW_MOD_SHIFT){ 
            translation[0] += 0.001f * Width_global;
            break;
          }else{
            vector<float> to_push;
            to_push.push_back(-5);
            to_push.push_back(0);
            to_push.push_back(1);
            to_push.push_back(0);
            rotations.push_back(to_push);
            vector<float> to_push2;
            to_push2.push_back(-5);
            to_push2.push_back(1);
            to_push2.push_back(0);
            to_push2.push_back(0);
            rotations.push_back(to_push2);
            break;
          }
        case GLFW_KEY_UP:
          if (action && mods == GLFW_MOD_SHIFT){ 
            translation[1] += 0.001f * Height_global;
            break;
          } else{
            vector<float> to_push;
            to_push.push_back(10);
            to_push.push_back(1);
            to_push.push_back(0);
            to_push.push_back(0);
            rotations.push_back(to_push);
            break;
          }
        case GLFW_KEY_DOWN :
          if (action && mods == GLFW_MOD_SHIFT){ 
            translation[1] -= 0.001f * Height_global;
            break;
          }
          else{
            vector<float> to_push;
            to_push.push_back(-5);
            to_push.push_back(1);
            to_push.push_back(0);
            to_push.push_back(0);
            rotations.push_back(to_push);
            break;
          }
        case GLFW_KEY_F:
          if (action && mods == GLFW_MOD_SHIFT) auto_strech = !auto_strech; break;
        case GLFW_KEY_SPACE: break;
            
        default: break;
    }
    
}

void drawSurfaceAdaptive(vector<Vector4f*> points, int n) {
  glColor3f(0,.5,1);
  glBegin(GL_TRIANGLES);

  for (int i = 0; i < numTriangles; i++) {
    /*if ( i < 6){
      glColor3f(1,0,1);
    } else if (i < 12){
      glColor3f(0, 1, 1);
    } else if (i < 18){
      glColor3f(1,1,0);
    } else if (i < 24){
      glColor3f(0,.5,1);
    }*/
    glNormal3f(triangles[i]->n1->x, triangles[i]->n1->y, triangles[i]->n1->z);
    glVertex3f(triangles[i]->v1.x, triangles[i]->v1.y, triangles[i]->v1.z);

    glNormal3f(triangles[i]->n2->x, triangles[i]->n2->y, triangles[i]->n2->z);
    glVertex3f(triangles[i]->v2.x, triangles[i]->v2.y, triangles[i]->v2.z);

    glNormal3f(triangles[i]->n3->x, triangles[i]->n3->y, triangles[i]->n3->z);
    glVertex3f(triangles[i]->v3.x, triangles[i]->v3.y, triangles[i]->v3.z);
  }

}



//****************************************************
// function that does the actual drawing of stuff
//***************************************************
void display( GLFWwindow* window, vector<Vector4f*> points, int n, Vector4f goal)
{
    
    glClearColor( 0.0f, 0.0f, 0.0f, 0.0f ); //clear background screen to black
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);                // clear the color buffer (sets everything to black)
    glMatrixMode(GL_MODELVIEW);                  // indicate we are specifying camera transformations
    glLoadIdentity();                            // make sure transformation is "zero'd"

    GLfloat mat_specular[] = { 0.0, 0.5, 0.8, 1.0 };
    GLfloat mat_shininess[] = { 50.0 };
    GLfloat light_position[] = { 0.0, 0.0, -1.0, 0.0 };
    
    glShadeModel(GL_SMOOTH);


    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);
    
    //----------------------- code to draw objects --------------------------
    glPushMatrix();
    glTranslatef (translation[0], translation[1], translation[2]);
    glScalef( scale[0], scale[1], scale[2] );
    for(int i = rotations.size()-1; i > -1; i--){
      glRotated(rotations[i][0], rotations[i][1], rotations[i][2], rotations[i][3]);
    }

    drawSurfaceAdaptive(points, n);

    glPopMatrix();

    Vector4f xone{1,0,0,0};
    Vector4f yone{0,1,0,0};
    Vector4f zone{0,0,1,0};
    
    Vector4f pone = goal + xone;
    Vector4f ptwo = goal + yone;
    Vector4f pthree = goal + zone;
    Vector4f pfour = goal - zone;
    Vector4f pfive = goal - yone;
    Vector4f psix = goal - zone;

    vector<Vector4f> octa_points;
    octa_points.push_back(pone);
    octa_points.push_back(ptwo);
    octa_points.push_back(pthree);
    octa_points.push_back(pfour);
    octa_points.push_back(pfive);
    octa_points.push_back(psix);

    vector<Vector4f> octa_normals;
    octa_normals.push_back(pone-goal);
    octa_normals.push_back(ptwo-goal);
    octa_normals.push_back(pthree-goal);
    octa_normals.push_back(pfour-goal);
    octa_normals.push_back(pfive-goal);
    octa_normals.push_back(psix-goal);
    
    int indices[][3] = {{0,1,2},{4,2,1},{4,5,2},{5,3,2},{3,6,1},{6,4,1},{6,5,4},{5,6,3}};
    /*for (int i = 0; i < 8; i++) {
      glBegin(GL_TRIANGLES);
      for(int k = 0; k < 3; k++){
        glNormal3f(octa_normals[indices[i][k]][0], octa_normals[indices[i][k]][1], octa_normals[indices[i][k]][2]);
        glVertex3f(octa_points[indices[i][k]][0], octa_points[indices[i][k]][1], octa_points[indices[i][k]][2]);
      }
      glEnd();
    }*/

    glEnd();

    glPushMatrix();
    glColor3f(0,.5,1.0);
    GLUquadric *quad;
    quad = gluNewQuadric();
    glTranslatef(goal(0), goal(1), goal(2));
    gluSphere(quad, .25, 50, 50);
    
    glfwSwapBuffers(window);

    
    // note: check out glPolygonMode and glShadeModel 
    // for wireframe and shading commands

    
}

//****************************************************
// function that is called when window is resized
//***************************************************
void size_callback(GLFWwindow* window, int width, int height)
{
    // Get the pixel coordinate of the window
    // it returns the size, in pixels, of the framebuffer of the specified window
    glfwGetFramebufferSize(window, &Width_global, &Height_global);
    
    glViewport(0, 0, Width_global, Height_global);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, Width_global, 0, Height_global, 10, -10);
    
}

Matrix3f crossMatrix(Vector4f v) {
  Matrix3f rx(3,3);
  rx(0,0) = 0;
  rx(1,0) = v[2];
  rx(2,0) = -v[1];
  rx(0,1) = -v[2];
  rx(1,1) = 0;
  rx(2,1) = v[0];
  rx(0,2) = v[1];
  rx(1,2) = -v[0];
  rx(2,2) = 0;
  //cout << "crossMatrix: " << endl << rx << endl;
  return rx;
}

Point cross_prod(Point a, Point b){
	return *(new Point(a.y*b.z-b.y*a.z, a.z*b.x-b.z*a.x, a.x*b.y-b.x*a.y));
}

std::vector<Vector4f*> absolute(std::vector<Vector4f*> points, int n){
	std::vector<Vector4f*> to_return;
	Vector4f to_add;
	to_return.push_back(points[0]);
	for(int i = 1; i < n; i++){
		//cout << "absolute, i: " << i << endl;
		to_add = *(new Vector4f(points[i][0][0], points[i][0][1], points[i][0][2], 0));
		//cout << "to_add: " << endl << to_add << endl;
		//cout << "*to_return[i-1]: " << endl << *to_return[i-1] << endl;
		to_add += *to_return[i-1];
		Vector4f* to_push = new Vector4f(to_add[0], to_add[1], to_add[2], 1);
		//cout << "to_add+=: " << endl << to_add << endl;
		to_return.push_back(to_push);
	}
	//cout << "finalized!" << endl;
	/*for(int i = 0; i < n; i++){
		cout << "i: " << i << endl;
		cout << *to_return[i] << endl;
	}*/
	return to_return;
}

vector<Triangle*> armPyramids(vector<Vector4f*> points, int n){
  vector<Triangle*> to_return;
  vector<Triangle*> tri;
  vector<Vector4f*> abs = absolute(points, n);
  for(int i = 0; i < n; i++){
    Vector4f p = *points[i];
    Vector3f one(1, 1, 1);
    Matrix3f cross_p = crossMatrix(p);
    Vector3f p_one = cross_p * one;
    Vector3f p_p_one = cross_p * p_one;
    /*if(i == 0){
      cout << "p: " << endl << p << endl;
      cout << "p_one: " << endl << p_one << endl;
      cout << "p_p_one: " << endl  << p_p_one << endl;
    }*/
    float p_one_norm = p_one.norm()*2;
    float p_p_one_norm = p_p_one.norm()*2;
    p_one /= p_one_norm;
    p_p_one /= p_p_one_norm;
    Point p_0 = *(new Point(p[0], p[1], p[2]));
    Point p_1 = *(new Point(p_one[0], p_one[1], p_one[2]));
    Point p_2 = *(new Point(p_p_one[0], p_p_one[1], p_p_one[2]));
    Point p_3 = p_1*-1;
    Point p_4 = p_2*-1;
    vector<Point> norms;
    norms.push_back(cross_prod(p_2-p_4, p_1-p_4));
    norms.push_back(cross_prod(p_3-p_4, p_2-p_4));
    norms.push_back(cross_prod(p_0-p_2, p_1-p_2));
    norms.push_back(cross_prod(p_0-p_3, p_2-p_3));
    norms.push_back(cross_prod(p_0-p_4, p_3-p_4));
    norms.push_back(cross_prod(p_0-p_1, p_4-p_1));
    /*if(i == 0){
      for(int j = 0; j < 6; j++){
        cout << "norms[j]: " << endl;
        norms[j].print();
      }
    }*/
    /*p_0.print();
    p_1.print();
    p_2.print();
    p_3.print();
    p_4.print();*/
    vector<Normal*> point_normals;
    Point pavg1 = (norms[3] + norms[2] + norms[5] + norms[4])/4;
    Point pavg2 = (norms[0] + norms[2] + norms[5])/3;
    Point pavg3 = (norms[0] + norms[2] + norms[3])/3;
    Point pavg4 = (norms[0] + norms[3] + norms[4])/3;
    Point pavg5 = (norms[0] + norms[4] + norms[5])/3;
    point_normals.push_back(new Normal(pavg1));
    point_normals.push_back(new Normal(pavg2));
    point_normals.push_back(new Normal(pavg3));
    point_normals.push_back(new Normal(pavg4));
    point_normals.push_back(new Normal(pavg5));
    Point* adjust = new Point(0,0,0);
    /*if(i > 0){
    	adjust = new Point((*abs[i-1])[0], (*abs[i-1])[1], (*abs[i-1])[2]);
    }*/
    tri.clear();
    tri.push_back(new Triangle(p_1, p_4, p_2, point_normals[1], point_normals[4], point_normals[2]));
    tri.push_back(new Triangle(p_2, p_4, p_3, point_normals[2], point_normals[4], point_normals[3]));
    tri.push_back(new Triangle(p_1, p_2, p_0, point_normals[1], point_normals[2], point_normals[0]));
    tri.push_back(new Triangle(p_2, p_3, p_0, point_normals[2], point_normals[3], point_normals[0]));
    tri.push_back(new Triangle(p_3, p_4, p_0, point_normals[3], point_normals[4], point_normals[0]));
    tri.push_back(new Triangle(p_4, p_1, p_0, point_normals[4], point_normals[1], point_normals[0]));
  	/*if(i == 2){
      cout << "round 1: " << endl;
      t1->print();
      t2->print();
      t3->print();
      t4->print();
      t5->print();
      t6->print();
    }*/
    /*t1->translate(adjust);
	  t2->translate(adjust);
	  t3->translate(adjust);
	  t4->translate(adjust);
	  t5->translate(adjust);
	  t6->translate(adjust);*/
    if(i != 0){
      for(int j = 0; j < 6; j++){
        tri[j]->v1.x += (*abs[i-1])[0];
        tri[j]->v1.y += (*abs[i-1])[1];
        tri[j]->v1.z += (*abs[i-1])[2];
        tri[j]->v2.x += (*abs[i-1])[0];
        tri[j]->v2.y += (*abs[i-1])[1];
        tri[j]->v2.z += (*abs[i-1])[2];
        tri[j]->v3.x += (*abs[i-1])[0];
        tri[j]->v3.y += (*abs[i-1])[1];
        tri[j]->v3.z += (*abs[i-1])[2];
      }
    }
	  /*if(i != -1){
      cout << "i: " << i << endl;
      t1->print();
      t2->print();
      t3->print();
      t4->print();
      t5->print();
      t6->print();
    }*/
    for(int j = 0; j < 6; j++){
      to_return.push_back(tri[j]);
    }
    /*to_return.push_back(t1);
	  to_return.push_back(t2);
	  to_return.push_back(t3);
	  to_return.push_back(t4);
	  to_return.push_back(t5);
	  to_return.push_back(t6);*/
  }
  /*for(int i = 0; i < 6*n; i++){
    cout << "i: " << i << endl;
    to_return[i]->print();
  }*/
  numTriangles = 6*n;
  triangles = to_return;
  return to_return;
}
