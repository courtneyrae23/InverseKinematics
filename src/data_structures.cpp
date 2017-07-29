#include "data_structures.h"
#include <iostream>
using namespace std;

inline float sqr(float x) { return x*x; };

Point Point::operator*(const float c) {
  Point *p = new Point();
  p->x = x*c;
  p->y = y*c;
  p->z = z*c;
  return *p;
}

Point Point::operator/(const float c) {
  Point *p = new Point();
  p->x = x/c;
  p->y = y/c;
  p->z = z/c;
  return *p;
}

Point Point::operator+(const Point& c) {
  Point* p = new Point();
  p->x = x + c.x;
  p->y = y + c.y;
  p->z = z + c.z;
  return *p;
}

Point Point::operator-(const Point& c) {
  Point* p = new Point();
  p->x = x - c.x;
  p->y = y - c.y;
  p->z = z - c.z;
  return *p;
}

void Point::print() {
  cout << "Point: [" << x << "," << y << "," << z  << "]" << endl;
}

void UV::print() {
  cout << "UV: [" << u << "," << v << "]" << endl;
}

void Normal::print() {
  cout << "Normal: [" << x << "," << y << "," << z  << "]" << endl;
}

Point* Triangle::vert_midpoint1() {
  return new Point((v1.x + v2.x)/2, (v1.y + v2.y)/2, (v1.z + v2.z)/2);
}

Point* Triangle::vert_midpoint2() {
  return new Point((v2.x + v3.x)/2, (v2.y + v3.y)/2, (v2.z + v3.z)/2);
}

Point* Triangle::vert_midpoint3() {
  return new Point((v1.x + v3.x)/2, (v1.y + v3.y)/2, (v1.z + v3.z)/2);
}


void Triangle::print() {
  cout << "Triangle Vertices:" << endl;
  v1.print();
  v2.print();
  v3.print(); 
  cout << "Triangle Normals:" << endl;
  n1->print();
  n2->print();
  n3->print();
}
