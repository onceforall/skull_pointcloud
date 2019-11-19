#pragma once 
#include <iostream>



struct Point
{
    double x;
    double y;
    double z;
};
struct Coefficient
{
    double a;
    double b;
    double c;
    double d;
};

class Mirror
{
public:
    Mirror(){};
    ~Mirror(){};
    Coefficient get_panal(Point p1,Point p2,Point p3);
    double dis_pt2panel(Point pt,Coefficient c);   
    Point get_mirrorpt(Point pt,Coefficient C);
};