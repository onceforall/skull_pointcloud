#include "mirror.hpp"
#include <algorithm>
#include <cmath>

Coefficient Mirror::get_panal(Point p1,Point p2,Point p3)
{
    Coefficient C;
    C.a=((p2.y-p1.y)*(p3.z-p1.z)-(p2.z-p1.z)*(p3.y-p1.y));
    C.b=((p2.z-p1.z)*(p3.x-p1.x)-(p2.x-p1.x)*(p3.z-p1.z));
    C.c=((p2.x-p1.x)*(p3.y-p1.y)-(p2.y-p1.y)*(p3.x-p1.x));
    C.d=(0-(C.a*p1.x+C.b*p1.y+C.c*p1.z));
    return C;
}

double Mirror::dis_pt2panel(Point pt,Coefficient C)
{
    return abs(C.a*pt.x+C.b*pt.y+C.c*pt.z+C.d)/sqrt(C.a*C.a+C.b*C.b+C.c*C.c);
}

Point Mirror::get_mirrorpt(Point pt,Coefficient C)
{
    Point mirror_pt;
    if(C.a!=0 && C.b!=0 && C.c!=0)
    {
        mirror_pt.x=-1*((pow(C.a,2)-pow(C.b,2)-pow(C.c,2))/C.a+2*C.b*pt.y+2*C.c*pt.z+2*C.d)/(pow(C.a,2)+pow(C.b,2)+pow(C.c,2));
        mirror_pt.y=pt.y-C.b/C.a*(pt.x-mirror_pt.x);
        mirror_pt.z=pt.z-C.c/C.a*(pt.x-mirror_pt.x);
    }
        

    return mirror_pt;
}