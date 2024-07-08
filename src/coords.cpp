#include "../include/coords.h"
#include <stdio.h>


Vector3D operator + (const Vector3D& a, const Vector3D& b)
{
    return Vector3D{a.x + b.x, a.y + b.y, a.z + b.z};
}

Vector3D operator * (double l, const Vector3D& vec3D)
{
    return Vector3D{ l*vec3D.x, l*vec3D.y, l*vec3D.z};
}

Vector3D operator * (const Vector3D& vec3D, double l)
{
    return Vector3D{ l*vec3D.x, l*vec3D.y, l*vec3D.z};
}

Vector3D operator - (const Vector3D& b)
{
    return (-1)*b;
}

Vector3D operator - (const Vector3D& a, const Vector3D& b)
{
    return a + (-b);
}

bool operator == (const Vector3D& a, const Vector3D& b)
{
    return (a.x == b.x) && (a.y == b.y) && (a.z == b.z);
}


double operator , (const Vector3D& a, const Vector3D& b)  // Dot product
{
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

Vector3D operator * (const Vector3D& a, const Vector3D& b) // cross product
{
    return {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x};
}


double Vector3D::abs() const 
{
    return sqrt((*this, *this));
}

double abs(const Vector3D& a)
{
    return a.abs();
}


double distance (const Vector3D& a, const Vector3D& b)
{
    return abs(a-b);
}

double distance (const CommandWaypoint& a, const CommandWaypoint& b)
{
    return distance(Vector3D(a), Vector3D(b));
}


bool operator == (const Line& l1, const Line& l2)
{
    return (l1.a * l2.a == Vector3D(0, 0, 0)) && ((l1.r0 - l2.r0) * l1.a == Vector3D(0, 0, 0));
}


double distance (const Vector3D& M, const Line& l)
{
    return abs((M - l.r0) * l.a) / abs(l.a);
}

double distance (const CommandWaypoint& M, const Line& l)
{
    return distance (Vector3D(M), l);
}
