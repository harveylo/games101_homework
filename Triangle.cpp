#include "Triangle.hpp"


// bool rayTriangleIntersect(const Vector3f& v0, const Vector3f& v1, const Vector3f& v2, const Vector3f& orig,
//                           const Vector3f& dir, float& tnear, float& u, float& v)
// {
//     Vector3f edge1 = v1 - v0;
//     Vector3f edge2 = v2 - v0;
//     Vector3f pvec = crossProduct(dir, edge2);
//     float det = dotProduct(edge1, pvec);
//     if (det == 0 || det < 0)
//         return false;

//     Vector3f tvec = orig - v0;
//     u = dotProduct(tvec, pvec);
//     if (u < 0 || u > det)
//         return false;

//     Vector3f qvec = crossProduct(tvec, edge1);
//     v = dotProduct(dir, qvec);
//     if (v < 0 || u + v > det)
//         return false;

//     float invDet = 1 / det;

//     tnear = dotProduct(edge2, qvec) * invDet;
//     u *= invDet;
//     v *= invDet;

//     return true;
// }