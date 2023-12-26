#include "Triangle.hpp"


bool rayTriangleIntersect(const Vector3f& v0, const Vector3f& v1, const Vector3f& v2, const Vector3f& orig,
                          const Vector3f& dir, float& tnear, float& u, float& v)
{
    // TODO: Implement this function that tests whether the triangle
    // that's specified by v0, v1 and v2 intersects with the ray (whose
    // origin is *orig* and direction is *dir*)
    // Also don't forget to update tnear, u and v.
    // auto P0 = v0 - v1;
    // auto P1 = v1 - v2;
    // auto P2 = v2 - v0;
    // auto E1 = P1-P0;
    // auto E2 = P2-P0;
    auto E1 = v1 - v0;
    auto E2 = v2 - v0;
    auto S = orig - v0;
    auto S1 = crossProduct(dir,E2);
    auto S2 = crossProduct(S,E1);
    auto coef = 1.0f/dotProduct(S1,E1);
    auto result = coef*Vector3f(dotProduct(S2,E2),dotProduct(S1,S),dotProduct(S2,dir));
    auto t = result.x, b1 = result.y, b2 = result.z;

    if(t < 0 || b1 < 0 || b2 < 0 || (b1 + b2) > 1)
        return false;

    // tnear here is only for passing the t value out of the function
    // thus should not judge whether it is smaller than the previous tnear
    tnear = t;
    u = b1;
    v = b2;
    
    return true;
}