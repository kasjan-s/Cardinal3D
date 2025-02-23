
#include "../rays/shapes.h"
#include "debug.h"

namespace PT {

const char* Shape_Type_Names[(int)Shape_Type::count] = {"None", "Sphere"};

BBox Sphere::bbox() const {

    BBox box;
    box.enclose(Vec3(-radius));
    box.enclose(Vec3(radius));
    return box;
}

Trace Sphere::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!

    Trace ret;
    ret.origin = ray.point;
    ret.hit = false;       // was there an intersection?
    // ret.distance = 0.0f;   // at what distance did the intersection occur?
    // ret.position = Vec3{}; // where was the intersection?
    // ret.normal = Vec3{};   // what was the surface normal at the intersection?

    if (!bbox().hit(ray, ray.dist_bounds)){
        return ret;
    }

    float a = ray.dir.norm_squared();
    float b = 2 * dot(ray.point, ray.dir);
    float c = ray.point.norm_squared() - radius;

    float discriminant = b * b - 4 * a * c;
    if (discriminant < 0) {
        return ret;
    } else if (discriminant == 0) {
        float t = (-b) / (2*a);
        if (t < ray.dist_bounds[0] || t > ray.dist_bounds[1]) {
            return ret;
        } else {
            ret.hit = true;
            ret.distance = t;
            ret.position = ray.at(t);
            ret.normal = ret.position;
            return ret;
        }
    } else {
        float t1 = (-b - sqrt(discriminant)) / (2 * a);
        float t2 = (-b + sqrt(discriminant)) / (2 * a);

        if (t1 >= ray.dist_bounds[0] && t1 <= ray.dist_bounds[1]) {
            ret.hit = true;
            ret.distance = t1;
            ret.position = ray.at(t1);
            ret.normal = ret.position;
        } else if (t2 >= ray.dist_bounds[0] && t1 <= ray.dist_bounds[1]) {
            ret.hit = true;
            ret.distance = t2;
            ret.position = ray.at(t2);
            ret.normal = ret.position;
        }

        return ret;
    }
}

} // namespace PT
