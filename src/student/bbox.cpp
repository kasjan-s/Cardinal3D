
#include "../lib/mathlib.h"
#include "debug.h"

namespace {
bool intersects(const Vec2& a, const Vec2& b) {
    return !(a[0] > b[1] || b[0] > a[1]);
}

Vec2 intersection(const Vec2& a, const Vec2& b) {
    return Vec2(std::max(a[0], b[0]), std::min(a[1], b[1]));
}
}

bool BBox::hit(const Ray& ray, Vec2& times) const {
    // TODO (PathTracer):
    // Implement ray - bounding box intersection test
    // If the ray intersected the bounding box within the range given by
    // [times.x,times.y], update times with the new intersection times.

    if (empty())
        return false;

    float x0 = times[0], x1 = times[1];
    if (ray.dir.x != 0.0f) {
        x0 = (min.x - ray.point.x) / ray.dir.x;
        x1 = (max.x - ray.point.x) / ray.dir.x;
    } else if (ray.point.x < min.x || ray.point.x > max.x) {
        return false;
    }

    float y0 = times[0], y1 = times[1];
    if (ray.dir.y != 0.0f) {
        y0 = (min.y - ray.point.y) / ray.dir.y;
        y1 = (max.y - ray.point.y) / ray.dir.y;
    } else if (ray.point.y < min.y || ray.point.y > max.y) {
        return false;
    }

    float z0 = times[0], z1 = times[1];
    if (ray.dir.z != 0.0f) {
        z0 = (min.z - ray.point.z) / ray.dir.z;
        z1 = (max.z - ray.point.z) / ray.dir.z;
    } else if (ray.point.z < min.z || ray.point.z > max.z) {
        return false;
    }

    Vec2 x_range(std::min(x0, x1), std::max(x0, x1));
    Vec2 y_range(std::min(y0, y1), std::max(y0, y1));
    Vec2 z_range(std::min(z0, z1), std::max(z0, z1));

    if (!intersects(x_range, y_range) || !intersects(y_range, z_range) || !intersects(x_range, z_range)) {
        return false;
    }

    Vec2 in_box = intersection(x_range, y_range);
    in_box = intersection(in_box, z_range);

    if (!intersects(in_box, ray.dist_bounds))
        return false;

    in_box = intersection(in_box, ray.dist_bounds);

    if (!intersects(in_box, times))
        return false;

    times = intersection(in_box, times);


    return true;
}
