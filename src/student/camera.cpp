
#include "../util/camera.h"
#include "../rays/samplers.h"
#include "debug.h"

Ray Camera::generate_ray(Vec2 screen_coord) const {
    float theta = (vert_fov / 180.0f) * PI_F;
    float sensor_height = 2 * std::tan(theta / 2.0f); 
    float sensor_width = aspect_ratio * sensor_height;

    Vec3 bottom_left(-sensor_width / 2.0f, -sensor_height / 2.0f, -1.0f);
    Vec3 sensor_offset((screen_coord.x * sensor_width), 
                       (screen_coord.y * sensor_height), 
                       0.0f);
    Vec3 sensor_coords = bottom_left + sensor_offset;

    Vec3 world_origin = iview * Vec3(0.0f, 0.0f, 0.0f);
    Vec3 world_sensor = iview * sensor_coords;

    return Ray(world_origin, world_sensor - world_origin);
}
