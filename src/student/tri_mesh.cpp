
#include "../rays/tri_mesh.h"
#include "debug.h"

namespace PT {

BBox Triangle::bbox() const {

    // TODO (PathTracer): Task 2
    // compute the bounding box of the triangle

    // Beware of flat/zero-volume boxes! You may need to
    // account for that here, or later on in BBox::intersect
    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];
    float x_min = std::min(std::min(v_0.position.x, v_1.position.x), v_2.position.x);
    float y_min = std::min(std::min(v_0.position.y, v_1.position.y), v_2.position.y);
    float z_min = std::min(std::min(v_0.position.z, v_1.position.z), v_2.position.z);
    float x_max = std::max(std::min(v_0.position.x, v_1.position.x), v_2.position.x);
    float y_max = std::max(std::min(v_0.position.y, v_1.position.y), v_2.position.y);
    float z_max = std::max(std::min(v_0.position.z, v_1.position.z), v_2.position.z);

    return BBox(Vec3(x_min, y_min, z_min), Vec3(x_max, y_max, z_max));
}

Trace Triangle::hit(const Ray& ray) const {

    // Vertices of triangle - has postion and surface normal
    // See rays/tri_mesh.h for a description of this struct
    
    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];
    
    // TODO (PathTracer): Task 2
    // Intersect this ray with a triangle defined by the above three points.
    // Intersection should yield a ray t-value, and a hit point (u,v) on the surface of the triangle

    // You'll need to fill in a "Trace" struct describing information about the hit (or lack of hit)
    
    Vec3 col1 = v_1.position - v_0.position;
    Vec3 col2 = v_2.position - v_0.position;
    Vec3 col3 = -ray.dir;
    Mat4 M(Vec4(col1, 0.0f), Vec4(col2, 0.0f), Vec4(col3, 0.0f), Vec4(0.0f, 0.0f, 0.0f, 1.0f));
    if (M.det() == 0) {
        Trace ret;
        ret.origin = ray.point;
        ret.hit = false;
        return ret;
    }

    Vec3 x = ray.point - v_0.position;
    Vec3 uvt = M.inverse() * x;
    float u = uvt.x;
    float v = uvt.y;
    Trace ret;
    ret.origin = ray.point;
    ret.hit = (u >= 0 && v >= 0 && u + v <= 1.0f);
    if (ret.hit) {
        ret.distance = uvt.z;
        ret.position = v_0.position + u * (v_1.position - v_0.position) + v * (v_2.position - v_0.position); 
        ret.normal = u * v_1.normal + v * v_2.normal + (1 - u - v) * v_0.normal;
    }

    return ret;
}

Triangle::Triangle(Tri_Mesh_Vert* verts, unsigned int v0, unsigned int v1, unsigned int v2)
    : vertex_list(verts), v0(v0), v1(v1), v2(v2) {
}

void Tri_Mesh::build(const GL::Mesh& mesh) {

    verts.clear();
    triangles.clear();

    for(const auto& v : mesh.verts()) {
        verts.push_back({v.pos, v.norm});
    }

    const auto& idxs = mesh.indices();

    std::vector<Triangle> tris;
    for(size_t i = 0; i < idxs.size(); i += 3) {
        tris.push_back(Triangle(verts.data(), idxs[i], idxs[i + 1], idxs[i + 2]));
    }

    triangles.build(std::move(tris), 4);
}

Tri_Mesh::Tri_Mesh(const GL::Mesh& mesh) {
    build(mesh);
}

Tri_Mesh Tri_Mesh::copy() const {
    Tri_Mesh ret;
    ret.verts = verts;
    ret.triangles = triangles.copy();
    return ret;
}

BBox Tri_Mesh::bbox() const {
    return triangles.bbox();
}

Trace Tri_Mesh::hit(const Ray& ray) const {
    Trace t = triangles.hit(ray);
    return t;
}

size_t Tri_Mesh::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                           const Mat4& trans) const {
    return triangles.visualize(lines, active, level, trans);
}

} // namespace PT
