
#include <queue>
#include <set>
#include <unordered_map>

#include "../geometry/halfedge.h"
#include "debug.h"
#include <iostream>

namespace {
Halfedge_Mesh::HalfedgeRef find_previous_halfedge(const Halfedge_Mesh::HalfedgeRef hf) {
    auto prev = hf->next();
    while (prev->next() != hf) {
        prev = prev->next();
    }
    return prev;
}

bool is_face_safe_to_collapse(const Halfedge_Mesh::FaceRef face, const Halfedge_Mesh::EdgeRef edge) {
    if (face->degree() > 3)
        return true;

    int boundary_edges = 0;
    auto hf = face->halfedge();
    do {
        boundary_edges += hf->edge()->on_boundary() ? 1 : 0;
        hf = hf->next();
    } while (hf != face->halfedge());

    // Mesh is a singular triangle that would become 2-edge line.
    if (boundary_edges == 3) 
        return false;

    // This will also lead to a 2-edge line, with a hanging boundary vertex.
    // However if edge IS on the boundary, then collapsing is safe to do, we'll 
    // just end up with a new boundary edge.
    if (boundary_edges == 2 && !edge->on_boundary()) 
        return false;

    return true;
}
}

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it does not want to perform the operation for
    whatever reason (e.g. you don't want to allow the user to erase the last vertex).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementaiton, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/

/*
    This method should replace the given vertex and all its neighboring
    edges and faces with a single face, returning the new face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_vertex(Halfedge_Mesh::VertexRef v) {
    std::vector<HalfedgeRef> outgoing_halfedges;

    auto hf = v->halfedge();
    do {
        outgoing_halfedges.push_back(hf);
        hf = hf->twin()->next();
    } while (hf != v->halfedge());

    auto face = new_face();
    face->halfedge() = outgoing_halfedges.front()->next();
    
    std::set<FaceRef> faces_to_delete;

    // Make sure "outside" halfedges (i.e. halfedges not adjacent to v)
    // point to the new face, and that outside vertexes point to valid halfedges that will stay.
    for (size_t i = 0; i < outgoing_halfedges.size(); ++i) {
        faces_to_delete.insert(outgoing_halfedges[i]->face());

        auto outside_hf = outgoing_halfedges[i]->next();
        auto outside_vertex = outgoing_halfedges[i]->twin()->vertex();

        // After erasing this vertex a neighbor would end up with a single degree, which 
        // is incorrect manifold mesh geometry.
        if (outside_vertex->degree() == 2 && !outside_vertex->on_boundary()) {
            erase(face);
            return std::nullopt;
        }
        outside_vertex->halfedge() = outside_hf;

        do {
            outside_hf->face() = face;
            outside_hf = outside_hf->next();
        } while (outside_hf != outgoing_halfedges[i]->next());
    }

    // Connect outside halfedges, so we can delete inside ones.
    for (size_t i = 0; i < outgoing_halfedges.size(); ++i) {
        auto outside_hf = outgoing_halfedges[i]->next();
        auto prev = find_previous_halfedge(outgoing_halfedges[i]->twin());
        prev->next() = outside_hf;
    }

    for (auto halfedge : outgoing_halfedges) {
        erase(halfedge->edge());
        erase(halfedge->twin());
        erase(halfedge);
    }

    for (auto face : faces_to_delete) {
        erase(face);
    }

    erase(v);

    return face;
}

/*
    This method should erase the given edge and return an iterator to the
    merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_edge(Halfedge_Mesh::EdgeRef e) {
    if (e->on_boundary())
        return std::nullopt;

    auto hf = e->halfedge();
    auto hf_twin = hf->twin();
    
    auto v0 = hf->vertex();
    auto v1 = hf->vertex();
    // If any of endpoints has a degree 2, erasing this edge would lead to degenerate case
    // with a leaf point.
    if (v0->degree() + v0->on_boundary() == 2 || v1->degree() + v1->on_boundary() == 2) 
        return std::nullopt;

    auto hf_prev = find_previous_halfedge(hf);
    auto hf_twin_prev = find_previous_halfedge(hf_twin);

    hf_prev->next() = hf_twin->next();
    hf_twin_prev->next() = hf->next();

    auto iter = hf_prev;
    do {
        iter->face() = hf->face();
        iter = iter->next();
    } while (iter != hf_prev);

    hf->vertex()->halfedge() = hf_twin->next();
    hf_twin->vertex()->halfedge() = hf->next();

    hf->face()->halfedge() = hf->next();

    erase(hf_twin->face());
    erase(e);
    erase(hf);
    erase(hf_twin);

    return hf->face();
}

void Halfedge_Mesh::erase_halfedge_face(Halfedge_Mesh::HalfedgeRef hf) {
    auto hf_next = hf->next();
    auto hf_prev = hf_next->next();
    // We will delete hf_next, hf_prev, this face,
    // and hf_next's edge. hf_prev's edge will stay.

    // hf_prev's edge stays, but hf_prev doesn't, so make sure
    // edge points to hf_prev's twin.
    hf_prev->edge()->halfedge() = hf_prev->twin();

    // hf_next's and hf_prev's twins stay. Connect them,
    // and make them point to hf_prev's edge.
    hf_next->twin()->twin() = hf_prev->twin();
    hf_prev->twin()->twin() = hf_next->twin();
    hf_next->twin()->edge() = hf_prev->edge();

    // Face is safe to be erased. It's only pointed to by
    // hf, hf_next and hf_prev, who will all get deleted.
    erase(hf->face());

    // Edge is safe to be erased. It's only pointd to by hf_next,
    // which will get deleted.
    erase(hf_next->edge());

    // hf_prev's vertex doen't get deleted. Make sure it doesn't point to hf_prev.
    hf_prev->vertex()->halfedge() = hf_next->twin();

    // hf_next's vertex will get deleted.
    // hf_next's twin doesn't point to it anymore.
    // hf_next's previous halfedge will get delete (hf).
    // hf_next's edge already erased.
    // hf_next's face already erased.
    erase(hf_next);

    // hf_prev's vertex already reassigned.
    // hf_prev's face already erased.
    // hf_prev's twin already reassigned.
    // hf_prev's edge already erased.
    // hf_prev's previous halfedge will get erased (hf_next).
    erase(hf_prev);

    erase(hf);
}

/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef e) {
    auto hf = e->halfedge();
    auto hf_twin = hf->twin();
    auto v0 = hf->vertex();
    auto v1 = hf->twin()->vertex();

    auto face = hf->face();
    auto face_twin = hf_twin->face();

    if (!is_face_safe_to_collapse(face, e) || !is_face_safe_to_collapse(face_twin, e)) {
        return std::nullopt;
    }

    // Grab all half-edges from v0 or v1.
    std::set<HalfedgeRef> halfedges;
    auto hf_iter = v0->halfedge()->twin();
    do {
        if (hf_iter->vertex() != v1) {
            halfedges.insert(hf_iter->twin());
        }
        hf_iter = hf_iter->next()->twin();
    } while (hf_iter != v0->halfedge()->twin());

    hf_iter = v1->halfedge()->twin();
    do {
        if (hf_iter->vertex() != v0) {
            halfedges.insert(hf_iter->twin());
        }
        hf_iter = hf_iter->next()->twin();
    } while (hf_iter != v1->halfedge()->twin());

    // Check if any of faces will collapse as well
    if (face->degree() == 3) {
        halfedges.erase(hf->next());
        erase_halfedge_face(hf);
    } else {
        // If it won't, we just need to connect prev and next halfedges.
        auto hf_prev = find_previous_halfedge(hf);
        hf_prev->next() = hf->next();
        hf->face()->halfedge() = hf_prev;
        erase(hf);
    }

    if (face_twin->degree() == 3) {
        halfedges.erase(hf_twin->next()->next());
        erase_halfedge_face(hf_twin);
    } else {
        auto hf_twin_prev = find_previous_halfedge(hf_twin);
        hf_twin_prev->next() = hf_twin->next();
        hf_twin->face()->halfedge() = hf_twin_prev;
        erase(hf_twin);
    }

    // Create new vertex, and hook gathered halfedges to it.
    VertexRef vertex = new_vertex();
    vertex->pos = e->center();
    for (auto halfedge : halfedges) {
        halfedge->vertex() = vertex;
        vertex->halfedge() = halfedge;
    }

    erase(v0);
    erase(v1);
    erase(e);

    return vertex;
}

/*
    This method should collapse the given face and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(Halfedge_Mesh::FaceRef f) {

    (void)f;
    return std::nullopt;
}

/*
    This method should flip the given edge and return an iterator to the
    flipped edge.
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(Halfedge_Mesh::EdgeRef e) {
    auto hf = e->halfedge();
    auto hf_twin = hf->twin();

    if (e->on_boundary()) { 
        return std::nullopt;
    }

    auto v0 = hf->vertex();
    auto v1 = hf_twin->vertex();
    auto w0 = hf_twin->next()->twin()->vertex();
    auto w1 = hf->next()->twin()->vertex();

    // Current edge spans vertexes v0 and v1.
    // We want to flip it so it connects to w0 and w1.

    // First, check that w0 and w1 aren't already connect by some other edge.
    // If they are, we'd end up with degenerate case where two edges overlap.
    auto existing_hf = w0->halfedge();
    do {
        if (existing_hf->twin()->vertex() == w1) 
            return std::nullopt;

        existing_hf = existing_hf->twin()->next();
    } while (existing_hf != w0->halfedge());

    auto new_v0_halfedge = hf_twin->next();
    auto new_v1_halfedge = hf->next();

    auto hf_prev = find_previous_halfedge(hf);
    auto twin_prev = find_previous_halfedge(hf_twin);
    hf_prev->next() = hf_twin->next();
    twin_prev->next() = hf->next();

    auto new_hf_next = hf->next()->next();
    auto new_hf_prev = hf_twin->next();

    auto new_twin_next = hf_twin->next()->next();
    auto new_twin_prev = hf->next();

    // Only set new values after we have figured out all the data, so we don't accidentally overwrite.
    hf->vertex() = w0;
    hf->next() = new_hf_next;
    new_hf_prev->next() = hf;
    hf_twin->vertex() = w1;
    new_hf_prev->face() = hf->face();
    new_twin_prev->face() = hf_twin->face();
    hf_twin->next() = new_twin_next;
    new_twin_prev->next() = hf_twin;
    v0->halfedge() = new_v0_halfedge;
    v1->halfedge() = new_v1_halfedge;
    hf->face()->halfedge() = hf;
    hf_twin->face()->halfedge() = hf_twin;

    return e;
}

/*
    This method should split the given edge and return an iterator to the
    newly inserted vertex. The halfedge of this vertex should point along
    the edge that was split, rather than the new edges.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(Halfedge_Mesh::EdgeRef e) {    
    auto hf = e->halfedge();
    auto hf_next = hf->next();
    auto hf_prev = find_previous_halfedge(hf);

    auto hf_twin = hf->twin();
    auto hf_twin_next = hf_twin->next();
    auto hf_twin_prev = find_previous_halfedge(hf_twin);

    auto v0 = hf->vertex();
    auto v1 = hf_twin->vertex();

    bool east_is_boundary = hf_twin->face()->is_boundary();
    bool west_is_boundary = hf->face()->is_boundary();

    if (east_is_boundary && west_is_boundary)
        return std::nullopt;

    // This operation should only work on triangle meshes.
    if ((hf->face()->degree() != 3 && !west_is_boundary) || (hf_twin->face()->degree() != 3 && !east_is_boundary))
        return std::nullopt;

    // We will create new vertex, four new edges, four new faces, and connect them as appropriate.
    // Then we will delete the old edge e, its halfedges, and its faces.
    // We will skip creating some elements if we're working at the boundary.
    VertexRef vertex = new_vertex();
    vertex->pos = e->center();

    // We will assume frame where v0 is at the bottom of the screen, and v1 at the top.
    // This way we can refer to the new edges as north/south/west/east to describe them easier.
    // hf is on the "west" side of edge, pointing north. hf_twin is opposite.
    EdgeRef edge_north = new_edge();
    EdgeRef edge_south = new_edge();
    HalfedgeRef hf_n = new_halfedge();
    HalfedgeRef hf_n_twin = new_halfedge();
    HalfedgeRef hf_s = new_halfedge();
    HalfedgeRef hf_s_twin = new_halfedge();
    
    // Assign current faces to the new halfedges, in case we're on a boundary.
    // If not, those will get overwritten to the right values later.
    hf_n->face() = hf->face();
    hf_s_twin->face() = hf->face();
    hf_s->face() = hf_twin->face();
    hf_n_twin->face() = hf_twin->face();
    hf->face()->halfedge() = hf_n;
    hf_twin->face()->halfedge() = hf_s;

    if (!west_is_boundary) {
        EdgeRef edge_west = new_edge();
        HalfedgeRef hf_w = new_halfedge();
        HalfedgeRef hf_w_twin = new_halfedge();
        FaceRef face_nw = new_face();
        FaceRef face_sw = new_face();

        hf_n->set_neighbors(hf->next(), hf_n_twin, vertex, edge_north, face_nw);
        hf_s_twin->set_neighbors(hf_w, hf_s, v0, edge_south, face_sw);
        hf_w->set_neighbors(hf_prev, hf_w_twin, vertex, edge_west, face_sw);
        hf_w_twin->set_neighbors(hf_n, hf_w, hf_prev->vertex(), edge_west, face_nw);

        edge_west->halfedge() = hf_w;
        face_nw->halfedge() = hf_n;
        face_sw->halfedge() = hf_w;

        hf_next->next() = hf_w_twin;

        hf_next->face() = face_nw;
        hf_prev->face() = face_sw;

        hf_n->face() = face_nw;
        hf_s_twin->face() = face_sw;
    } else {
        hf_n->set_neighbors(hf->next(), hf_n_twin, vertex, edge_north, hf->face());
        hf_s_twin->set_neighbors(hf_n, hf_s, v0, edge_south, hf->face());
    }

    if (!east_is_boundary) {
        EdgeRef edge_east = new_edge();
        HalfedgeRef hf_e = new_halfedge();
        HalfedgeRef hf_e_twin = new_halfedge();
        FaceRef face_ne = new_face();
        FaceRef face_se = new_face();

        hf_n_twin->set_neighbors(hf_e, hf_n, v1, edge_north, face_ne);
        hf_s->set_neighbors(hf_twin_next, hf_s_twin, vertex, edge_south, face_se);
        hf_e->set_neighbors(hf_twin_prev, hf_e_twin, vertex, edge_east, face_ne);
        hf_e_twin->set_neighbors(hf_s, hf_e, hf_twin_prev->vertex(), edge_east, face_se);

        edge_east->halfedge() = hf_e;
        face_ne->halfedge() = hf_e;
        face_se->halfedge() = hf_s;

        hf_twin_next->next() = hf_e_twin;

        hf_twin_next->face() = face_se;
        hf_twin_prev->face() = face_ne;

        hf_s->face() = face_se;
        hf_n_twin->face() = face_ne;
    } else {
        hf_n_twin->set_neighbors(hf_s, hf_n, v1, edge_north, hf_twin->face());
        hf_s->set_neighbors(hf_twin_next, hf_s_twin, vertex, edge_south, hf_twin->face());
    }

    if (west_is_boundary) {
        vertex->halfedge() = hf_s;
        edge_north->halfedge() = hf_n_twin;
        edge_south->halfedge() = hf_s;

        v0->halfedge() = hf_twin_next;
        v1->halfedge() = hf_n_twin;
    } else {
        vertex->halfedge() = hf_n;
        edge_north->halfedge() = hf_n;
        edge_south->halfedge() = hf_s_twin;
        
        v0->halfedge() = hf_s_twin;
        v1->halfedge() = hf_next;
    }

    hf_prev->next() = hf_s_twin;
    hf_twin_prev->next() = hf_n_twin;
    
    // We are ready to delete old elements now.
    if (!west_is_boundary)
        erase(hf->face());

    if (!east_is_boundary)
        erase(hf_twin->face());
        
    erase(e);
    erase(hf);
    erase(hf_twin);

    return vertex;
}

/* Note on the beveling process:

    Each of the bevel_vertex, bevel_edge, and bevel_face functions do not represent
    a full bevel operation. Instead, they should update the _connectivity_ of
    the mesh, _not_ the positions of newly created vertices. In fact, you should set
    the positions of new vertices to be exactly the same as wherever they "started from."

    When you click on a mesh element while in bevel mode, one of those three functions
    is called. But, because you may then adjust the distance/offset of the newly
    beveled face, we need another method of updating the positions of the new vertices.

    This is where bevel_vertex_positions, bevel_edge_positions, and
    bevel_face_positions come in: these functions are called repeatedly as you
    move your mouse, the position of which determins the normal and tangent offset
    parameters. These functions are also passed an array of the original vertex
    positions: for  bevel_vertex, it has one element, the original vertex position,
    for bevel_edge,  two for the two vertices, and for bevel_face, it has the original
    position of each vertex in halfedge order. You should use these positions, as well
    as the normal and tangent offset fields to assign positions to the new vertices.

    Finally, note that the normal and tangent offsets are not relative values - you
    should compute a particular new position from them, not a delta to apply.
*/

/*
    This method should replace the vertex v with a face, corresponding to
    a bevel operation. It should return the new face.  NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_vertex_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(Halfedge_Mesh::VertexRef v) {
    if (v->on_boundary() || v->degree() < 3) {
        return std::nullopt;
    }

    std::vector<HalfedgeRef> outgoing_halfedges;
    auto hf = v->halfedge();
    do {
        outgoing_halfedges.push_back(hf);
        hf = hf->twin()->next();
    } while (hf != v->halfedge());

    auto degree = outgoing_halfedges.size();

    std::vector<VertexRef> new_vertexes;
    std::vector<EdgeRef> new_edges;
    std::vector<HalfedgeRef> new_halfedges;
    auto face = new_face();
    for (size_t i = 0; i < degree; ++i) {
        new_vertexes.push_back(new_vertex());
        new_vertexes.back()->pos = v->pos;
        new_edges.push_back(new_edge());
        new_halfedges.push_back(new_halfedge());
        new_halfedges.push_back(new_halfedge());
    }

    for (size_t i = 0; i < degree; ++i) {
        size_t next_idx = i + 1;
        if (next_idx >= degree) {
            next_idx = degree - 1;
        }
        size_t prev_idx = i == 0 ? degree - 1 : i - 1;

        auto old_hf = outgoing_halfedges[i];
        auto old_hf_prev = find_previous_halfedge(old_hf);

        auto hf = new_halfedges[2 * i];

        hf->set_neighbors(old_hf, new_halfedges[2 * i + 1], new_vertexes[prev_idx], new_edges[i], old_hf->face());

        hf->twin()->set_neighbors(new_halfedges[2 * prev_idx + 1], hf, new_vertexes[i], new_edges[i], face);

        new_edges[i]->halfedge() = hf;

        old_hf_prev->next() = hf;

        old_hf->vertex() = new_vertexes[i];
        new_vertexes[i]->halfedge() = hf->twin();

        face->halfedge() = hf->twin();
    }

    erase(v);

    return face;
}

/*
    This method should replace the edge e with a face, corresponding to a
    bevel operation. It should return the new face. NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_edge_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(Halfedge_Mesh::EdgeRef e) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)e;
    return std::nullopt;
}

/*
    This method should replace the face f with an additional, inset face
    (and ring of faces around it), corresponding to a bevel operation. It
    should return the new face.  NOTE: This method is responsible for updating
    the *connectivity* of the mesh only---it does not need to update the vertex
    positions. These positions will be updated in
    Halfedge_Mesh::bevel_face_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_face(Halfedge_Mesh::FaceRef f) {
    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."
    std::vector<VertexRef> vertexes;
    std::vector<HalfedgeRef> halfedges;
    auto hf = f->halfedge();
    do {
        vertexes.push_back(hf->vertex());
        halfedges.push_back(hf);
        hf = hf->next();
    } while (hf != f->halfedge());

    std::vector<VertexRef> new_vertexes;
    std::vector<EdgeRef> new_edges;
    std::vector<FaceRef> new_faces;
    std::vector<HalfedgeRef> new_halfedges;
    for (auto v : vertexes) {
        new_vertexes.push_back(new_vertex());
        new_vertexes.back()->pos = v->pos;
        new_edges.push_back(new_edge());
        new_edges.push_back(new_edge());
        new_halfedges.push_back(new_halfedge());
        new_halfedges.push_back(new_halfedge());
        new_halfedges.push_back(new_halfedge());
        new_halfedges.push_back(new_halfedge());
        new_faces.push_back(new_face());
    }

    for (size_t i = 0; i < vertexes.size(); ++i) {
        //       <-- top_hf
        // v[i+1] __________v[i]
        // |    hf1 -->     |
        // |                |
        // | hf4        hf2 | edge_side
        // |                |
        // |    hf3         |
        // |_____edge_bot___| n_v[i]
        // n_v[i+1] top_hf_twin

        int prev_idx = i - 1;
        if (prev_idx < 0) {
            prev_idx = vertexes.size() - 1;
        }

        int next_idx = i + 1;
        if (i+1 >= vertexes.size()) {
            next_idx = 0;
        }

        auto face = new_faces[i];
        auto hf1 = new_halfedges[4*i + 0];
        auto hf2 = new_halfedges[4*i + 1];
        auto hf3 = new_halfedges[4*i + 2];
        auto hf4 = new_halfedges[4*i + 3];

        auto hf4_twin = new_halfedges[4*next_idx +1];
        auto hf2_twin = new_halfedges[4*prev_idx + 3];

        auto edge_side = new_edges[2*i + 0];
        auto edge_bot = new_edges[2*i + 1];
        auto edge_side_next = new_edges[2*next_idx + 0];

        auto top_hf = halfedges[i];
        auto top_hf_twin = top_hf->twin();

        face->halfedge() = hf1;

        hf1->set_neighbors(hf2, top_hf, vertexes[next_idx], top_hf->edge(), face);
        hf2->set_neighbors(hf3, hf2_twin, vertexes[i], edge_side, face);
        hf3->set_neighbors(hf4, top_hf_twin, new_vertexes[i], edge_bot, face);
        hf4->set_neighbors(hf1, hf4_twin, new_vertexes[next_idx], edge_side_next, face);

        top_hf->set_neighbors(top_hf->next(), hf1, vertexes[i], top_hf->edge(), f);
        top_hf_twin->set_neighbors(top_hf_twin->next(), hf3, new_vertexes[next_idx], edge_bot, top_hf_twin->face());
        
        top_hf->edge()->halfedge() = top_hf;

        edge_bot->halfedge() = hf3;
        edge_side->halfedge() = hf2;
        vertexes[i]->halfedge() = top_hf;
        new_vertexes[i]->halfedge() = hf3;
    
    }

    // Make sure all halfedges of new_vertexes point back to them.
    for (auto vertex : new_vertexes) {
        auto new_vert_hf = vertex->halfedge();
        do {
            new_vert_hf->vertex() = vertex;
            new_vert_hf = new_vert_hf->twin()->next();
        } while (new_vert_hf != vertex->halfedge());
    }

    return f;
}

/*
    Compute new vertex positions for the vertices of the beveled vertex.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the original vertex position and its associated outgoing edge
    to compute a new vertex position along the outgoing edge.
*/
void Halfedge_Mesh::bevel_vertex_positions(const std::vector<Vec3>& start_positions,
                                           Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());


    for (size_t i = 0; i < new_halfedges.size(); ++i) {
        auto hf = new_halfedges[i];
        auto vertex = hf->vertex();
        auto outside_vertex = hf->twin()->next()->twin()->vertex();

        auto direction = start_positions[i] - outside_vertex->pos;
        direction = direction.unit();
        vertex->pos += direction * tangent_offset;

        auto distance_from_start = (start_positions[i] - vertex->pos).norm();
        auto distance_from_end = (outside_vertex->pos - vertex->pos).norm();
        auto original_distance = (start_positions[i] - outside_vertex->pos).norm();

        // Clamp possible positions to stay between original vertexes.
        if (distance_from_start > original_distance) {
            if (distance_from_end > distance_from_start) {
                vertex->pos = start_positions[i];
            } else {
                vertex->pos = outside_vertex->pos;
            }
        } else if (distance_from_end > original_distance) {
            if (distance_from_end < distance_from_start) {
                vertex->pos = outside_vertex->pos;
            } else {
                vertex->pos = start_positions[i];
            }
        }
    }
}

/*
    Compute new vertex positions for the vertices of the beveled edge.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the orig array) to compute an offset vertex position.

    Note that there is a 1-to-1 correspondence between halfedges in
    newHalfedges and vertex positions
    in orig.  So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vector3D pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_edge_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled face.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the start_positions array) to compute an offset vertex
    position.

    Note that there is a 1-to-1 correspondence between halfedges in
    new_halfedges and vertex positions
    in orig. So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vec3 pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_face_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset,
                                         float normal_offset) {

    if(flip_orientation) normal_offset = -normal_offset;
    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    auto face_normal = face->normal();
    auto face_centerpoint = face->center();

    for (auto hf : new_halfedges) {
        auto vertex = hf->vertex();
        auto direction_from_center = vertex->pos - face_centerpoint;
        direction_from_center.normalize();
        vertex->pos += face_normal * normal_offset + direction_from_center * tangent_offset;
    }
}

/*
    Splits all non-triangular faces into triangles.
*/
void Halfedge_Mesh::triangulate() {

    // For each face...
    // Store faces in a separate container to avoid issues with modifying faces in place while iterating
    std::vector<FaceRef> original_faces;
    for (FaceRef fc = faces_begin(); fc != faces_end(); ++fc) {
        if (fc->degree() > 3 && !fc->is_boundary()) {
            original_faces.push_back(fc);
        }
    }

    // Process each face for triangulation
    for (FaceRef fc : original_faces) {
        HalfedgeRef h_0 = fc->halfedge();
        VertexRef v_0 = h_0->vertex();

        HalfedgeRef h_p1 = h_0->next();
        HalfedgeRef h_p2 = h_p1->next();

        while (fc->degree() > 3) {
            VertexRef p2 = h_p2->vertex();

            // Create new edge, halfedges, and face
            EdgeRef new_edge_ = new_edge();
            FaceRef new_face_ = new_face();
            HalfedgeRef new_h = new_halfedge();
            HalfedgeRef new_h_twin = new_halfedge();

            // Connect new halfedges and edges
            new_h->set_neighbors(h_p2, new_h_twin, v_0, new_edge_, fc);
            new_h_twin->set_neighbors(h_0, new_h, p2, new_edge_, new_face_);

            // Update remaining old halfedges
            h_0->face() = new_face_;
            h_p1->face() = new_face_;

            find_previous_halfedge(h_0)->next() = new_h;
            h_p1->next() = new_h_twin;

            // Update face and edge relationships
            new_face_->halfedge() = new_h_twin;
            new_edge_->halfedge() = new_h;
            fc->halfedge() = new_h;

            // Update next pointers for iteration
            h_0 = new_h;
            h_p1 = h_0->next();
            h_p2 = h_p1->next();
        }
    }

}

/* Note on the quad subdivision process:

        Unlike the local mesh operations (like bevel or edge flip), we will perform
        subdivision by splitting *all* faces into quads "simultaneously."  Rather
        than operating directly on the halfedge data structure (which as you've
        seen is quite difficult to maintain!) we are going to do something a bit nicer:
           1. Create a raw list of vertex positions and faces (rather than a full-
              blown halfedge mesh).
           2. Build a new halfedge mesh from these lists, replacing the old one.
        Sometimes rebuilding a data structure from scratch is simpler (and even
        more efficient) than incrementally modifying the existing one.  These steps are
        detailed below.

  Step I: Compute the vertex positions for the subdivided mesh.
        Here we're going to do something a little bit strange: since we will
        have one vertex in the subdivided mesh for each vertex, edge, and face in
        the original mesh, we can nicely store the new vertex *positions* as
        attributes on vertices, edges, and faces of the original mesh. These positions
        can then be conveniently copied into the new, subdivided mesh.
        This is what you will implement in linear_subdivide_positions() and
        catmullclark_subdivide_positions().

  Steps II-IV are provided (see Halfedge_Mesh::subdivide()), but are still detailed
  here:

  Step II: Assign a unique index (starting at 0) to each vertex, edge, and
        face in the original mesh. These indices will be the indices of the
        vertices in the new (subdivided mesh).  They do not have to be assigned
        in any particular order, so long as no index is shared by more than one
        mesh element, and the total number of indices is equal to V+E+F, i.e.,
        the total number of vertices plus edges plus faces in the original mesh.
        Basically we just need a one-to-one mapping between original mesh elements
        and subdivided mesh vertices.

  Step III: Build a list of quads in the new (subdivided) mesh, as tuples of
        the element indices defined above. In other words, each new quad should be
        of the form (i,j,k,l), where i,j,k and l are four of the indices stored on
        our original mesh elements.  Note that it is essential to get the orientation
        right here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces
        should circulate in the same direction as old faces (think about the right-hand
        rule).

  Step IV: Pass the list of vertices and quads to a routine that clears
        the internal data for this halfedge mesh, and builds new halfedge data from
        scratch, using the two lists.
*/

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    simple linear interpolation, e.g., the edge midpoints and face
    centroids.
*/
void Halfedge_Mesh::linear_subdivide_positions() {
    // For each vertex, assign Vertex::new_pos to
    // its original position, Vertex::pos.
    for (VertexRef v = vertices_begin(); v != vertices_end(); v++){
        v->new_pos = v->pos;
    }

    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.
    for (EdgeRef e = edges_begin(); e != edges_end(); e++) {
        Vec3 v1 = e->halfedge()->vertex()->pos;
        Vec3 v2 = e->halfedge()->twin()->vertex()->pos;
        e->new_pos = (v1 + v2) / 2;
    }

    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!
    for (FaceRef f = faces_begin(); f != faces_end(); f++) {
        // forget about boundary
        if (f->is_boundary() == true || f->degree() == 0) {
            continue;
        }

        Vec3 v_face = Vec3(0, 0, 0);
        HalfedgeRef h = f->halfedge();

        do {
            v_face += h->vertex()->pos;
            h = h->next();
        } while(h != f->halfedge());

        v_face /= f->degree();

        f->new_pos = v_face;
    }
}

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    the Catmull-Clark rules for subdivision.

    Note: this will only be called on meshes without boundary
*/
void Halfedge_Mesh::catmullclark_subdivide_positions() {

    // The implementation for this routine should be
    // a lot like Halfedge_Mesh:linear_subdivide_positions:(),
    // except that the calculation of the positions themsevles is
    // slightly more involved, using the Catmull-Clark subdivision
    // rules. (These rules are outlined in the Developer Manual.)

    // Faces
    for (FaceRef f = faces_begin(); f != faces_end(); f++) {
        // forget about boundary
        if (f->is_boundary() == true || f->degree() == 0) {
            continue;
        }

        Vec3 v_face = Vec3(0, 0, 0);
        HalfedgeRef h = f->halfedge();

        do {
            v_face += h->vertex()->pos;
            h = h->next();
        } while(h != f->halfedge());

        v_face /= f->degree();

        f->new_pos = v_face;
    }

    // Edges
    for (EdgeRef e = edges_begin(); e != edges_end(); e++) {
        Vec3 v1 = e->halfedge()->vertex()->pos;
        Vec3 v_f1 = e->halfedge()->face()->new_pos;
        Vec3 v2 = e->halfedge()->twin()->vertex()->pos;
        Vec3 v_f2 = e->halfedge()->twin()->face()->new_pos;
        e->new_pos = (v1 + v2 + v_f1 + v_f2) / 4;
    }


    // Vertex positions
    for (VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        int n = v->degree();
        if (n == 0) continue;

        // Compute Q (face centroid average)
        Vec3 Q(0, 0, 0);
        Vec3 R(0, 0, 0);  // Edge midpoint average
        HalfedgeRef h = v->halfedge();

        HalfedgeRef start = h;
        do {
            if (!h->face()->is_boundary()) {
                Q += h->face()->new_pos;
            }
            R += h->edge()->new_pos;
            h = h->twin()->next();
        } while (h != start);

        Q /= n;
        R /= n;

        Vec3 S = v->pos;
        v->new_pos = (Q + 2 * R + (n - 3) * S) / n;
    }
}

/*
        This routine should increase the number of triangles in the mesh
        using Loop subdivision. Note: this is will only be called on triangle meshes.
*/
void Halfedge_Mesh::loop_subdivide() {

    // Compute new positions for all the vertices in the input mesh, using
    // the Loop subdivision rule, and store them in Vertex::new_pos.
    // -> At this point, we also want to mark each vertex as being a vertex of the
    //    original mesh. Use Vertex::is_new for this.
    // -> Next, compute the updated vertex positions associated with edges, and
    //    store it in Edge::new_pos.
    // -> Next, we're going to split every edge in the mesh, in any order.  For
    //    future reference, we're also going to store some information about which
    //    subdivided edges come from splitting an edge in the original mesh, and
    //    which edges are new, by setting the flat Edge::is_new. Note that in this
    //    loop, we only want to iterate over edges of the original mesh.
    //    Otherwise, we'll end up splitting edges that we just split (and the
    //    loop will never end!)
    // -> Now flip any new edge that connects an old and new vertex.
    // -> Finally, copy the new vertex positions into final Vertex::pos.

    // Each vertex and edge of the original surface can be associated with a
    // vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to
    // *first* compute the new positions
    // using the connectivity of the original (coarse) mesh; navigating this mesh
    // will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse.  We
    // will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.

    // Compute updated positions for all the vertices in the original mesh, using
    // the Loop subdivision rule.

    // Next, compute the updated vertex positions associated with edges.

    // Next, we're going to split every edge in the mesh, in any order. For
    // future reference, we're also going to store some information about which
    // subdivided edges come from splitting an edge in the original mesh, and
    // which edges are new.
    // In this loop, we only want to iterate over edges of the original
    // mesh---otherwise, we'll end up splitting edges that we just split (and
    // the loop will never end!)

    // Finally, flip any new edge that connects an old and new vertex.

    // Copy the updated vertex positions to the subdivided mesh.
}

/*
    Isotropic remeshing. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if this is not a triangle mesh)
*/
bool Halfedge_Mesh::isotropic_remesh() {

    // Compute the mean edge length.
    // Repeat the four main steps for 5 or 6 iterations
    // -> Split edges much longer than the target length (being careful about
    //    how the loop is written!)
    // -> Collapse edges much shorter than the target length.  Here we need to
    //    be EXTRA careful about advancing the loop, because many edges may have
    //    been destroyed by a collapse (which ones?)
    // -> Now flip each edge if it improves vertex degree
    // -> Finally, apply some tangential smoothing to the vertex positions

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}

/* Helper type for quadric simplification */
struct Edge_Record {
    Edge_Record() {
    }
    Edge_Record(std::unordered_map<Halfedge_Mesh::VertexRef, Mat4>& vertex_quadrics,
                Halfedge_Mesh::EdgeRef e)
        : edge(e) {

        // Compute the combined quadric from the edge endpoints.
        // -> Build the 3x3 linear system whose solution minimizes the quadric error
        //    associated with these two endpoints.
        // -> Use this system to solve for the optimal position, and store it in
        //    Edge_Record::optimal.
        // -> Also store the cost associated with collapsing this edge in
        //    Edge_Record::cost.
    }
    Halfedge_Mesh::EdgeRef edge;
    Vec3 optimal;
    float cost;
};

/* Comparison operator for Edge_Records so std::set will properly order them */
bool operator<(const Edge_Record& r1, const Edge_Record& r2) {
    if(r1.cost != r2.cost) {
        return r1.cost < r2.cost;
    }
    Halfedge_Mesh::EdgeRef e1 = r1.edge;
    Halfedge_Mesh::EdgeRef e2 = r2.edge;
    return &*e1 < &*e2;
}

/** Helper type for quadric simplification
 *
 * A PQueue is a minimum-priority queue that
 * allows elements to be both inserted and removed from the
 * queue.  Together, one can easily change the priority of
 * an item by removing it, and re-inserting the same item
 * but with a different priority.  A priority queue, for
 * those who don't remember or haven't seen it before, is a
 * data structure that always keeps track of the item with
 * the smallest priority or "score," even as new elements
 * are inserted and removed.  Priority queues are often an
 * essential component of greedy algorithms, where one wants
 * to iteratively operate on the current "best" element.
 *
 * PQueue is templated on the type T of the object
 * being queued.  For this reason, T must define a comparison
 * operator of the form
 *
 *    bool operator<( const T& t1, const T& t2 )
 *
 * which returns true if and only if t1 is considered to have a
 * lower priority than t2.
 *
 * Basic use of a PQueue might look
 * something like this:
 *
 *    // initialize an empty queue
 *    PQueue<myItemType> queue;
 *
 *    // add some items (which we assume have been created
 *    // elsewhere, each of which has its priority stored as
 *    // some kind of internal member variable)
 *    queue.insert( item1 );
 *    queue.insert( item2 );
 *    queue.insert( item3 );
 *
 *    // get the highest priority item currently in the queue
 *    myItemType highestPriorityItem = queue.top();
 *
 *    // remove the highest priority item, automatically
 *    // promoting the next-highest priority item to the top
 *    queue.pop();
 *
 *    myItemType nextHighestPriorityItem = queue.top();
 *
 *    // Etc.
 *
 *    // We can also remove an item, making sure it is no
 *    // longer in the queue (note that this item may already
 *    // have been removed, if it was the 1st or 2nd-highest
 *    // priority item!)
 *    queue.remove( item2 );
 *
 */
template<class T> struct PQueue {
    void insert(const T& item) {
        queue.insert(item);
    }
    void remove(const T& item) {
        if(queue.find(item) != queue.end()) {
            queue.erase(item);
        }
    }
    const T& top(void) const {
        return *(queue.begin());
    }
    void pop(void) {
        queue.erase(queue.begin());
    }
    size_t size() {
        return queue.size();
    }

    std::set<T> queue;
};

/*
    Mesh simplification. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if you can't simplify the mesh any
    further without destroying it.)
*/
bool Halfedge_Mesh::simplify() {

    std::unordered_map<VertexRef, Mat4> vertex_quadrics;
    std::unordered_map<FaceRef, Mat4> face_quadrics;
    std::unordered_map<EdgeRef, Edge_Record> edge_records;
    PQueue<Edge_Record> edge_queue;

    // Compute initial quadrics for each face by simply writing the plane equation
    // for the face in homogeneous coordinates. These quadrics should be stored
    // in face_quadrics
    // -> Compute an initial quadric for each vertex as the sum of the quadrics
    //    associated with the incident faces, storing it in vertex_quadrics
    // -> Build a priority queue of edges according to their quadric error cost,
    //    i.e., by building an Edge_Record for each edge and sticking it in the
    //    queue. You may want to use the above PQueue<Edge_Record> for this.
    // -> Until we reach the target edge budget, collapse the best edge. Remember
    //    to remove from the queue any edge that touches the collapsing edge
    //    BEFORE it gets collapsed, and add back into the queue any edge touching
    //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    //    top of the queue.

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}
