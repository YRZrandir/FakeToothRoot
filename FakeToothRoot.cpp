#include "FakeToothRoot.h"
#include <CGAL/aff_transformation_tags.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/boost/graph/Face_filtered_graph.h>
#include <CGAL/Surface_mesh_deformation.h>
#include <CGAL/subdivision_method_3.h>
#include <CGAL/Polygon_mesh_processing/refine.h>
#include <CGAL/Polygon_mesh_processing/fair.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <nlohmann/json.hpp>

void LoadLabels( Polyhedron& mesh, std::string path )
{
    using namespace nlohmann;
    std::ifstream label_ifs( path );
    json data = json::parse( label_ifs );
    if (data.find( "labels" ) == data.end())
    {
        std::cout << "Invalid Json" << std::endl;
        return;
    }
    std::vector<int> labels = data["labels"].get<std::vector<int>>();
    if(labels.size() != mesh.size_of_vertices())
    {
        std::cout << "number of labels != number of vertices" << std::endl;
        return;
    }
    
    int count = 0;
    for(auto hv = mesh.vertices_begin(); hv != mesh.vertices_end(); hv++)
    {
        hv->_label = labels[count++];
        if(hv->_label == 100)
            hv->_label = 0;
    }

    for(auto hf : CGAL::faces(mesh))
    {
        int l0 = hf->halfedge()->vertex()->_label;
        int l1 = hf->halfedge()->next()->vertex()->_label;
        int l2 = hf->halfedge()->prev()->vertex()->_label;
        if(l0 == l1 && l1 == l2)
            hf->_label == l0;
        else 
            hf->_label = 0;
    }

    for(auto hf : CGAL::faces(mesh))
    {
        auto hh0 = hf->halfedge();
        auto hh1 = hf->halfedge()->next();
        auto hh2 = hf->halfedge()->prev();

        std::vector<int> neigh_labels;
        if(!hh0->opposite()->is_border())
            neigh_labels.push_back(hh0->opposite()->facet()->_label);
        if(!hh1->opposite()->is_border())
            neigh_labels.push_back(hh1->opposite()->facet()->_label);
        if(!hh2->opposite()->is_border())
            neigh_labels.push_back(hh2->opposite()->facet()->_label);

        if(std::find(neigh_labels.begin(), neigh_labels.end(), hf->_label) == neigh_labels.end())
        {
            if(neigh_labels.size() == 3)
            {
                if(neigh_labels[1] == neigh_labels[2] && neigh_labels[0] != neigh_labels[2])
                    hf->_label = neigh_labels[1];
                else
                    hf->_label = neigh_labels[0];
            }
            else if(neigh_labels.size() == 2 || neigh_labels.size() == 1)
            {
                hf->_label = neigh_labels[0];
            }
        }
    }
}

std::unordered_map<int, ToothFrame> LoadToothFrames( const std::string& path )
{
    using namespace nlohmann;
    std::unordered_map<int, ToothFrame> result;

    std::ifstream ifs(path);
    json data = json::parse(ifs);

    
    for(int i = 11; i < 49; i++)
    {
        if(i % 10 >= 8)
            continue;
        if(data.find(std::to_string(i)) != data.end())
        {
            std::vector<std::vector<float>> frame_raw = data[std::to_string(i)].get<std::vector<std::vector<float>>>();
            Point_3 centroid{ frame_raw[3][0], frame_raw[3][1], frame_raw[3][2] };
            Vector_3 up{frame_raw[2][0], frame_raw[2][1], frame_raw[2][2]};
            result[i] = {centroid, up};
        }
    }
    return result;
}

std::unordered_map<int, ToothFrame> LoadToothFrames( const char* jsonstr )
{
    using namespace nlohmann;
    std::unordered_map<int, ToothFrame> result;

    json data = json::parse(jsonstr);

    for(int i = 11; i < 49; i++)
    {
        if(i % 10 >= 8)
            continue;
        if(data.find(std::to_string(i)) != data.end())
        {
            std::vector<std::vector<float>> frame_raw = data[std::to_string(i)].get<std::vector<std::vector<float>>>();
            Point_3 centroid{ frame_raw[3][0], frame_raw[3][1], frame_raw[3][2] };
            Vector_3 up{frame_raw[2][0], frame_raw[2][1], frame_raw[2][2]};
            result[i] = {centroid, up};
        }
    }
    return result;
}

void PreprocessMesh( Polyhedron& mesh )
{
    std::vector<hHalfedge> faces_to_erase;
    for(hFacet hf = mesh.facets_begin(); hf != mesh.facets_end(); hf++)
    {
        auto hh = hf->halfedge();
        if(!hh->is_border() && (hh->vertex()->_label == 0 ||
        hh->next()->vertex()->_label == 0 || 
        hh->prev()->vertex()->_label == 0) )
        {
            faces_to_erase.push_back(hh);
        }
    }
    for(auto hh : faces_to_erase)
    {
        mesh.erase_facet(hh);
    }
}

std::vector<Polyhedron> SplitByLabel(Polyhedron& mesh)
{
    std::array<std::vector<hFacet>, 49> facet_ranges;
    for(auto hf : CGAL::faces(mesh))
    {
        int l0 = hf->halfedge()->vertex()->_label;
        int l1 = hf->halfedge()->next()->vertex()->_label;
        int l2 = hf->halfedge()->prev()->vertex()->_label;
        if(l0 % 10 < 8 && l0 != 0)
            facet_ranges[l0].push_back(hf);
        if(l1 % 10 < 8 && l1 != 0)
            facet_ranges[l1].push_back(hf);
        if(l2 % 10 < 8 && l2 != 0)
            facet_ranges[l2].push_back(hf);
    }

    std::vector<Polyhedron> meshes;

    for(int i = 11; i < 48; i++)
    {
        if(!facet_ranges[i].empty())
        {
            CGAL::Face_filtered_graph<Polyhedron> filtered_mesh(mesh, facet_ranges[i]);
            Polyhedron mesh_label;
            CGAL::copy_face_graph(filtered_mesh, mesh_label);
            for(auto hf : CGAL::faces(mesh_label))
                hf->_label = i;
            for(auto hv : CGAL::vertices(mesh_label))
                hv->_label = i;
            meshes.emplace_back(std::move(mesh_label));

        }
    }
    return meshes;
}

void ProcessOneTooth( Polyhedron& m, Point_3 centroid, Vector_3 up, int label )
{
    up = -up;
    CGAL::Polygon_mesh_processing::keep_largest_connected_components(m, 1);
    std::vector<hHalfedge> borders;
    CGAL::Polygon_mesh_processing::extract_boundary_cycles(m, std::back_inserter(borders));

    hHalfedge max_hole_edge = *std::max_element(borders.begin(), borders.end(), [&m]( hHalfedge hh0, hHalfedge hh1 ) {
        int len_hole0 = 0;
        for(auto iborder : CGAL::halfedges_around_face(hh0, m))
            len_hole0++;
        int len_hole1 = 0;
        for(auto iborder : CGAL::halfedges_around_face(hh1, m))
            len_hole1++;
        return len_hole0 < len_hole1;
    });

    std::vector<hHalfedge> border_edges;
    for(auto hh : CGAL::halfedges_around_face(max_hole_edge, m))
        border_edges.push_back(hh);

    for(int i = 0; i < 5; i++)
    {
        std::vector<Point_3> new_positions(border_edges.size());
        for(size_t j = 0; j < border_edges.size(); j++)
        {
            auto hh = border_edges[j];
            hHalfedge prev;
            hHalfedge next;
            if(j == 0)
                prev = border_edges.back();
            else
                prev = border_edges[j - 1];
            if(j == border_edges.size() - 1)
                next = border_edges[0];
            else
                next = border_edges[j + 1];
            const Point_3& p0 = prev->vertex()->point();
            const Point_3& p2 = next->vertex()->point();
            Point_3 np = CGAL::midpoint(p0, p2) ;
            new_positions[j] = np;
        }
        
        for(size_t j = 0; j < border_edges.size(); j++)
        {
            border_edges[j]->vertex()->point() = new_positions[j];
        }
    }

    
    float r = 1.5f;
    float d = 7.0f;

    switch( label % 10 )
    {
        case 1:
        case 2:
        case 3:
            r = 1.5f;
            break;
        case 4:
        case 5:
            r = 2.f;
            break;
        case 6:
        case 7:
            r = 2.5f;
            break;
    }

    std::vector<Point_3> circle_pts;
    Point_3 c = centroid + up * d;
    Vector_3 u = CGAL::cross_product(up, Vector_3(1, 0, 0));
    u /= std::sqrt(u.squared_length());
    Vector_3 v = CGAL::cross_product(up, u);
    v /= std::sqrt(v.squared_length());
    Polyhedron::Plane_3 plane(c, c + u, c + v);
    Polyhedron::Traits::Kernel::Line_3 lu(c, u);
    Polyhedron::Traits::Kernel::Line_3 lv(c, v);
    int size_max_hole = 0;
    for(auto hh : CGAL::halfedges_around_face(max_hole_edge, m))
        size_max_hole++;

    for(int i = 0; i < size_max_hole; i++)
    {
        float ang = (float)i / size_max_hole * 2 * 3.14159f;
        Point_3 pt = c + std::sin(ang) * u + std::cos(ang) * v;
        circle_pts.push_back(pt);
    }

    auto centroid_proj = plane.projection(centroid);

    std::vector<hFacet> new_faces;
    std::vector<hHalfedge> new_edges;
    for(int i = 0; i < border_edges.size(); i++)
    {
        auto hh = border_edges[i];
        auto hh1 = border_edges[(i + 1) % border_edges.size()];
        auto hv = hh1->vertex();
        auto proj_point = plane.projection(hv->point());

        auto diff = (proj_point - centroid_proj);
        diff /= std::sqrt(diff.squared_length());
        
        auto pt_on_circle = c + diff * r;

        auto proj_on_u = lu.projection(c + diff);
        auto proj_on_v = lv.projection(c + diff);
        float du = std::sqrt((c - proj_on_u).squared_length());
        float dv = std::sqrt((c - proj_on_v).squared_length());
        
        auto nh = m.add_vertex_and_facet_to_border(hh, hh1);
        new_faces.push_back(nh->face());
        nh->vertex()->point() = pt_on_circle;
        new_edges.push_back(nh->next()->opposite());
        border_edges[(i+1) % border_edges.size()] = nh->opposite();
    }

    for(int i = 0; i < new_edges.size(); i++)
    {
        auto nh = m.add_facet_to_border(new_edges[i], new_edges[(i+1)%new_edges.size()]);
        new_edges[(i+1)%new_edges.size()] = nh->opposite();
        new_faces.push_back(nh->face());
    }

    CGAL::Polygon_mesh_processing::triangulate_hole(m, new_edges[0], std::back_inserter(new_faces));
    CGAL::Polygon_mesh_processing::isotropic_remeshing(new_faces, 0.3, m, CGAL::parameters::number_of_iterations(10));

    {
        std::vector<hHalfedge> borders;
        CGAL::Polygon_mesh_processing::extract_boundary_cycles(m, std::back_inserter(borders));
        for(auto hh : borders)
        {
            CGAL::Polygon_mesh_processing::triangulate_and_refine_hole(m, hh, CGAL::Emptyset_iterator(), CGAL::Emptyset_iterator());
        }
    }
}

void ProcessOneToothLaplacian( Polyhedron& m, Point_3 centroid, Vector_3 up, int label)
{
    up = -up;
    CGAL::Polygon_mesh_processing::keep_largest_connected_components(m, 1);
    std::vector<hHalfedge> borders;
    CGAL::Polygon_mesh_processing::extract_boundary_cycles(m, std::back_inserter(borders));

    auto max_hole = std::max_element(borders.begin(), borders.end(), [&m]( hHalfedge hh0, hHalfedge hh1 ) {
        int len_hole0 = 0;
        for(auto iborder : CGAL::halfedges_around_face(hh0, m))
            len_hole0++;
        int len_hole1 = 0;
        for(auto iborder : CGAL::halfedges_around_face(hh1, m))
            len_hole1++;
        
        return len_hole0 < len_hole1;
    });
    hHalfedge max_hole_edge = *max_hole;

    std::vector<hFacet> patch_facets;
    std::vector<hVertex> patch_vertices;
    CGAL::Polygon_mesh_processing::triangulate_and_refine_hole(m, max_hole_edge, std::back_inserter(patch_facets), std::back_inserter(patch_vertices));

    std::unordered_set<hVertex> vertices_to_fair(patch_vertices.begin(), patch_vertices.end());
    for(int i = 0; i < 4; i++)  //k-rings
    {
        std::unordered_set<hVertex> temp = vertices_to_fair;
        for(auto hv : vertices_to_fair)
        {
            for(auto nei : CGAL::vertices_around_target(hv, m))
                temp.insert(nei);
        }
        vertices_to_fair = std::move(temp);
    }
    CGAL::Polygon_mesh_processing::fair(m, vertices_to_fair);

    KernelEpick::Ray_3 ray(centroid, up);
    hFacet target;
    double t_max = 0.0;
    for(auto hf : patch_facets)
    {
        KernelEpick::Triangle_3 tri { hf->halfedge()->vertex()->point(),
            hf->halfedge()->next()->vertex()->point(),
            hf->halfedge()->next()->next()->vertex()->point()
        };
        auto ret = CGAL::intersection(ray, tri);
        if(ret.has_value() && (*ret).which() == 0)
        {
            Point_3 p = boost::get<Point_3>(*ret);
            double t = CGAL::squared_distance(p, centroid);
            if(t > t_max)
            {
                t_max = t;
                target = hf;
            }
        }
    }

    if(target == nullptr)
        return;

    Point_3 center = CGAL::midpoint(CGAL::midpoint(target->halfedge()->vertex()->point(), target->halfedge()->next()->vertex()->point()), target->halfedge()->prev()->vertex()->point());
    double max_dist = 0.0;
    // for(auto hv : patch_vertices)
    // {
    //     double dist = std::sqrt(CGAL::squared_distance(center, hv->point()));
    //     max_dist = std::max(max_dist, dist);
    // }
    // for(auto hv : patch_vertices)
    // {
    //     double dist = std::sqrt(CGAL::squared_distance(center, hv->point()));
    //     hv->point() += up * (1.0 - dist / max_dist) * 7.0;
    // }

    CGAL::set_halfedgeds_items_id(m);

    CGAL::Surface_mesh_deformation<Polyhedron> deform_mesh(m);

    deform_mesh.set_sre_arap_alpha(1.0);

    float r = 1.5f;
    switch( label % 10 )
    {
        case 1:
        case 2:
        case 3:
            r = 2.0f;
            break;
        case 4:
        case 5:
            r = 2.5f;
            break;
        case 6:
        case 7:
            r = 3.0f;
            break;
    }
    std::unordered_set<hVertex> control_vertices;
    for(auto hv : patch_vertices)
    {
        double dist = std::sqrt(CGAL::squared_distance(center, hv->point()));
        if(dist < r)
        {
            control_vertices.insert(hv);
        }
    }

    deform_mesh.insert_roi_vertices(vertices_to_fair.begin(), vertices_to_fair.end());
    deform_mesh.insert_control_vertices(control_vertices.begin(), control_vertices.end());

    bool success = deform_mesh.preprocess();
    if(!success)
    {
        std::cout << "deform preprocess fail" << std::endl;
    }

    for(auto hv : control_vertices)
    {
        double dist = std::sqrt(CGAL::squared_distance(center, hv->point()));
        KernelEpick::Aff_transformation_3 aff(CGAL::Translation(), up * (4.0 + 1.5 * (1.f - dist / r * dist / r)));
        deform_mesh.set_target_position(hv, hv->point().transform(aff));
    }

    deform_mesh.deform(100, 1e-4);

    {
        std::vector<hHalfedge> borders;
        CGAL::Polygon_mesh_processing::extract_boundary_cycles(m, std::back_inserter(borders));
        for(auto hh : borders)
        {
            CGAL::Polygon_mesh_processing::triangulate_refine_and_fair_hole(m, hh, CGAL::Emptyset_iterator(), CGAL::Emptyset_iterator());
        }
    }
}