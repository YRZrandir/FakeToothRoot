#include "FakeToothRoot.h"
#include <CGAL/aff_transformation_tags.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/boost/graph/Face_filtered_graph.h>
#include <CGAL/Surface_mesh_deformation.h>
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
        hf->_label = std::max(l0, std::max(l1, l2));
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
        if(hf->_label % 10 < 8 && hf->_label != 0)
            facet_ranges[hf->_label].push_back(hf);
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

void ProcessOneTooth( Polyhedron& m, Point_3 centroid, Vector_3 up )
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


    // borders.erase(max_hole);

    // for(auto hh : borders)
    // {
    //     std::vector<hFacet> patch_faces;
    //     CGAL::Polygon_mesh_processing::triangulate_hole(m, hh, std::back_inserter(patch_faces));
    // }

    std::vector<hHalfedge> border_edges;
    for(auto hh : CGAL::halfedges_around_face(max_hole_edge, m))
        border_edges.push_back(hh);

    for(int i = 0; i < 5; i++)
    {
        std::vector<Point_3> new_positions(border_edges.size());
        for(size_t j = 0; j < border_edges.size(); j++)
        {
            auto hh = border_edges[j];
            const Point_3& p0 = hh->prev()->vertex()->point();
            const Point_3& p1 = hh->vertex()->point();
            const Point_3& p2 = hh->next()->vertex()->point();
            double l0 = std::sqrt((p1 - p0).squared_length());
            double l1 = std::sqrt((p1 - p2).squared_length());
            Point_3 np = CGAL::midpoint(p0, p2) ;
            new_positions[j] = np;
        }
        
        for(size_t j = 0; j < border_edges.size(); j++)
        {
            border_edges[j]->vertex()->point() = new_positions[j];
        }
    }


    auto aabb = CGAL::bounding_box(m.points_begin(), m.points_end()); 
    float r = 1.5f;
    float d = 7.0f;
    
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
        nh->vertex()->point() = pt_on_circle;
        new_edges.push_back(nh->next()->opposite());
        border_edges[(i+1) % border_edges.size()] = nh->opposite();
    }

    for(int i = 0; i < new_edges.size(); i++)
    {
        auto nh = m.add_facet_to_border(new_edges[i], new_edges[(i+1)%new_edges.size()]);
        new_edges[(i+1)%new_edges.size()] = nh->opposite();
    }

    CGAL::Polygon_mesh_processing::triangulate_hole(m, new_edges[0], CGAL::Emptyset_iterator());
}

void ProcessOneToothLaplacian( Polyhedron& m, Point_3 centroid, Vector_3 up)
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
    CGAL::Polygon_mesh_processing::triangulate_refine_and_fair_hole(m, max_hole_edge, std::back_inserter(patch_facets), std::back_inserter(patch_vertices));

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

    CGAL::set_halfedgeds_items_id(m);

    CGAL::Surface_mesh_deformation<Polyhedron> deform_mesh(m);

    deform_mesh.set_sre_arap_alpha(1.0);

    std::unordered_set<hVertex> control_vertices{ target->halfedge()->vertex(), target->halfedge()->next()->vertex(), target->halfedge()->prev()->vertex() };
    for(int i = 0; i < 1; i++)
    {
        std::vector<hVertex> neighbors;
        for(auto hv : control_vertices)
        {
            for(auto nei : CGAL::vertices_around_target(hv, m))
            {
                neighbors.push_back(nei);
            }
        }
        control_vertices.insert(neighbors.begin(), neighbors.end());
    }

    deform_mesh.insert_roi_vertices(patch_vertices.begin(), patch_vertices.end());
    deform_mesh.insert_control_vertices(control_vertices.begin(), control_vertices.end());

    bool success = deform_mesh.preprocess();
    if(!success)
    {
        std::cout << "deform preprocess fail" << std::endl;
    }

    KernelEpick::Aff_transformation_3 aff(CGAL::Translation(), up * 7.0);
    for(auto hv : control_vertices)
    {
        deform_mesh.set_target_position(hv, hv->point().transform(aff));
    }

    deform_mesh.deform(100, 1e-4);
}