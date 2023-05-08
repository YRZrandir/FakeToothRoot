#include <filesystem>
#include <iostream>
#include <CGAL/boost/graph/copy_face_graph.h>
#include <CGAL/boost/graph/IO/OBJ.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/version.h>
#include "FakeToothRoot.h"

void GenFakeToothRoot(const float* vertices, const unsigned nb_vertices, const unsigned* indices, unsigned nb_faces, const unsigned* labels, const char* frame_json,
 float** out_vertices, unsigned** out_indices, unsigned* nb_out_vertices, unsigned* nb_out_faces)
{
    std::vector<Point_3> points;
    std::vector<int> faces;
    for(unsigned i = 0; i < nb_vertices; i++)
        points.emplace_back(vertices[i * 3], vertices[i * 3 + 1], vertices[i * 3 + 2]);
    for(unsigned i = 0; i < nb_faces * 3; i++)
        faces.push_back(indices[i]);
    Polyhedron scanmesh(points, faces);
    std::cout << scanmesh.size_of_vertices() << ' ' << scanmesh.size_of_facets() << ' ' << scanmesh.is_valid() << std::endl;
    size_t count = 0;
    for(auto hv : CGAL::vertices(scanmesh))
        hv->_label = labels[hv->_idx];

    for(auto hf : CGAL::faces(scanmesh))
    {
        int l0 = hf->halfedge()->vertex()->_label;
        int l1 = hf->halfedge()->next()->vertex()->_label;
        int l2 = hf->halfedge()->prev()->vertex()->_label;
        hf->_label = std::max(l0, std::max(l1, l2));
    }

    scanmesh.WriteOBJ("../test/testout.obj");
    auto frames = LoadToothFrames(frame_json);
    auto meshes = SplitByLabel(scanmesh);

    for(int i = 0; i < meshes.size(); i++)
    {
        auto& m = meshes[i];
        int label = m.vertices_begin()->_label;
        ProcessOneTooth(m, frames[label].centroid, frames[label].up);
    }

    Polyhedron result_mesh;
    for(auto& m : meshes)
    {
        std::unordered_map<hVertex, hVertex> v2v;
        CGAL::copy_face_graph(m, result_mesh, CGAL::parameters::vertex_to_vertex_map(boost::make_assoc_property_map(v2v)));
        for(auto& [vs, vt] : v2v)
            vt->_label = vs->_label;
    }

    std::cout << result_mesh.size_of_vertices() << " " << result_mesh.size_of_facets() << std::endl;

    *nb_out_vertices = result_mesh.size_of_vertices();
    *nb_out_faces = result_mesh.size_of_facets();

    *out_vertices = new float[*nb_out_vertices * 3];
    *out_indices = new unsigned[*nb_out_faces * 3];

    count = 0;
    std::unordered_map<hVertex, int> idmap;
    for(auto& hv : CGAL::vertices(result_mesh))
    {
        auto& p = hv->point();
        (*out_vertices)[count * 3 + 0] = p.x();
        (*out_vertices)[count * 3 + 1] = p.y();
        (*out_vertices)[count * 3 + 2] = p.z();
        idmap[hv] = count;
        ++count;
    }

    count = 0;
    for(auto& hf : CGAL::faces(result_mesh))
    {
        auto hv0 = hf->halfedge()->vertex();
        auto hv1 = hf->halfedge()->next()->vertex();
        auto hv2 = hf->halfedge()->next()->next()->vertex();
        (*out_indices)[count * 3 + 0] = idmap[hv0];
        (*out_indices)[count * 3 + 1] = idmap[hv1];
        (*out_indices)[count * 3 + 2] = idmap[hv2];
        ++count;
    }
}

void Run(int argc, char* argv[])
{
    std::string input_path;
    std::string output_path;
    std::string frame_path;
    std::string label_path;
    for(int i = 1; i < argc; i++)
    {
        if(std::strcmp(argv[i], "-i") == 0)
        {
            input_path = std::string(argv[i+1]);
        }
        if(std::strcmp(argv[i], "-o") == 0)
        {
            output_path = std::string(argv[i+1]);
        }
        if(std::strcmp(argv[i], "-f") == 0)
        {
            frame_path = std::string(argv[i+1]);
        }
        if(std::strcmp(argv[i], "-l") == 0)
        {
            label_path = std::string(argv[i+1]);
        }
    }

    Polyhedron scanmesh;
    CGAL::IO::read_OBJ(input_path, scanmesh);
    LoadLabels(scanmesh, label_path);
    auto frames = LoadToothFrames(frame_path);
    auto meshes = SplitByLabel(scanmesh);
    for(int i = 0; i < meshes.size(); i++)
    {
        auto& m = meshes[i];
        int label = m.vertices_begin()->_label;
        ProcessOneTooth(m, frames[label].centroid, frames[label].up);
    }

    Polyhedron result_mesh;
    for(auto& m : meshes)
    {
        std::unordered_map<hVertex, hVertex> v2v;
        CGAL::copy_face_graph(m, result_mesh, CGAL::parameters::vertex_to_vertex_map(boost::make_assoc_property_map(v2v)));
        for(auto& [vs, vt] : v2v)
            vt->_label = vs->_label;
    }

    result_mesh.PrintInfo();
    result_mesh.WriteOBJ(output_path);
}

int main(int argc, char* argv[])
{
    Polyhedron scanmesh;
    CGAL::IO::read_OBJ("../test/oral_scan_U.obj", scanmesh);
    LoadLabels(scanmesh, "../test/oral_scan_U.json");

    auto [points, indices] = scanmesh.ToVerticesFaces();
    std::vector<float> vertices;
    for(auto& p : points)
    {
        vertices.push_back(p.x());
        vertices.push_back(p.y());
        vertices.push_back(p.z());
    }

    std::vector<unsigned> labels;
    for(auto& hv : CGAL::vertices(scanmesh))
        labels.push_back(hv->_label);

    std::ifstream frame_ifs("../test/frame.json");
    std::string frame_json{std::istreambuf_iterator<char>(frame_ifs), std::istreambuf_iterator<char>()};

    float* out_vertices{nullptr};
    unsigned* out_indices{nullptr};
    unsigned nb_out_vertices{0};
    unsigned nb_out_faces{0};
    GenFakeToothRoot(vertices.data(), scanmesh.size_of_vertices(), indices.data(), scanmesh.size_of_facets(), labels.data(), frame_json.c_str(), &out_vertices, &out_indices, &nb_out_vertices, &nb_out_faces);

    std::vector<Point_3> out_points;
    std::vector<int> out_faces;
    for(unsigned i = 0; i < nb_out_vertices; i++)
        out_points.emplace_back(out_vertices[i * 3], out_vertices[i * 3 + 1], out_vertices[i * 3 + 2]);
    for(unsigned i = 0; i < nb_out_faces * 3; i++)
        out_faces.push_back(out_indices[i]);
    
    std::cout << out_points.size() << " " << out_faces.size() << std::endl;
    Polyhedron result_mesh{out_points, out_faces};
    result_mesh.WriteOBJ("../test/testout.obj");
    return 0;
}