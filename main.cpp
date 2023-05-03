#include <filesystem>
#include <iostream>
#include <CGAL/boost/graph/copy_face_graph.h>
#include <CGAL/boost/graph/IO/OBJ.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/version.h>
#include "FakeToothRoot.h"

int main(int argc, char* argv[])
{
#ifdef _DEBUG
    std::cout << "Mode: Debug" << std::endl;
#endif
    std::cout << "CGAL Version: " << CGAL_STR(CGAL_VERSION) << std::endl;

    std::cout << "working dir=" << std::filesystem::current_path() << std::endl;

    Polyhedron scanmesh;
    CGAL::IO::read_OBJ("../../test/mesh1.obj", scanmesh);
    LoadLabels(scanmesh, "../../test/mesh1.json");
    scanmesh.WriteOBJ("../../test/out.obj");
    PreprocessMesh(scanmesh);
    auto meshes = SplitByLabel(scanmesh);


    for(auto& m : meshes)
    {
        ProcessOneTooth(m);
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
    result_mesh.WriteOBJ("../../test/result.obj");
    return 0;
}