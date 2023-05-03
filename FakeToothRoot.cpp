#include "FakeToothRoot.h"
#include <assimp/mesh.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/postprocess.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/boost/graph/Face_filtered_graph.h>
#include <nlohmann/json.hpp>

static bool gVerbose = true;

std::pair<std::vector<Point_3>, std::vector<Triangle>> LoadVFAssimp( const std::string& path )
{
    Assimp::Importer importer;
    importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS, aiComponent_COLORS | aiComponent_NORMALS | aiComponent_TEXCOORDS );
    const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_RemoveComponent);
    if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode || scene->mNumMeshes < 1)
    {
        std::cout << "Input invalid." << std::endl;
        exit(-1);
    }
    const aiMesh* mesh = scene->mMeshes[0];

    auto to_cgal = [](const aiVector3D& v) { return Point_3{v.x, v.y, v.z};};

    std::vector<Point_3> vertices;
    for(int i = 0; i < mesh->mNumVertices; i++)
    {
        vertices.push_back(to_cgal(mesh->mVertices[i]));
    }

    std::vector<Triangle> faces;
    for(int i = 0; i < mesh->mNumFaces; i++)
    {
        const auto& f = mesh->mFaces[i];
        faces.emplace_back(f.mIndices[0], f.mIndices[1], f.mIndices[2]);
    }
    if(gVerbose)
    {
        std::cout << "Loading " << vertices.size() << " vertices, " << faces.size() << " faces. " << std::endl;
    }
    return {vertices, faces};
}

void WriteVFAssimp( const std::vector<Point_3>& vertices, const std::vector<Triangle>& faces, const std::string& path)
{
    Assimp::Exporter exporter;
    auto scene = std::make_unique<aiScene>();

    scene->mRootNode = new aiNode();
    scene->mRootNode->mNumMeshes = 1;
    scene->mRootNode->mMeshes = new unsigned[1];
    scene->mRootNode->mMeshes[0] = 0;

    scene->mNumMaterials = 1;
    scene->mMaterials = new aiMaterial*[]{ new aiMaterial() };
    scene->mMetaData = new aiMetadata();

    scene->mNumMeshes = 1;
    scene->mMeshes = new aiMesh*[1];
    scene->mMeshes[0] = new aiMesh();

    aiMesh* m = scene->mMeshes[0];
    m->mNumFaces = faces.size();
    m->mNumVertices = vertices.size();
    m->mVertices = new aiVector3D[m->mNumVertices];
    m->mFaces = new aiFace[m->mNumFaces];
    m->mPrimitiveTypes = aiPrimitiveType_TRIANGLE;

    for(int i = 0; i < m->mNumVertices; i++)
    {
        m->mVertices[i] = aiVector3D(vertices[i].x(), vertices[i].y(), vertices[i].z());
        
    }

    for(int i = 0; i < m->mNumFaces; i++)
    {
        m->mFaces[i].mNumIndices = 3;
        m->mFaces[i].mIndices = new unsigned int[3];
        m->mFaces[i].mIndices[0] = faces[i][0];
        m->mFaces[i].mIndices[1] = faces[i][1];
        m->mFaces[i].mIndices[2] = faces[i][2];
    }

    std::string postfix = path.substr(path.rfind('.') + 1);
    
    if(postfix == std::string("ply"))
    {
        postfix = "plyb";
    }
    else if (postfix == std::string("stl"))
    {
        postfix = "stlb";
    }
    //Assimp::ExportProperties prop;
    exporter.Export(scene.get(), postfix, path);
}

void WriteCgalPolyAssimp( const Polyhedron& m, const std::string& path )
{
    auto [vertices, faces] = PolyhedronToVF( m );
    WriteVFAssimp(vertices, faces, path);
}

std::pair<std::vector<Point_3>, std::vector<Triangle>> PolyhedronToVF( const Polyhedron& m )
{
    std::vector<Point_3> vertices;

    std::unordered_map<hVertex, unsigned> idmap;
    int count = 0;
    for(auto hv : CGAL::vertices(m))
    {
        idmap[hv] = count;
        vertices.push_back(hv->point());
        count++; 
    }

    std::vector<Triangle> triangles;
    for(auto hf : CGAL::faces(m))
    {
        unsigned i0 = idmap[hf->halfedge()->vertex()];
        unsigned i1 = idmap[hf->halfedge()->next()->vertex()];
        unsigned i2 = idmap[hf->halfedge()->prev()->vertex()];
        triangles.emplace_back(i0, i1, i2);
    }

    return {vertices, triangles};
}

std::unique_ptr<Polyhedron> LoadPolyhedron( const std::string& path )
{
    auto [vertices, faces] = LoadVFAssimp(path);
    std::vector<int> indices;
    for(auto& tri : faces)
    {
        indices.push_back(tri[0]);
        indices.push_back(tri[1]);
        indices.push_back(tri[2]);
    }
    return std::make_unique<Polyhedron>(vertices, indices);
}

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
    std::vector<Polyhedron> meshes;
    for(int i = 11; i < 48; i++)
    {
        std::vector<hFacet> facet_range;
        for(hFacet hf = mesh.facets_begin(); hf != mesh.facets_end(); hf++)
        {
            if(hf->_label == i)
            {
                facet_range.push_back(hf);
            }
        }
        if(facet_range.empty())
            continue;
        CGAL::Face_filtered_graph<Polyhedron> filtered_mesh(mesh, facet_range);
        Polyhedron mesh_label;
        CGAL::copy_face_graph(filtered_mesh, mesh_label);
        for(auto hf : CGAL::faces(mesh_label))
            hf->_label = i;
        for(auto hv : CGAL::vertices(mesh_label))
            hv->_label = i;
        meshes.emplace_back(std::move(mesh_label));
    }
    return meshes;
}

void ProcessOneTooth( Polyhedron& m )
{
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
    borders.erase(max_hole);

    for(auto hh : borders)
    {
        std::vector<hFacet> patch_faces;
        CGAL::Polygon_mesh_processing::triangulate_hole(m, hh, std::back_inserter(patch_faces));
    }

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


    Point_3 center = CGAL::centroid(m.points_begin(), m.points_end());
    Vector_3 dir{0, 0, -1};
    float r = 1.5f;
    float d = 7.0f;
    
    std::vector<Point_3> circle_pts;
    Point_3 c = center + dir * d;
    Vector_3 u = CGAL::cross_product(dir, Vector_3(1, 0, 0));
    u /= std::sqrt(u.squared_length());
    Vector_3 v = CGAL::cross_product(dir, u);
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


    
    std::vector<hHalfedge> new_edges;
    for(int i = 0; i < border_edges.size(); i++)
    {
        auto hh = border_edges[i];
        auto hh1 = border_edges[(i + 1) % border_edges.size()];
        auto hv = hh1->vertex();
        auto proj_point = plane.projection(hv->point());
        auto diff = (proj_point - c);
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