//
// Created by yrz on 7/7/22.
//

#ifndef ELASTICITY_POLYHEDRON_H
#define ELASTICITY_POLYHEDRON_H
#include <memory>
#include <utility>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Bbox_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Color.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <nlohmann/json.hpp>


template <class Refs, typename Tag, typename Point>
class MyVertex : public CGAL::HalfedgeDS_vertex_base<Refs, CGAL::Tag_true, Point>
{
public:
    MyVertex() = default;
    explicit MyVertex( const Point& p ) : CGAL::HalfedgeDS_vertex_base<Refs, CGAL::Tag_true, Point>( p ) {}
    MyVertex(const MyVertex& p) : CGAL::HalfedgeDS_vertex_base<Refs, CGAL::Tag_true, Point>( p ), _label(p._label) {}
    MyVertex(const Point& p, int label) : CGAL::HalfedgeDS_vertex_base<Refs, CGAL::Tag_true, Point>(p), _label(label) {}
public:
    int _label{ 0 };
};

template <class Refs>
class MyFace : public CGAL::HalfedgeDS_face_base<Refs>
{
public:
    MyFace() = default;
    int _label{0};
};



template <typename G>
class LabelConstraint
{
    using edge_descriptor = boost::graph_traits<G>::edge_descriptor;
    using category = boost::readable_property_map_tag;
    using value_type = bool;
    using reference = bool;
    using key_type = edge_descriptor;

    value_type operator[](edge_descriptor e) const
    {
        auto hh0 = e.halfedge();
        auto hh1 = hh0->opposite();
        if(hh0->is_border() || hh1->is_border())
            return true;
        return hh0->facet()->_label != hh1->facet()->_label;
    }
    friend inline
    value_type get(const LabelConstraint& m, const key_type k)
    {
      return m[k];
    }

};


class MyItems : public CGAL::Polyhedron_items_3
{
public:
    template<class Refs, class Traits>
    struct Vertex_wrapper
    {
        typedef typename Traits::Point_3 Point;
        typedef MyVertex<Refs, CGAL::Tag_true, Point> Vertex;
    };

    template<class Refs, class Traits>
    struct Face_wrapper
    {
        using Face = MyFace<Refs>;
    };
};

using KernelEpick = CGAL::Exact_predicates_inexact_constructions_kernel;
using CPolyhedron = CGAL::Polyhedron_3<KernelEpick, MyItems>;
using hHalfedge = CPolyhedron::Halfedge_handle;
using hVertex = CPolyhedron::Vertex_handle;
using hFacet = CPolyhedron::Facet_handle;
using Halfedge = CPolyhedron::Halfedge;
using CVertex = CPolyhedron::Vertex;
using Facet = CPolyhedron::Facet;
using iHalfedge = CPolyhedron::Halfedge_iterator;
using iVertex = CPolyhedron::Vertex_iterator;
using iFacet = CPolyhedron::Facet_iterator;
using Point_3 = CPolyhedron::Point_3;
using Vector_3 = CPolyhedron::Traits::Vector_3;



class Polyhedron : public CPolyhedron
{
public:
    Polyhedron( const std::vector<Point_3>& vertices, const std::vector<int>& indices );
    Polyhedron( const Polyhedron& mesh );
    Polyhedron() : CPolyhedron() {}

    std::pair<std::vector<Point_3>, std::vector<int>> ToVerticesFaces() const;

    void WriteOFF( const std::string& path ) const;
    void WriteOBJ( const std::string& path ) const;
    void PrintInfo() const;
};



template <typename HDS>
class PolyhedronObjBulider : public CGAL::Modifier_base<HDS>
{
public:
    PolyhedronObjBulider( const std::vector<Point_3>& vertices, const std::vector<int>& indices )
        : _vertices(vertices), _indices(indices) {}
    virtual void operator()( HDS& hds ) override;

protected:
    const std::vector<Point_3>& _vertices;
    const std::vector<int>& _indices;
};

template<typename HDS>
inline void PolyhedronObjBulider<HDS>::operator()( HDS& hds )
{
    using namespace nlohmann;
    std::ifstream label_ifs( "../../test/mesh1.json" );
    json data = json::parse( label_ifs );
    if (data.find( "labels" ) == data.end())
    {
        std::cout << "Invalid Json" << std::endl;
        return;
    }
    std::vector<int> labels = data["labels"].get<std::vector<int>>();
    if(labels.size() != _vertices.size())
    {
        std::cout << "number of labels != number of vertices" << std::endl;
        return;
    }

    CGAL::Polyhedron_incremental_builder_3<HDS> builder( hds, true );
    builder.begin_surface( _vertices.size(), _indices.size() / 3 );
    for (size_t i = 0, size = _vertices.size(); i < size; i += 1)
    {
        auto hv = builder.add_vertex( _vertices[i] );
        hv->_label = labels[i];
    }
    for (int f = 0, size = _indices.size() / 3; f < size; ++f)
    {
        builder.begin_facet();
        builder.add_vertex_to_facet( _indices[f * 3 + 0]);
        builder.add_vertex_to_facet( _indices[f * 3 + 1] );
        builder.add_vertex_to_facet( _indices[f * 3 + 2] );
        builder.end_facet();
    }
    builder.end_surface();
}

#define CGAL_GRAPH_TRAITS_INHERITANCE_CLASS_NAME Polyhedron
#define CGAL_GRAPH_TRAITS_INHERITANCE_BASE_CLASS_NAME CPolyhedron
#include <CGAL/boost/graph/graph_traits_inheritance_macros.h>
#undef CGAL_GRAPH_TRAITS_INHERITANCE_CLASS_NAME
#undef CGAL_GRAPH_TRAITS_INHERITANCE_BASE_CLASS_NAME

class FaceLabelMap
{
public:
    using value_type = typename boost::graph_traits<Polyhedron>::faces_size_type;
    using referece_type = typename boost::graph_traits<Polyhedron>::faces_size_type;
    using key_type = boost::graph_traits<Polyhedron>::face_descriptor;
    using category = boost::readable_property_map_tag;

    friend inline typename boost::graph_traits<Polyhedron>::faces_size_type get(FaceLabelMap& m, boost::graph_traits<Polyhedron>::face_descriptor hf )
    {
        return hf->_label;   
    }
};
#endif //ELASTICITY_POLYHEDRON_H
