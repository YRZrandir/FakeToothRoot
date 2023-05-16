#include "Polyhedron.h"

struct Triangle
{
public:
    Triangle( unsigned i0, unsigned i1, unsigned i2)
    {
        _id[0] = i0;
        _id[1] = i1;
        _id[2] = i2;
    }
    size_t& operator[](size_t i) { return _id[i]; }
    const size_t& operator[](size_t i) const { return _id[i]; }
    std::pair<size_t, size_t> GetEdge(size_t i)
    {
        switch (i)
        {
        case 0:
            return std::make_pair(_id[0], _id[1]);
            break;
        case 1:
            return std::make_pair(_id[1], _id[2]);
            break;
        case 2:
            return std::make_pair(_id[2], _id[0]);
            break;
        }
        return std::make_pair<size_t, size_t>(0, 0);
    }
protected:
    size_t _id[3]{0, 0, 0};
};

struct ToothFrame
{
    Point_3 centroid;
    Vector_3 up;
};

std::unordered_map<int, ToothFrame> LoadToothFrames( const std::string& path );
std::unordered_map<int, ToothFrame> LoadToothFrames( const char* json );
void LoadLabels( Polyhedron& mesh, std::string path );
void PreprocessMesh( Polyhedron& mesh );
std::vector<Polyhedron> SplitByLabel(Polyhedron& mesh);

void ProcessOneTooth( Polyhedron& m, Point_3 centroid, Vector_3 up);
void ProcessOneToothLaplacian( Polyhedron& m, Point_3 centroid, Vector_3 up);