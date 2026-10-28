#include "Stdafx.h"

#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <cstring>
#include <cmath>
#include <iomanip>
#include <map>
#include <algorithm>
#include <excpt.h>              // EXCEPTION_EXECUTE_HANDLER

#pragma warning(push)
#pragma warning(disable: 4248)

// ------------------------------------------------------------------
// Native (MicroStation SDK) headers — isolate from /clr
// ------------------------------------------------------------------
#pragma managed(push, off)
#include <ElementGraphics.h>
#include <GPArray.h>
#include <mselemen.fdf>
#include <msmisc.fdf>
#include <msbsplin.fdf>
#include <mskisolid.fdf>
#include <mselmdsc.fdf>
#include <msscancrit.fdf>
#include <msselect.fdf>
#include <msmdlmesh.fdf>
#include <msmline.fdf>
#include <msdim.fdf>          // mdlDim_* (getActualValues, getStrings, extractPoints, ...)
#include <mscell.fdf>
#include <msscell.fdf>
#include <msraster.fdf>       // mdlRaster_*
#include <msreffil.fdf>       // mdlRefFile_*
#include <mslinkge.fdf>       // mdlLinkage_extractFromElement, extractThickness, getClipBoundaryLinkage
#pragma managed(pop)

using namespace Bentley::Ustn;
using namespace Bentley::Ustn::Element;

#define EXPORT_FILE_PATH "C:\\temp\\RawGeometryExport.txt"

// ============================================================
// RAW GEOMETRY STRUCTURES
// ============================================================

struct RawVertex {
    double x, y, z;
    RawVertex() : x(0), y(0), z(0) {}
    RawVertex(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
};

struct RawPole {
    double x, y, z, w;
    RawPole() : x(0), y(0), z(0), w(1.0) {}
    RawPole(double _x, double _y, double _z, double _w)
        : x(_x), y(_y), z(_z), w(_w) {}
};

struct RawNurbsCurveData {
    int  degree;
    bool closed, rational;
    std::vector<RawPole> poles;
    std::vector<double>  knots;
    RawNurbsCurveData() : degree(3), closed(false), rational(false) {}
};

struct RawNurbsSurfaceData {
    int  uDegree, vDegree;
    int  uNumPoles, vNumPoles;
    bool uClosed, vClosed, rational;
    std::vector<RawPole> poles;
    std::vector<double>  uKnots, vKnots;
    RawNurbsSurfaceData()
        : uDegree(3), vDegree(3), uNumPoles(0), vNumPoles(0),
        uClosed(false), vClosed(false), rational(false) {}
};

struct RawEdgeData {
    int  edgeType;
    bool closed;
    bool reversed;
    int  nativeCurveType;
    std::vector<int> pointIndices;
    RawEdgeData() : edgeType(0), closed(false), reversed(false), nativeCurveType(0) {}
};

struct RawFaceData {
    int       surfaceType;         // 1=planar, 5=b-spline, 0=tessellated
    RawVertex normal, origin;
    RawNurbsSurfaceData bspline;
    std::vector<int>         vertexLoop;
    std::vector<RawEdgeData> edges;
    RawFaceData() : surfaceType(0) {}
};

struct RawSolidBody {
    std::string kernel;
    std::vector<RawVertex>   vertices;
    std::vector<RawFaceData> faces;
};

// ============================================================
// NAME HELPERS
// ============================================================

static const char* RawTypeName(int type) {
    switch (type) {
    case CELL_LIB_ELM:        return "CELL_LIB";
    case CELL_HEADER_ELM:     return "CELL";
    case LINE_ELM:            return "LINE";
    case LINE_STRING_ELM:     return "LINE_STRING";
    case SHAPE_ELM:           return "SHAPE";
    case TEXT_NODE_ELM:       return "TEXT_NODE";
    case CURVE_ELM:           return "CURVE";
    case CMPLX_STRING_ELM:    return "CMPLX_STRING";
    case CONIC_ELM:           return "CONIC";
    case CMPLX_SHAPE_ELM:     return "CMPLX_SHAPE";
    case ELLIPSE_ELM:         return "ELLIPSE";
    case ARC_ELM:             return "ARC";
    case TEXT_ELM:            return "TEXT";
    case SURFACE_ELM:         return "SURFACE";
    case SOLID_ELM:           return "SOLID";
    case BSPLINE_POLE_ELM:    return "BSPLINE_POLE";
    case POINT_STRING_ELM:    return "POINT_STRING";
    case CONE_ELM:            return "CONE";
    case BSPLINE_SURFACE_ELM: return "BSPLINE_SURFACE";
    case BSURF_BOUNDARY_ELM:  return "BSURF_BOUNDARY";
    case BSPLINE_KNOT_ELM:    return "BSPLINE_KNOT";
    case BSPLINE_CURVE_ELM:   return "BSPLINE_CURVE";
    case BSPLINE_WEIGHT_ELM:  return "BSPLINE_WEIGHT";
    case DIMENSION_ELM:       return "DIMENSION";
    case SHAREDCELL_DEF_ELM:  return "SHAREDCELL_DEF";
    case SHARED_CELL_ELM:     return "SHARED_CELL";
    case MULTILINE_ELM:       return "MULTILINE";
    case MESH_HEADER_ELM:     return "MESH";
    case EXTENDED_ELM:        return "EXTENDED";
    case EXTENDED_NONGRAPHIC_ELM: return "EXTENDED_NONGRAPHIC";
    case RASTER_HDR:          return "RASTER_HDR";
    case RASTER_COMP:         return "RASTER_COMP";
    case RASTER_REFERENCE_ELM:    return "RASTER_REFERENCE";
    case RASTER_REFERENCE_COMP:   return "RASTER_REFERENCE_COMP";
    case RASTER_HIERARCHY_ELM:    return "RASTER_HIERARCHY";
    case RASTER_HIERARCHY_COMP:   return "RASTER_HIERARCHY_COMP";
    case RASTER_FRAME_ELM:    return "RASTER_FRAME";
    case TABLE_ENTRY_ELM:     return "TABLE_ENTRY";
    case TABLE_ELM:           return "TABLE";
    case VIEW_GROUP_ELM:      return "VIEW_GROUP";
    case VIEW_ELM:            return "VIEW";
    case LEVEL_MASK_ELM:      return "LEVEL_MASK";
    case REFERENCE_ATTACH_ELM:return "REFERENCE_ATTACH";
    case MATRIX_HEADER_ELM:   return "MATRIX_HEADER";
    case MATRIX_INT_DATA_ELM: return "MATRIX_INT_DATA";
    case MATRIX_DOUBLE_DATA_ELM: return "MATRIX_DOUBLE_DATA";
    case REFERENCE_OVERRIDE_ELM: return "REFERENCE_OVERRIDE";
    case NAMED_GROUP_HDR_ELM: return "NAMED_GROUP_HDR";
    case NAMED_GROUP_COMPONENT_ELM: return "NAMED_GROUP_COMP";
    case ATTRIBUTE_ELM:       return "ATTRIBUTE";
    case MICROSTATION_ELM:    return "MICROSTATION";
    case GROUP_DATA_ELM:      return "GROUP_DATA";
    case DIG_SETDATA_ELM:     return "DIG_SETDATA";
    case DGNFIL_HEADER_ELM:   return "DGNFIL_HEADER";
    case LEV_SYM_ELM:         return "LEV_SYM";
    default:                  return "UNKNOWN";
    }
}

static const char* RawClass_Name(int cls) {
    switch (cls) {
    case PRIMARY_CLASS:            return "PRIMARY";
    case PATTERN_COMPONENT_CLASS:  return "PATTERN_COMPONENT";
    case CONSTRUCTION_CLASS:       return "CONSTRUCTION";
    case DIMENSION_CLASS:          return "DIMENSION";
    case PRIMARY_RULE_CLASS:       return "PRIMARY_RULE";
    case LINEAR_PATTERNED_CLASS:   return "LINEAR_PATTERNED";
    case CONSTRUCTION_RULE_CLASS:  return "CONSTRUCTION_RULE";
    default:                       return "UNKNOWN";
    }
}

static bool IsElement3D(MSElementDescr* edP, int depth = 0) {
    if (!edP || depth > 16) return false;
    if (!edP->el.hdr.ehdr.isGraphics) return false;
    if (edP->el.hdr.dhdr.props.b.is3d) return true;
    if (edP->el.hdr.ehdr.isComplexHeader) {
        for (MSElementDescr* c = edP->h.firstElem; c; c = c->h.next)
            if (IsElement3D(c, depth + 1)) return true;
    }
    return false;
}

static inline void W_V(std::ostream& o, double x, double y, double z) {
    o << "(" << x << ", " << y << ", " << z << ")";
}

static inline void W_P(std::ostream& o, double x, double y, double z, double w) {
    o << "(" << x << ", " << y << ", " << z << ", w=" << w << ")";
}

static inline RawVertex ApplyXf(DPoint3d p, Transform const& t) {
    DPoint3d q = p;
    mdlTMatrix_transformPoint(&q, &t);
    return RawVertex((double)q.x, (double)q.y, (double)q.z);
}

// Нормаль: обратная транспонированная (корректно при non-uniform scale)
static inline RawVertex ApplyXfRotScale(DVec3d v, Transform const& t) {
    double M[3][3] = {
        { t.form3d[0][0], t.form3d[0][1], t.form3d[0][2] },
        { t.form3d[1][0], t.form3d[1][1], t.form3d[1][2] },
        { t.form3d[2][0], t.form3d[2][1], t.form3d[2][2] }
    };
    double c00 = M[1][1] * M[2][2] - M[1][2] * M[2][1];
    double c01 = M[1][2] * M[2][0] - M[1][0] * M[2][2];
    double c02 = M[1][0] * M[2][1] - M[1][1] * M[2][0];
    double c10 = M[0][2] * M[2][1] - M[0][1] * M[2][2];
    double c11 = M[0][0] * M[2][2] - M[0][2] * M[2][0];
    double c12 = M[0][1] * M[2][0] - M[0][0] * M[2][1];
    double c20 = M[0][1] * M[1][2] - M[0][2] * M[1][1];
    double c21 = M[0][2] * M[1][0] - M[0][0] * M[1][2];
    double c22 = M[0][0] * M[1][1] - M[0][1] * M[1][0];

    double det = M[0][0] * c00 + M[0][1] * c01 + M[0][2] * c02;
    if (fabs(det) < 1e-15) det = 1.0;

    double nx = (c00 * v.x + c10 * v.y + c20 * v.z) / det;
    double ny = (c01 * v.x + c11 * v.y + c21 * v.z) / det;
    double nz = (c02 * v.x + c12 * v.y + c22 * v.z) / det;

    double len = sqrt(nx * nx + ny * ny + nz * nz);
    if (len > 1e-15) { nx /= len; ny /= len; nz /= len; }
    return RawVertex(nx, ny, nz);
}

static inline int RoundMicro(double v) {
    return (v >= 0.0) ? (int)(v * 1e6 + 0.5) : (int)(v * 1e6 - 0.5);
}

struct VertexKey {
    int x, y, z;
    VertexKey() : x(0), y(0), z(0) {}
    VertexKey(int _x, int _y, int _z) : x(_x), y(_y), z(_z) {}
    bool operator<(VertexKey const& r) const {
        if (x != r.x) return x < r.x;
        if (y != r.y) return y < r.y;
        return z < r.z;
    }
};

// ============================================================
// NURBS EXTRACTION (native poles / knots / weights)
// ============================================================

static void ExtractRawNurbsCurve(MSBsplineCurveCR curve, Transform const& xf,
    RawNurbsCurveData& out)
{
    out.degree = (int)curve.params.order - 1;
    out.closed = (curve.params.closed != 0);
    out.rational = (curve.weights != NULL);

    int nP = (int)curve.params.numPoles;
    out.poles.reserve(nP);
    for (int i = 0; i < nP; ++i) {
        double w = (curve.weights != NULL) ? curve.weights[i] : 1.0;
        DPoint3d p = curve.poles[i];
        mdlTMatrix_transformPoint(&p, &xf);
        out.poles.push_back(RawPole((double)p.x, (double)p.y, (double)p.z, w));
    }

    int nK = mdlBspline_numberKnots(nP, (int)curve.params.order,
        (int)curve.params.closed);
    if (curve.knots) {
        for (int i = 0; i < nK; ++i) out.knots.push_back(curve.knots[i]);
    }
    else {
        for (int i = 0; i < nK; ++i)
            out.knots.push_back((double)i / (double)(nK - 1));
    }
}

static void ExtractRawNurbsSurface(MSBsplineSurfaceCR s, Transform const& xf,
    RawNurbsSurfaceData& out)
{
    out.uDegree = (int)s.uParams.order - 1;
    out.vDegree = (int)s.vParams.order - 1;
    out.uNumPoles = (int)s.uParams.numPoles;
    out.vNumPoles = (int)s.vParams.numPoles;
    out.uClosed = (s.uParams.closed != 0);
    out.vClosed = (s.vParams.closed != 0);
    out.rational = (s.weights != NULL);

    int total = out.uNumPoles * out.vNumPoles;
    out.poles.reserve(total);
    for (int i = 0; i < total; ++i) {
        double w = (s.weights != NULL) ? s.weights[i] : 1.0;
        DPoint3d p = s.poles[i];
        mdlTMatrix_transformPoint(&p, &xf);
        out.poles.push_back(RawPole((double)p.x, (double)p.y, (double)p.z, w));
    }

    int uK = mdlBspline_numberKnots(out.uNumPoles, (int)s.uParams.order,
        (int)s.uParams.closed);
    int vK = mdlBspline_numberKnots(out.vNumPoles, (int)s.vParams.order,
        (int)s.vParams.closed);

    if (s.uKnots) { for (int i = 0; i < uK; ++i) out.uKnots.push_back(s.uKnots[i]); }
    else { for (int i = 0; i < uK; ++i) out.uKnots.push_back((double)i / (uK - 1)); }

    if (s.vKnots) { for (int i = 0; i < vK; ++i) out.vKnots.push_back(s.vKnots[i]); }
    else { for (int i = 0; i < vK; ++i) out.vKnots.push_back((double)i / (vK - 1)); }
}

// ============================================================
// B-REP EXTRACTOR (solid / sheet topology)
// ============================================================

class BrepExtractor
{
public:
    RawSolidBody& sb;
    std::map<VertexKey, int> vertexMap;
    explicit BrepExtractor(RawSolidBody& s) : sb(s) {}

    int GetOrAddVertex(RawVertex const& v) {
        VertexKey key(RoundMicro(v.x), RoundMicro(v.y), RoundMicro(v.z));
        std::map<VertexKey, int>::iterator it = vertexMap.find(key);
        if (it != vertexMap.end()) return it->second;
        int idx = (int)sb.vertices.size();
        sb.vertices.push_back(v);
        vertexMap[key] = idx;
        return idx;
    }

    bool SampleEdge(KIEDGE* edge, Transform const& xf, RawEdgeData& out)
    {
        KIVERTEX* v0 = NULL, * v1 = NULL;
        DPoint3d startPt, endPt;
        double   startParam = 0, endParam = 0;
        KICOEDGE* co = NULL; void* curve = NULL; int reversed = 0;

        if (SUCCESS != mdlKISolid_edgeData(&v0, &v1, &startPt, &endPt,
            &startParam, &endParam,
            &co, &curve, &reversed, edge))
            return false;

        out.edgeType = mdlKISolid_getEdgeType(edge);
        out.reversed = (reversed != 0);
        out.nativeCurveType = (curve != NULL) ? 1 : 0;

        double du = endParam - startParam;
        double dx = (double)endPt.x - (double)startPt.x;
        double dy = (double)endPt.y - (double)startPt.y;
        double dz = (double)endPt.z - (double)startPt.z;
        bool samePoint = (dx * dx + dy * dy + dz * dz) < 1e-12;
        out.closed = samePoint && (fabs(du) > 1e-9);

        int N = 16;
        if (fabs(du) > 1e-12) {
            DPoint3d p0, pM, p1, d1, d2, tan, nor, bin;
            double uM = 0.5 * (startParam + endParam);
            bool ok0 = (SUCCESS == mdlKISolid_evaluateEdge(&p0, &d1, &d2, &tan, &nor, &bin, startParam, edge));
            bool okM = (SUCCESS == mdlKISolid_evaluateEdge(&pM, &d1, &d2, &tan, &nor, &bin, uM, edge));
            bool ok1 = (SUCCESS == mdlKISolid_evaluateEdge(&p1, &d1, &d2, &tan, &nor, &bin, endParam, edge));
            if (ok0 && okM && ok1) {
                DVec3d va, vb, cross;
                mdlVec_subtractDPoint3dDPoint3d(&va, &pM, &p0);
                mdlVec_subtractDPoint3dDPoint3d(&vb, &p1, &p0);
                mdlVec_crossProduct(&cross, &va, &vb);
                double cm = mdlVec_magnitude(&cross);
                double sc = mdlVec_magnitude(&vb);
                N = (cm < 1e-6 * (sc * sc + 1e-9)) ? 1 : (out.closed ? 32 : 16);
            }
        }

        out.pointIndices.clear();
        out.pointIndices.reserve(N + 1);
        for (int k = 0; k <= N; ++k) {
            double   t = (double)k / (double)N;
            double   u = startParam + du * t;
            DPoint3d pt, d1, d2, tan, nor, bin;
            RawVertex w;
            if (SUCCESS == mdlKISolid_evaluateEdge(&pt, &d1, &d2, &tan, &nor, &bin, u, edge))
                w = ApplyXf(pt, xf);
            else if (k == 0) w = ApplyXf(startPt, xf);
            else if (k == N) w = ApplyXf(endPt, xf);
            else {
                DPoint3d ip; ip.x = startPt.x + t * dx; ip.y = startPt.y + t * dy; ip.z = startPt.z + t * dz;
                w = ApplyXf(ip, xf);
            }
            out.pointIndices.push_back(GetOrAddVertex(w));
        }

        // Учёт reversed: обратный порядок точек
        if (out.reversed) {
            std::reverse(out.pointIndices.begin(), out.pointIndices.end());
        }
        return true;
    }

    bool Extract(KIBODY* body, const char* kernel, Transform const& xf)
    {
        sb.kernel = kernel;
        KIENTITY_LIST* faceList = NULL;
        mdlKISolid_listCreate(&faceList);
        mdlKISolid_getFaceList(faceList, body);

        int faceCount = 0;
        mdlKISolid_listCount(&faceCount, faceList);

        for (int f = 0; f < faceCount; ++f) {
            KIFACE* face = NULL;
            mdlKISolid_listNthEl((void**)&face, faceList, f);
            if (!face) continue;

            RawFaceData fd;
            int fType = mdlKISolid_getFaceType(face);

            if (fType == 1) { // planar
                DPoint3d orig, nrm;
                if (SUCCESS == mdlKISolid_planarFaceData(&orig, &nrm, face)) {
                    fd.surfaceType = 1;
                    fd.origin = ApplyXf(orig, xf);
                    DVec3d vn; vn.x = nrm.x; vn.y = nrm.y; vn.z = nrm.z;
                    fd.normal = ApplyXfRotScale(vn, xf);
                }
            }
            else if (fType != 0) {
                MSElementDescr* faceEdP = NULL;
                if (SUCCESS == mdlKISolid_faceToElement(&faceEdP, face, FALSE, 8, NULL, NULL) && faceEdP) {
                    MSBsplineSurface surf;
                    memset(&surf, 0, sizeof(surf));
                    if (SUCCESS == mdlBspline_convertToSurface(&surf, faceEdP)) {
                        fd.surfaceType = 5;
                        ExtractRawNurbsSurface(surf, xf, fd.bspline);
                        mdlBspline_freeSurface(&surf);
                    }
                    mdlElmdscr_freeAll(&faceEdP);
                }
            }

            KIENTITY_LIST* edgeList = NULL;
            mdlKISolid_listCreate(&edgeList);
            mdlKISolid_getEdgeListFromFace(edgeList, face);

            int edgeCount = 0;
            mdlKISolid_listCount(&edgeCount, edgeList);
            for (int e = 0; e < edgeCount; ++e) {
                KIEDGE* edge = NULL;
                mdlKISolid_listNthEl((void**)&edge, edgeList, e);
                if (!edge) continue;
                RawEdgeData ed;
                if (SampleEdge(edge, xf, ed)) fd.edges.push_back(ed);
            }
            mdlKISolid_listDelete(&edgeList);

            for (size_t e = 0; e < fd.edges.size(); ++e) {
                RawEdgeData const& ed = fd.edges[e];
                size_t startK = 0;
                if (!fd.vertexLoop.empty() && !ed.pointIndices.empty() &&
                    fd.vertexLoop.back() == ed.pointIndices[0])
                    startK = 1;
                for (size_t k = startK; k < ed.pointIndices.size(); ++k) {
                    if (ed.closed && k == ed.pointIndices.size() - 1) continue;
                    fd.vertexLoop.push_back(ed.pointIndices[k]);
                }
            }
            sb.faces.push_back(fd);
        }

        mdlKISolid_listDelete(&faceList);
        return !sb.faces.empty();
    }
};

// ============================================================
// SEH-ISOLATED WRAPPERS
// ============================================================

static bool SafeElementToBody(MSElementDescr* edP, DgnModelRefP modelRef,
    KIBODY** ppBody, Transform* pXf)
{
    *ppBody = NULL;
    __try {
        mdlTMatrix_getIdentity(pXf);
        return SUCCESS == mdlKISolid_elementToBody2(ppBody, pXf, edP, modelRef, 1L, FALSE);
    }
    __except (EXCEPTION_EXECUTE_HANDLER) {
        *ppBody = NULL;
        return false;
    }
}

static bool SampleBodySEH(BrepExtractor* ex, KIBODY* body, Transform const& xf)
{
    __try {
        return ex->Extract(body, "Parasolid", xf);
    }
    __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

static bool SafeBodyToRawSolid(KIBODY* body, Transform const& xf,
    RawSolidBody& outSb, std::ostream& log)
{
    BrepExtractor ex(outSb);
    bool ok = SampleBodySEH(&ex, body, xf);
    if (!ok) {
        outSb.vertices.clear();
        outSb.faces.clear();
        log << "#   [SEH] AV caught in BrepExtractor; partial data dropped\n";
        return false;
    }
    return true;
}

// ============================================================
// MAIN EXTRACTOR
// ============================================================

class RawGeometryExtractor
{
public:
    int m_nextId;
    int m_maxDepth;
    RawGeometryExtractor() : m_nextId(1), m_maxDepth(32) {}

    void Extract(MSElementDescr* edP, DgnModelRefP modelRef, std::ostream& out)
    {
        DumpElement(edP, modelRef, out, 0);
    }

private:
    // -- Dispatcher ------------------------------------------------
    void DumpElement(MSElementDescr* edP, DgnModelRefP modelRef,
        std::ostream& out, int depth)
    {
        if (!edP || depth > m_maxDepth) return;

        std::string ind(depth * 2, ' ');
        int id = m_nextId++;
        int typeNum = mdlElement_getType(&edP->el);

        out << ind << "ELEMENT {\n";
        out << ind << "  ID: " << id << "\n";
        out << ind << "  UNIQUE_ID: " << edP->el.ehdr.uniqueId << "\n";
        out << ind << "  TYPE: " << RawTypeName(typeNum) << " (" << typeNum << ")\n";

        // --- Метаданные заголовка ---
        out << ind << "  ELEMENT_SIZE: " << edP->el.ehdr.elementSize << "\n";
        out << ind << "  ATTR_OFFSET: " << edP->el.ehdr.attrOffset << "\n";
        out << ind << "  LAST_MODIFIED: " << edP->el.ehdr.lastModified << "\n";
        out << ind << "  ARCHIVE: " << edP->el.ehdr.archive << "\n";
        out << ind << "  DELETED: " << edP->el.ehdr.deleted << "\n";
        out << ind << "  LOCKED: " << edP->el.ehdr.locked << "\n";
        out << ind << "  NON_MODEL: " << edP->el.ehdr.nonModel << "\n";
        out << ind << "  IS_COMPLEX_HEADER: " << edP->el.ehdr.isComplexHeader << "\n";
        out << ind << "  IS_COMPLEX: " << edP->el.ehdr.complex << "\n";
        out << ind << "  IS_GRAPHICS: " << edP->el.ehdr.isGraphics << "\n";

        UInt32 lvl = 0, col = 0, wgt = 0; Int32 sty = 0;
        mdlElement_getSymbology(&col, &wgt, &sty, &edP->el);
        int elClass = -1, locked = -1, newElm = -1, modified = -1;
        int viewIndepend = -1, solidHole = -1;
        UInt32 ggNum = 0;
        mdlElement_getProperties(&lvl, &ggNum, &elClass, &locked, &newElm,
            &modified, &viewIndepend, &solidHole, &edP->el);
        out << ind << "  LEVEL: " << lvl << "\n";
        out << ind << "  COLOR: " << col << "\n";
        out << ind << "  WEIGHT: " << wgt << "\n";
        out << ind << "  STYLE: " << sty << "\n";
        out << ind << "  CLASS: " << RawClass_Name(elClass) << " (" << elClass << ")\n";
        out << ind << "  GRAPHIC_GROUP: " << ggNum << "\n";
        out << ind << "  LOCKED_FLAG: " << locked << "\n";
        out << ind << "  NEW: " << newElm << "\n";
        out << ind << "  MODIFIED: " << modified << "\n";
        out << ind << "  VIEW_INDEPENDENT: " << viewIndepend << "\n";
        out << ind << "  SOLID_HOLE: " << solidHole << "\n";
        out << ind << "  IS_3D: " << (IsElement3D(edP) ? 1 : 0) << "\n";

        if (edP->el.hdr.ehdr.isGraphics) {
            out << ind << "  PRIORITY: " << edP->el.hdr.dhdr.priority << "\n";
            out << ind << "  INVISIBLE: " << edP->el.hdr.dhdr.props.b.invisible << "\n";
            out << ind << "  DYNAMIC_RANGE: " << edP->el.hdr.dhdr.props.b.dynamicRange << "\n";
            out << ind << "  PLANAR: " << edP->el.hdr.dhdr.props.b.p << "\n";
            out << ind << "  SNAPPABLE: " << (edP->el.hdr.dhdr.props.b.s ? 0 : 1) << "\n";
            out << ind << "  HOLE_SOLID: " << edP->el.hdr.dhdr.props.b.h << "\n";
            out << ind << "  RANGE: low=";
            W_V(out, (double)edP->el.hdr.dhdr.range.xlowlim,
                (double)edP->el.hdr.dhdr.range.ylowlim,
                (double)edP->el.hdr.dhdr.range.zlowlim);
            out << " high=";
            W_V(out, (double)edP->el.hdr.dhdr.range.xhighlim,
                (double)edP->el.hdr.dhdr.range.yhighlim,
                (double)edP->el.hdr.dhdr.range.zhighlim);
            out << "\n";
        }

        // --- Атрибуты и linkages ---
        DumpAttributes(edP, out, ind);

        switch (typeNum) {
        case LINE_ELM:
        case LINE_STRING_ELM:
        case POINT_STRING_ELM:
            DumpLinear(edP, out, ind);
            break;

        case SHAPE_ELM:
            DumpShape(edP, out, ind);
            break;

        case CURVE_ELM:
            DumpCurve(edP, out, ind);
            break;

        case CONIC_ELM:
            DumpConic(edP, out, ind);
            break;

        case BSPLINE_POLE_ELM:
            DumpBsplinePole(edP, out, ind);
            break;

        case ARC_ELM:
            DumpArc(edP, out, ind);
            break;

        case ELLIPSE_ELM:
            DumpEllipse(edP, out, ind);
            break;

        case CONE_ELM:
            DumpCone(edP, out, ind);
            break;

        case TEXT_ELM:
            DumpText(edP, out, ind);
            break;

        case TEXT_NODE_ELM:
            DumpTextNode(edP, out, ind);
            DumpChildren(edP, modelRef, out, depth + 1);
            break;

        case BSPLINE_CURVE_ELM:
            DumpBsplineCurve(edP, out, ind);
            break;

        case BSPLINE_SURFACE_ELM:
        case SURFACE_ELM:
            if (!DumpBrep(edP, modelRef, out, ind))
                DumpChildren(edP, modelRef, out, depth + 1);
            break;

        case SOLID_ELM:
            if (!DumpBrep(edP, modelRef, out, ind))
                DumpChildren(edP, modelRef, out, depth + 1);
            break;

        case CELL_HEADER_ELM:
        case SHARED_CELL_ELM:
        case SHAREDCELL_DEF_ELM:
            DumpCell(edP, out, ind);
            if (!DumpBrep(edP, modelRef, out, ind))
                DumpChildren(edP, modelRef, out, depth + 1);
            break;

        case MESH_HEADER_ELM:
            DumpMesh(edP, out, ind);
            break;

        case CMPLX_STRING_ELM:
        case CMPLX_SHAPE_ELM:
            DumpChildren(edP, modelRef, out, depth + 1);
            break;

        case DIMENSION_ELM:
            DumpDimension(edP, out, ind);
            DumpChildren(edP, modelRef, out, depth + 1);
            break;

        case MULTILINE_ELM:
            DumpMultiline(edP, out, ind);
            DumpChildren(edP, modelRef, out, depth + 1);
            break;

        case EXTENDED_ELM:
            DumpChildren(edP, modelRef, out, depth + 1);
            break;

        case EXTENDED_NONGRAPHIC_ELM:
            out << ind << "  (non-graphic extended element)\n";
            break;

        case RASTER_HDR:
            DumpRasterHdr(edP, out, ind);
            DumpChildren(edP, modelRef, out, depth + 1);
            break;

        case RASTER_COMP:
            DumpRasterComp(edP, out, ind);
            break;

        case REFERENCE_ATTACH_ELM:
            DumpReferenceAttach(edP, out, ind);
            break;

        case MATRIX_HEADER_ELM:
            out << ind << "  (matrix header — data in components)\n";
            DumpChildren(edP, modelRef, out, depth + 1);
            break;

        case MATRIX_INT_DATA_ELM:
        case MATRIX_DOUBLE_DATA_ELM:
            out << ind << "  (matrix data — see mesh/bsurf parent)\n";
            break;

        case ATTRIBUTE_ELM:
            DumpAttributeElement(edP, out, ind);
            break;

        case BSURF_BOUNDARY_ELM:
            DumpBsurfBoundary(edP, out, ind);
            break;

        case BSPLINE_KNOT_ELM:
            DumpBsplineKnot(edP, out, ind);
            break;

        case BSPLINE_WEIGHT_ELM:
            DumpBsplineWeight(edP, out, ind);
            break;

        default:
            out << ind << "  (no native dumper for this type)\n";
            break;
        }

        out << ind << "}\n";
    }

    // -- Children traversal ---------------------------------------
    void DumpChildren(MSElementDescr* edP, DgnModelRefP modelRef,
        std::ostream& out, int depth)
    {
        if (!edP || depth > m_maxDepth) return;
        MSElementDescr* child = edP->h.firstElem;
        if (!child) return;
        std::string ind(depth * 2, ' ');
        out << ind << "COMPONENTS {\n";
        for (; child; child = child->h.next)
            DumpElement(child, modelRef, out, depth + 1);
        out << ind << "}\n";
    }

    // -- Attributes / linkages -------------------------------------
    struct LinkEnumCtx {
        std::ostream* out;
        std::string   ind;
        int           count;
    };

    static int __cdecl LinkEnumCallback(void* linkageP, void* paramsP)
    {
        LinkEnumCtx* ctx = static_cast<LinkEnumCtx*>(paramsP);
        if (!ctx || !linkageP) return 0;
        LinkageHeader* hdr = static_cast<LinkageHeader*>(linkageP);
        int words = mdlLinkage_getWords(hdr);
        (*ctx->out) << ctx->ind << "    LINKAGE[" << ctx->count << "]: primaryID="
            << hdr->primaryID
            << " words=" << words
            << " user=" << hdr->user
            << " modified=" << hdr->modified
            << " remote=" << hdr->remote
            << " info=" << hdr->info << "\n";
        ctx->count++;
        return 0;
    }

    void DumpAttributes(MSElementDescr* edP, std::ostream& out, std::string const& ind)
    {
        short attrs[MAX_ATTRIBSIZE];
        int   len = 0;
        memset(attrs, 0, sizeof(attrs));
        mdlElmdscr_extractAttributes(&len, attrs, edP);
        if (len > 0) {
            out << ind << "  ATTRIBUTES: len=" << len << " words, data:";
            for (int i = 0; i < len && i < 16; ++i) out << " " << attrs[i];
            if (len > 16) out << " ...";
            out << "\n";
        }

        // Linkages: обход через callback (reqID=0 — DMRS)
        LinkEnumCtx ctx = { &out, ind, 0 };
        mdlLinkage_extractFromElement(NULL, &edP->el, 0, 0, NULL,
            (MdlFunctionP)LinkEnumCallback, &ctx);
        if (ctx.count == 0) {
            out << ind << "  LINKAGES: (none found for reqID=0)\n";
        }

        // Thickness linkage
        double thickness = 0;
        DPoint3d dir;
        BoolInt capped = 0;
        memset(&dir, 0, sizeof(dir));
        if (SUCCESS == mdlLinkage_extractThickness(&thickness, &dir, &capped, &edP->el)) {
            out << ind << "  THICKNESS: " << thickness
                << " capped=" << capped
                << " dir=(" << (double)dir.x << ", " << (double)dir.y << ", " << (double)dir.z << ")\n";
        }

        // Clip boundary linkage
        RotMatrix rmClip;
        DPoint3d  clipOrigin;
        BoolInt   frontOn = 0, backOn = 0;
        double    zFront = 0, zBack = 0;
        UInt32    nClip = 0;
        DPoint2d  clipPts[256];
        memset(&rmClip, 0, sizeof(rmClip));
        memset(&clipOrigin, 0, sizeof(clipOrigin));
        if (SUCCESS == mdlLinkage_getClipBoundaryLinkage(&rmClip, &clipOrigin,
            &frontOn, &zFront, &backOn, &zBack,
            &nClip, clipPts, 256, &edP->el)) {
            out << ind << "  CLIP_BOUNDARY: nPoints=" << nClip
                << " frontOn=" << frontOn << " zFront=" << zFront
                << " backOn=" << backOn << " zBack=" << zBack << "\n";
            for (UInt32 i = 0; i < nClip && i < 256; ++i) {
                out << ind << "    CP[" << i << "]: ("
                    << (double)clipPts[i].x << ", " << (double)clipPts[i].y << ")\n";
            }
        }
    }

    // -- Native primitive dumps -----------------------------------
    void DumpLinear(MSElementDescr* edP, std::ostream& out, std::string const& ind)
    {
        std::vector<DPoint3d> pts(MAX_VERTICES);
        int nPts = 0;
        if (SUCCESS != mdlLinear_extract(&pts[0], &nPts, &edP->el, NULL) || nPts <= 0) {
            out << ind << "  (no linear vertices extracted)\n";
            return;
        }
        if (nPts > MAX_VERTICES) nPts = MAX_VERTICES;
        out << ind << "  GEOMETRY {\n";
        out << ind << "    NUM_VERTICES: " << nPts << "\n";
        for (int i = 0; i < nPts; ++i) {
            out << ind << "    V[" << i << "]: ";
            W_V(out, (double)pts[i].x, (double)pts[i].y, (double)pts[i].z);
            out << "\n";
        }
        out << ind << "  }\n";
    }

    void DumpShape(MSElementDescr* edP, std::ostream& out, std::string const& ind)
    {
        std::vector<DPoint3d> pts(MAX_VERTICES);
        int nPts = 0;
        if (SUCCESS == mdlLinear_extract(&pts[0], &nPts, &edP->el, NULL) && nPts > 0) {
            if (nPts > MAX_VERTICES) nPts = MAX_VERTICES;
            out << ind << "  GEOMETRY {\n";
            out << ind << "    KIND: SHAPE\n";
            out << ind << "    NUM_VERTICES: " << nPts << "\n";
            for (int i = 0; i < nPts; ++i) {
                out << ind << "    V[" << i << "]: ";
                W_V(out, (double)pts[i].x, (double)pts[i].y, (double)pts[i].z);
                out << "\n";
            }

            UInt32 fillColor = 0;
            mdlElement_getFillColor(&fillColor, &edP->el);
            out << ind << "    FILL_COLOR: " << fillColor << "\n";

            double thickness = 0;
            DPoint3d dir;
            BoolInt capped = 0;
            memset(&dir, 0, sizeof(dir));
            if (SUCCESS == mdlLinkage_extractThickness(&thickness, &dir, &capped, &edP->el)) {
                out << ind << "    THICKNESS: " << thickness
                    << " capped=" << capped
                    << " dir=(" << (double)dir.x << ", " << (double)dir.y << ", " << (double)dir.z << ")\n";
            }
            out << ind << "  }\n";
        }
        else {
            out << ind << "  (shape extraction failed)\n";
        }
    }

    void DumpCurve(MSElementDescr* edP, std::ostream& out, std::string const& ind)
    {
        out << ind << "  GEOMETRY {\n";
        out << ind << "    KIND: CURVE\n";

        std::vector<DPoint3d> pts(MAX_VERTICES);
        int nPts = 0;
        if (SUCCESS == mdlLinear_extract(&pts[0], &nPts, &edP->el, NULL) && nPts > 0) {
            if (nPts > MAX_VERTICES) nPts = MAX_VERTICES;
            out << ind << "    NUM_VERTICES: " << nPts << "\n";
            for (int i = 0; i < nPts; ++i) {
                out << ind << "    V[" << i << "]: ";
                W_V(out, (double)pts[i].x, (double)pts[i].y, (double)pts[i].z);
                out << "\n";
            }

            if (nPts >= 2) {
                double dx = (double)pts[1].x - (double)pts[0].x;
                double dy = (double)pts[1].y - (double)pts[0].y;
                double dz = (double)pts[1].z - (double)pts[0].z;
                double len = sqrt(dx * dx + dy * dy + dz * dz);
                if (len > 1e-15) {
                    out << ind << "    START_TANGENT_APPROX: ";
                    W_V(out, dx / len, dy / len, dz / len);
                    out << "\n";
                }
                dx = (double)pts[nPts - 1].x - (double)pts[nPts - 2].x;
                dy = (double)pts[nPts - 1].y - (double)pts[nPts - 2].y;
                dz = (double)pts[nPts - 1].z - (double)pts[nPts - 2].z;
                len = sqrt(dx * dx + dy * dy + dz * dz);
                if (len > 1e-15) {
                    out << ind << "    END_TANGENT_APPROX: ";
                    W_V(out, dx / len, dy / len, dz / len);
                    out << "\n";
                }
            }
        }
        out << ind << "  }\n";
    }

    void DumpConic(MSElementDescr* edP, std::ostream& out, std::string const& ind)
    {
        out << ind << "  GEOMETRY {\n";
        out << ind << "    KIND: CONIC\n";
        std::vector<DPoint3d> pts(MAX_VERTICES);
        int nPts = 0;
        if (SUCCESS == mdlLinear_extract(&pts[0], &nPts, &edP->el, NULL) && nPts > 0) {
            if (nPts > MAX_VERTICES) nPts = MAX_VERTICES;
            out << ind << "    NUM_VERTICES: " << nPts << "\n";
            for (int i = 0; i < nPts; ++i) {
                out << ind << "    V[" << i << "]: ";
                W_V(out, (double)pts[i].x, (double)pts[i].y, (double)pts[i].z);
                out << "\n";
            }
        }
        out << ind << "  }\n";
    }

    void DumpBsplinePole(MSElementDescr* edP, std::ostream& out, std::string const& ind)
    {
        out << ind << "  GEOMETRY {\n";
        out << ind << "    KIND: BSPLINE_POLE\n";
        if (IsElement3D(edP)) {
            Bspline_pole_3d const* bp = reinterpret_cast<Bspline_pole_3d const*>(&edP->el);
            out << ind << "    NUM_POLES: " << bp->numpoles << "\n";
            for (UInt32 i = 0; i < bp->numpoles; ++i) {
                out << ind << "    P[" << i << "]: ";
                W_V(out, (double)bp->poles[i].x, (double)bp->poles[i].y, (double)bp->poles[i].z);
                out << "\n";
            }
        }
        else {
            Bspline_pole_2d const* bp = reinterpret_cast<Bspline_pole_2d const*>(&edP->el);
            out << ind << "    NUM_POLES: " << bp->numpoles << "\n";
            for (UInt32 i = 0; i < bp->numpoles; ++i) {
                out << ind << "    P[" << i << "]: ";
                W_V(out, (double)bp->poles[i].x, (double)bp->poles[i].y, 0.0);
                out << "\n";
            }
        }
        out << ind << "  }\n";
    }

    void DumpArc(MSElementDescr* edP, std::ostream& out, std::string const& ind)
    {
        DPoint3d startEndPts[2];
        double start = 0, sweep = 0, axis1 = 0, axis2 = 0;
        RotMatrix rm; DPoint3d center;
        if (SUCCESS != mdlArc_extract(startEndPts, &start, &sweep,
            &axis1, &axis2, &rm, &center, &edP->el)) {
            out << ind << "  (arc extraction failed)\n";
            return;
        }
        out << ind << "  GEOMETRY {\n";
        out << ind << "    KIND: ARC\n";
        out << ind << "    CENTER: ";
        W_V(out, (double)center.x, (double)center.y, (double)center.z);
        out << "\n";
        out << ind << "    PRIMARY_AXIS: " << axis1 << "\n";
        out << ind << "    SECONDARY_AXIS: " << axis2 << "\n";
        out << ind << "    START_ANGLE: " << start << "\n";
        out << ind << "    SWEEP_ANGLE: " << sweep << "\n";
        out << ind << "    DIRECTION: " << (sweep >= 0 ? "CCW" : "CW") << "\n";
        out << ind << "    START: ";
        W_V(out, (double)startEndPts[0].x, (double)startEndPts[0].y, (double)startEndPts[0].z);
        out << "\n";
        out << ind << "    END:   ";
        W_V(out, (double)startEndPts[1].x, (double)startEndPts[1].y, (double)startEndPts[1].z);
        out << "\n";
        out << ind << "    ROTATION: [[" << rm.form3d[0][0] << "," << rm.form3d[0][1] << "," << rm.form3d[0][2] << "],"
            << "[" << rm.form3d[1][0] << "," << rm.form3d[1][1] << "," << rm.form3d[1][2] << "],"
            << "[" << rm.form3d[2][0] << "," << rm.form3d[2][1] << "," << rm.form3d[2][2] << "]]\n";
        out << ind << "    NORMAL: ";
        W_V(out, rm.form3d[2][0], rm.form3d[2][1], rm.form3d[2][2]);
        out << "\n";
        out << ind << "  }\n";
    }

    void DumpEllipse(MSElementDescr* edP, std::ostream& out, std::string const& ind)
    {
        DPoint3d startEndPts[2];
        double start = 0, sweep = 0, axis1 = 0, axis2 = 0;
        RotMatrix rm; DPoint3d center;
        if (SUCCESS != mdlArc_extract(startEndPts, &start, &sweep,
            &axis1, &axis2, &rm, &center, &edP->el)) {
            out << ind << "  (ellipse extraction failed)\n";
            return;
        }
        out << ind << "  GEOMETRY {\n";
        out << ind << "    KIND: ELLIPSE\n";
        out << ind << "    CENTER: ";
        W_V(out, (double)center.x, (double)center.y, (double)center.z);
        out << "\n";
        out << ind << "    PRIMARY_AXIS: " << axis1 << "\n";
        out << ind << "    SECONDARY_AXIS: " << axis2 << "\n";
        out << ind << "    START_ANGLE: " << start << "\n";
        out << ind << "    SWEEP_ANGLE: " << sweep << "  (ellipse sweep is 2*PI)\n";
        out << ind << "    ROTATION: [[" << rm.form3d[0][0] << "," << rm.form3d[0][1] << "," << rm.form3d[0][2] << "],"
            << "[" << rm.form3d[1][0] << "," << rm.form3d[1][1] << "," << rm.form3d[1][2] << "],"
            << "[" << rm.form3d[2][0] << "," << rm.form3d[2][1] << "," << rm.form3d[2][2] << "]]\n";
        out << ind << "    NORMAL: ";
        W_V(out, rm.form3d[2][0], rm.form3d[2][1], rm.form3d[2][2]);
        out << "\n";
        out << ind << "  }\n";
    }

    void DumpCone(MSElementDescr* edP, std::ostream& out, std::string const& ind)
    {
        double topR = 0, botR = 0;
        DPoint3d topC, botC;
        RotMatrix rm;
        if (SUCCESS != mdlCone_extract(&topR, &botR, &topC, &botC, &rm, &edP->el)) {
            out << ind << "  (cone extraction failed)\n";
            return;
        }
        Cone_3d const* cone = reinterpret_cast<Cone_3d const*>(&edP->el);
        out << ind << "  GEOMETRY {\n";
        out << ind << "    KIND: CONE\n";
        out << ind << "    CONE_TYPE: " << cone->b.type << "\n";
        out << ind << "    IS_SURFACE: " << cone->b.surf << "\n";
        out << ind << "    TOP_RADIUS: " << topR << "\n";
        out << ind << "    BOTTOM_RADIUS: " << botR << "\n";
        out << ind << "    TOP_CENTER: ";
        W_V(out, (double)topC.x, (double)topC.y, (double)topC.z);
        out << "\n";
        out << ind << "    BOTTOM_CENTER: ";
        W_V(out, (double)botC.x, (double)botC.y, (double)botC.z);
        out << "\n";
        double hx = (double)topC.x - (double)botC.x;
        double hy = (double)topC.y - (double)botC.y;
        double hz = (double)topC.z - (double)botC.z;
        double height = sqrt(hx * hx + hy * hy + hz * hz);
        out << ind << "    HEIGHT: " << height << "\n";
        out << ind << "    AXIS: ";
        if (height > 1e-15) W_V(out, hx / height, hy / height, hz / height);
        else W_V(out, 0, 0, 1);
        out << "\n";
        out << ind << "    ROTATION: [[" << rm.form3d[0][0] << "," << rm.form3d[0][1] << "," << rm.form3d[0][2] << "],"
            << "[" << rm.form3d[1][0] << "," << rm.form3d[1][1] << "," << rm.form3d[1][2] << "],"
            << "[" << rm.form3d[2][0] << "," << rm.form3d[2][1] << "," << rm.form3d[2][2] << "]]\n";
        out << ind << "  }\n";
    }

    void DumpText(MSElementDescr* edP, std::ostream& out, std::string const& ind)
    {
        DPoint3d origin = { 0,0,0 }, userOrigin = { 0,0,0 };
        int edfields = 0;
        char str[4096] = { 0 };
        RotMatrix rm;
        TextStyleInfo tfs;
        memset(&tfs, 0, sizeof(tfs));
        int just = 0;
        MSTextSize tileSize, textSize;
        memset(&tileSize, 0, sizeof(tileSize));
        memset(&textSize, 0, sizeof(textSize));

        if (SUCCESS != mdlText_extract(&origin, &userOrigin, &edfields, NULL,
            str, &rm, &tfs, &just,
            &tileSize, &textSize, &edP->el)) {
            out << ind << "  (text extraction failed)\n";
            return;
        }
        out << ind << "  GEOMETRY {\n";
        out << ind << "    KIND: TEXT\n";
        out << ind << "    ORIGIN: ";
        W_V(out, (double)origin.x, (double)origin.y, (double)origin.z);
        out << "\n";
        out << ind << "    USER_ORIGIN: ";
        W_V(out, (double)userOrigin.x, (double)userOrigin.y, (double)userOrigin.z);
        out << "\n";
        out << ind << "    JUSTIFICATION: " << just << "\n";
        out << ind << "    ED_FIELDS: " << edfields << "\n";
        out << ind << "    TEXT: \"" << str << "\"\n";
        out << ind << "    TILE_SIZE: width=" << tileSize.width
            << " height=" << tileSize.height << "\n";
        out << ind << "    TEXT_SIZE: width=" << textSize.width
            << " height=" << textSize.height << "\n";
        out << ind << "    ROTATION: [[" << rm.form3d[0][0] << "," << rm.form3d[0][1] << "," << rm.form3d[0][2] << "],"
            << "[" << rm.form3d[1][0] << "," << rm.form3d[1][1] << "," << rm.form3d[1][2] << "],"
            << "[" << rm.form3d[2][0] << "," << rm.form3d[2][1] << "," << rm.form3d[2][2] << "]]\n";
        out << ind << "    FONT: " << tfs.font << "\n";
        out << ind << "    STYLE: " << tfs.style << "\n";
        out << ind << "  }\n";
    }

    void DumpTextNode(MSElementDescr* edP, std::ostream& out, std::string const& ind)
    {
        DPoint3d userOrigin = { 0,0,0 };
        RotMatrix rm;
        MSTextSize tileSize;
        memset(&tileSize, 0, sizeof(tileSize));
        double lineSpacing = 0;
        TextParam txtParams;
        memset(&txtParams, 0, sizeof(txtParams));
        int nodeNumber = 0;

        if (SUCCESS != mdlTextNode_extract(&userOrigin, &rm, &tileSize,
            &lineSpacing, &txtParams,
            &nodeNumber, &edP->el, NULL)) {
            out << ind << "  (text node extraction failed)\n";
            return;
        }
        out << ind << "  GEOMETRY {\n";
        out << ind << "    KIND: TEXT_NODE\n";
        out << ind << "    ORIGIN: ";
        W_V(out, (double)userOrigin.x, (double)userOrigin.y, (double)userOrigin.z);
        out << "\n";
        out << ind << "    NODE_NUMBER: " << nodeNumber << "\n";
        out << ind << "    LINE_SPACING: " << lineSpacing << "\n";
        out << ind << "    TILE_SIZE: width=" << tileSize.width
            << " height=" << tileSize.height << "\n";
        out << ind << "    ROTATION: [[" << rm.form3d[0][0] << "," << rm.form3d[0][1] << "," << rm.form3d[0][2] << "],"
            << "[" << rm.form3d[1][0] << "," << rm.form3d[1][1] << "," << rm.form3d[1][2] << "],"
            << "[" << rm.form3d[2][0] << "," << rm.form3d[2][1] << "," << rm.form3d[2][2] << "]]\n";
        out << ind << "    FONT: " << txtParams.font << "\n";
        out << ind << "    JUSTIFICATION: " << txtParams.just << "\n";
        out << ind << "  }\n";
    }

    void DumpBsplineCurve(MSElementDescr* edP, std::ostream& out, std::string const& ind)
    {
        MSBsplineCurve curve;
        memset(&curve, 0, sizeof(curve));
        if (SUCCESS != mdlBspline_convertToCurve(&curve, edP)) {
            out << ind << "  (bspline curve conversion failed)\n";
            return;
        }
        out << ind << "  GEOMETRY {\n";
        out << ind << "    KIND: BSPLINE_CURVE\n";
        out << ind << "    ORDER: " << (int)curve.params.order << "\n";
        out << ind << "    NUM_POLES: " << (int)curve.params.numPoles << "\n";
        out << ind << "    RATIONAL: " << (curve.weights ? 1 : 0) << "\n";
        out << ind << "    CLOSED: " << (curve.params.closed ? 1 : 0) << "\n";

        Bspline_curve const* bc = reinterpret_cast<Bspline_curve const*>(&edP->el);
        Bspline_flags f = bc->flags;
        out << ind << "    CURVE_TYPE: " << f.curve_type << "\n";
        out << ind << "    CONSTRUCT_TYPE: " << f.construct_type << "\n";
        out << ind << "    CURVE_DISPLAY: " << f.curve_display << "\n";
        out << ind << "    POLY_DISPLAY: " << f.poly_display << "\n";
        out << ind << "    RATIONAL_FLAG: " << f.rational << "\n";
        out << ind << "    CLOSED_FLAG: " << f.closed << "\n";
        out << ind << "    NATURAL_TANGENTS: " << f.isNaturalTangents << "\n";
        out << ind << "    CHORDLEN_TANGENTS: " << f.isChordLenTangents << "\n";
        out << ind << "    COLINEAR_TANGENTS: " << f.isColinearTangents << "\n";
        out << ind << "    CHORDLEN_KNOTS: " << f.isChordLenKnots << "\n";
        out << ind << "    ZERO_START_TANGENT: " << f.isZeroStartTangent << "\n";
        out << ind << "    ZERO_END_TANGENT: " << f.isZeroEndTangent << "\n";
        out << ind << "    STORE_FULL_KNOTS: " << f.storeFullKnots << "\n";

        out << ind << "    POLES:\n";
        for (int i = 0; i < (int)curve.params.numPoles; ++i) {
            double w = (curve.weights) ? curve.weights[i] : 1.0;
            out << ind << "      P[" << i << "]: ";
            W_P(out, (double)curve.poles[i].x, (double)curve.poles[i].y, (double)curve.poles[i].z, w);
            out << "\n";
        }
        int nK = mdlBspline_numberKnots(curve.params.numPoles,
            curve.params.order,
            curve.params.closed);
        out << ind << "    KNOTS (" << nK << "):\n";
        for (int i = 0; i < nK && curve.knots; ++i)
            out << ind << "      K[" << i << "]: " << curve.knots[i] << "\n";
        out << ind << "  }\n";
        mdlBspline_freeCurve(&curve);
    }

    void DumpBsplineSurface(MSElementDescr* edP, std::ostream& out, std::string const& ind)
    {
        MSBsplineSurface surf;
        memset(&surf, 0, sizeof(surf));
        if (SUCCESS != mdlBspline_convertToSurface(&surf, edP)) {
            out << ind << "  (bspline surface conversion failed)\n";
            return;
        }
        out << ind << "  GEOMETRY {\n";
        out << ind << "    KIND: BSPLINE_SURFACE\n";
        out << ind << "    U_ORDER: " << (int)surf.uParams.order << "\n";
        out << ind << "    V_ORDER: " << (int)surf.vParams.order << "\n";
        out << ind << "    U_NUM_POLES: " << (int)surf.uParams.numPoles << "\n";
        out << ind << "    V_NUM_POLES: " << (int)surf.vParams.numPoles << "\n";
        out << ind << "    U_CLOSED: " << (surf.uParams.closed ? 1 : 0) << "\n";
        out << ind << "    V_CLOSED: " << (surf.vParams.closed ? 1 : 0) << "\n";
        out << ind << "    RATIONAL: " << (surf.weights ? 1 : 0) << "\n";

        Bspline_surface const* bs = reinterpret_cast<Bspline_surface const*>(&edP->el);
        out << ind << "    ARC_SPACING: " << bs->bsurf_flags.arcSpacing << "\n";
        out << ind << "    STORE_FULL_KNOTS: " << bs->bsurf_flags.storeFullKnots << "\n";
        out << ind << "    NUM_BOUNDS: " << bs->num_bounds << "\n";

        int total = (int)(surf.uParams.numPoles * surf.vParams.numPoles);
        out << ind << "    POLES:\n";
        for (int i = 0; i < total; ++i) {
            double w = (surf.weights) ? surf.weights[i] : 1.0;
            out << ind << "      P[" << i << "]: ";
            W_P(out, (double)surf.poles[i].x, (double)surf.poles[i].y, (double)surf.poles[i].z, w);
            out << "\n";
        }
        int uK = mdlBspline_numberKnots(surf.uParams.numPoles,
            surf.uParams.order,
            surf.uParams.closed);
        int vK = mdlBspline_numberKnots(surf.vParams.numPoles,
            surf.vParams.order,
            surf.vParams.closed);
        out << ind << "    U_KNOTS (" << uK << "):\n";
        for (int i = 0; i < uK && surf.uKnots; ++i)
            out << ind << "      K[" << i << "]: " << surf.uKnots[i] << "\n";
        out << ind << "    V_KNOTS (" << vK << "):\n";
        for (int i = 0; i < vK && surf.vKnots; ++i)
            out << ind << "      K[" << i << "]: " << surf.vKnots[i] << "\n";
        out << ind << "  }\n";
        mdlBspline_freeSurface(&surf);
    }

    void DumpBsplineKnot(MSElementDescr* edP, std::ostream& out, std::string const& ind)
    {
        out << ind << "  GEOMETRY {\n";
        out << ind << "    KIND: BSPLINE_KNOT\n";
        Bspline_knot const* bk = reinterpret_cast<Bspline_knot const*>(&edP->el);
        int nWords = (int)edP->el.ehdr.elementSize - (int)(sizeof(Elm_hdr) / sizeof(Int16));
        int nKnots = nWords / (int)(sizeof(double) / sizeof(Int16));
        out << ind << "    NUM_KNOTS: " << nKnots << "\n";
        for (int i = 0; i < nKnots; ++i) {
            out << ind << "    K[" << i << "]: " << bk->knots[i] << "\n";
        }
        out << ind << "  }\n";
    }

    void DumpBsplineWeight(MSElementDescr* edP, std::ostream& out, std::string const& ind)
    {
        out << ind << "  GEOMETRY {\n";
        out << ind << "    KIND: BSPLINE_WEIGHT\n";
        Bspline_weight const* bw = reinterpret_cast<Bspline_weight const*>(&edP->el);
        int nWords = (int)edP->el.ehdr.elementSize - (int)(sizeof(Elm_hdr) / sizeof(Int16));
        int nW = nWords / (int)(sizeof(double) / sizeof(Int16));
        out << ind << "    NUM_WEIGHTS: " << nW << "\n";
        for (int i = 0; i < nW; ++i) {
            out << ind << "    W[" << i << "]: " << bw->weights[i] << "\n";
        }
        out << ind << "  }\n";
    }

    void DumpBsurfBoundary(MSElementDescr* edP, std::ostream& out, std::string const& ind)
    {
        out << ind << "  GEOMETRY {\n";
        out << ind << "    KIND: BSURF_BOUNDARY\n";
        Bsurf_boundary const* bb = reinterpret_cast<Bsurf_boundary const*>(&edP->el);
        out << ind << "    BOUNDARY_NUMBER: " << bb->number << "\n";
        out << ind << "    NUM_VERTS: " << bb->numverts << "\n";
        for (UInt32 i = 0; i < bb->numverts; ++i) {
            out << ind << "    V[" << i << "]: ("
                << (double)bb->vertices[i].x << ", " << (double)bb->vertices[i].y << ")\n";
        }
        out << ind << "  }\n";
    }

    bool DumpBrep(MSElementDescr* edP, DgnModelRefP modelRef,
        std::ostream& out, std::string const& ind)
    {
        KIBODY* body = NULL;
        Transform xf;
        if (!SafeElementToBody(edP, modelRef, &body, &xf) || !body)
            return false;

        RawSolidBody sb;
        bool ok = SafeBodyToRawSolid(body, xf, sb, out);
        mdlKISolid_freeBody(body);

        if (!ok || sb.faces.empty()) return false;

        out << ind << "  GEOMETRY {\n";
        out << ind << "    KIND: BREP\n";
        WriteSolidBody(sb, out, ind);
        out << ind << "  }\n";
        return true;
    }

    // -- Solid body writer ----------------------------------------
    static void WriteSolidBody(RawSolidBody const& sb, std::ostream& out,
        std::string const& ind)
    {
        std::string i2 = ind + "    ";
        std::string i3 = ind + "      ";

        out << i2 << "KERNEL: " << sb.kernel << "\n";
        out << i2 << "VERTEX_COUNT: " << sb.vertices.size() << "\n";
        out << i2 << "FACE_COUNT: " << sb.faces.size() << "\n";

        out << i2 << "VERTICES:\n";
        for (size_t i = 0; i < sb.vertices.size(); ++i) {
            out << i2 << "  V[" << i << "]: ";
            W_V(out, sb.vertices[i].x, sb.vertices[i].y, sb.vertices[i].z);
            out << "\n";
        }

        for (size_t f = 0; f < sb.faces.size(); ++f) {
            RawFaceData const& fd = sb.faces[f];
            out << i2 << "FACE[" << f << "] {\n";

            if (fd.surfaceType == 1) {
                out << i3 << "SURFACE: PLANE\n";
                out << i3 << "NORMAL: "; W_V(out, fd.normal.x, fd.normal.y, fd.normal.z); out << "\n";
                out << i3 << "ORIGIN: "; W_V(out, fd.origin.x, fd.origin.y, fd.origin.z); out << "\n";
            }
            else if (fd.surfaceType == 5) {
                out << i3 << "SURFACE: BSPLINE\n";
                out << i3 << "U_DEGREE: " << fd.bspline.uDegree << "\n";
                out << i3 << "V_DEGREE: " << fd.bspline.vDegree << "\n";
                out << i3 << "U_NUM_POLES: " << fd.bspline.uNumPoles << "\n";
                out << i3 << "V_NUM_POLES: " << fd.bspline.vNumPoles << "\n";
                out << i3 << "U_CLOSED: " << (fd.bspline.uClosed ? 1 : 0) << "\n";
                out << i3 << "V_CLOSED: " << (fd.bspline.vClosed ? 1 : 0) << "\n";
                out << i3 << "RATIONAL: " << (fd.bspline.rational ? 1 : 0) << "\n";
                out << i3 << "POLES:\n";
                for (size_t p = 0; p < fd.bspline.poles.size(); ++p) {
                    out << i3 << "  P[" << p << "]: ";
                    W_P(out, fd.bspline.poles[p].x, fd.bspline.poles[p].y,
                        fd.bspline.poles[p].z, fd.bspline.poles[p].w);
                    out << "\n";
                }
                out << i3 << "U_KNOTS:";
                for (size_t k = 0; k < fd.bspline.uKnots.size(); ++k)
                    out << " " << fd.bspline.uKnots[k];
                out << "\n";
                out << i3 << "V_KNOTS:";
                for (size_t k = 0; k < fd.bspline.vKnots.size(); ++k)
                    out << " " << fd.bspline.vKnots[k];
                out << "\n";
            }
            else {
                out << i3 << "SURFACE: TESSELLATED\n";
            }

            out << i3 << "VERTEX_LOOP:";
            for (size_t i = 0; i < fd.vertexLoop.size(); ++i)
                out << " " << fd.vertexLoop[i];
            out << "\n";

            out << i3 << "EDGE_COUNT: " << fd.edges.size() << "\n";
            for (size_t e = 0; e < fd.edges.size(); ++e) {
                RawEdgeData const& ed = fd.edges[e];
                out << i3 << "EDGE[" << e << "] { TYPE=" << ed.edgeType
                    << " CLOSED=" << (ed.closed ? 1 : 0)
                    << " REVERSED=" << (ed.reversed ? 1 : 0)
                    << " NATIVE_CURVE=" << ed.nativeCurveType
                    << " POINTS:";
                for (size_t k = 0; k < ed.pointIndices.size(); ++k)
                    out << " " << ed.pointIndices[k];
                out << " }\n";
            }

            out << i2 << "}\n";
        }
    }

    void DumpMesh(MSElementDescr* edP, std::ostream& out, std::string const& ind)
    {
        EmbeddedIntArray* pIdx = jmdlEmbeddedIntArray_grab();
        EmbeddedDPoint3dArray* pXYZ = jmdlEmbeddedDPoint3dArray_grab();
        if (!pIdx || !pXYZ) {
            if (pIdx) jmdlEmbeddedIntArray_drop(pIdx);
            if (pXYZ) jmdlEmbeddedDPoint3dArray_drop(pXYZ);
            return;
        }
        int numPerFace = 0, numFace = 0;
        if (SUCCESS != mdlMesh_getPolyfaceArrays(edP, pIdx, pXYZ, &numPerFace, &numFace)
            || numFace == 0) {
            jmdlEmbeddedIntArray_drop(pIdx);
            jmdlEmbeddedDPoint3dArray_drop(pXYZ);
            out << ind << "  (mesh extraction failed)\n";
            return;
        }

        RawSolidBody sb;
        sb.kernel = "Mesh";
        BrepExtractor dedup(sb);

        const int BUFSZ = 512;
        int      idxBuf[BUFSZ];
        BoolInt  flagBuf[BUFSZ];
        DPoint3d xyzBuf[BUFSZ];
        int readIdx = 0;

        Transform idXf;
        mdlTMatrix_getIdentity(&idXf);

        for (int f = 0; f < numFace; ++f) {
            int nPts = 0, nextRead = 0;
            if (SUCCESS != mdlMesh_getVariableSizePolyfaceFromOneBasedIndexedArrays(
                idxBuf, flagBuf, xyzBuf, &nPts, &nextRead, BUFSZ,
                pIdx, pXYZ, numPerFace, readIdx))
                break;
            readIdx = nextRead;
            if (nPts < 3) continue;

            RawFaceData fd;
            fd.surfaceType = 0;
            fd.vertexLoop.reserve(nPts);
            for (int k = 0; k < nPts; ++k) {
                RawVertex w = ApplyXf(xyzBuf[k], idXf);
                fd.vertexLoop.push_back(dedup.GetOrAddVertex(w));
            }
            fd.edges.reserve(nPts);
            for (int k = 0; k < nPts; ++k) {
                RawEdgeData ed;
                ed.edgeType = 0;
                ed.closed = false;
                ed.reversed = false;
                ed.nativeCurveType = 0;
                ed.pointIndices.push_back(fd.vertexLoop[k]);
                ed.pointIndices.push_back(fd.vertexLoop[(k + 1) % nPts]);
                fd.edges.push_back(ed);
            }
            sb.faces.push_back(fd);
        }
        jmdlEmbeddedIntArray_drop(pIdx);
        jmdlEmbeddedDPoint3dArray_drop(pXYZ);

        out << ind << "  GEOMETRY {\n";
        out << ind << "    KIND: MESH\n";
        out << ind << "    NUM_PER_FACE: " << numPerFace << "\n";
        WriteSolidBody(sb, out, ind);
        out << ind << "  }\n";
    }

    // -- Cell / shared cell ---------------------------------------
    void DumpCell(MSElementDescr* edP, std::ostream& out, std::string const& ind)
    {
        out << ind << "  CELL_DATA {\n";
        int typeNum = mdlElement_getType(&edP->el);
        if (typeNum == CELL_HEADER_ELM) {
            if (IsElement3D(edP)) {
                Cell_3d const* c = reinterpret_cast<Cell_3d const*>(&edP->el);
                MSWChar name[256] = { 0 };
                c->GetName(name, 256);
                char narrow[512] = { 0 };
                mdlCnv_convertUnicodeToMultibyte(name, -1, narrow, sizeof(narrow));
                out << ind << "    NAME: \"" << narrow << "\"\n";
                out << ind << "    COMPONENT_COUNT: " << c->componentCount << "\n";
                out << ind << "    ORIGIN: ";
                W_V(out, (double)c->origin.x, (double)c->origin.y, (double)c->origin.z);
                out << "\n";
                out << ind << "    RANGE_LOW: ";
                W_V(out, (double)c->rnglow.x, (double)c->rnglow.y, (double)c->rnglow.z);
                out << "\n";
                out << ind << "    RANGE_HIGH: ";
                W_V(out, (double)c->rnghigh.x, (double)c->rnghigh.y, (double)c->rnghigh.z);
                out << "\n";
                out << ind << "    TRANSFORM: [[" << c->transform[0][0] << "," << c->transform[0][1] << "," << c->transform[0][2] << "],"
                    << "[" << c->transform[1][0] << "," << c->transform[1][1] << "," << c->transform[1][2] << "],"
                    << "[" << c->transform[2][0] << "," << c->transform[2][1] << "," << c->transform[2][2] << "]]\n";
                out << ind << "    IS_ANNOTATION: " << c->flags.isAnnotation << "\n";
                out << ind << "    MATCH_ANN_SIZE: " << c->flags.matchSourceAnnotationSize << "\n";
                out << ind << "    MATCH_DIM_VALUE: " << c->flags.matchSourceDimValue << "\n";
                out << ind << "    MATCH_ML_OFFSET: " << c->flags.matchSourceMultilineOffset << "\n";
            }
            else {
                Cell_2d const* c = reinterpret_cast<Cell_2d const*>(&edP->el);
                MSWChar name[256] = { 0 };
                c->GetName(name, 256);
                char narrow[512] = { 0 };
                mdlCnv_convertUnicodeToMultibyte(name, -1, narrow, sizeof(narrow));
                out << ind << "    NAME: \"" << narrow << "\"\n";
                out << ind << "    COMPONENT_COUNT: " << c->componentCount << "\n";
                out << ind << "    ORIGIN: (" << (double)c->origin.x << ", " << (double)c->origin.y << ")\n";
                out << ind << "    RANGE_LOW: (" << (double)c->rnglow.x << ", " << (double)c->rnglow.y << ")\n";
                out << ind << "    RANGE_HIGH: (" << (double)c->rnghigh.x << ", " << (double)c->rnghigh.y << ")\n";
                out << ind << "    TRANSFORM: [[" << c->transform[0][0] << "," << c->transform[0][1] << "],"
                    << "[" << c->transform[1][0] << "," << c->transform[1][1] << "]]\n";
                out << ind << "    IS_ANNOTATION: " << c->flags.isAnnotation << "\n";
            }
        }
        else {
            out << ind << "    (shared cell / def — native struct not exposed)\n";
        }
        out << ind << "  }\n";
    }

    // -- Dimension ------------------------------------------------
    void DumpDimension(MSElementDescr* edP, std::ostream& out, std::string const& ind)
    {
        out << ind << "  DIMENSION_DATA {\n";
        MSElement* dim = &edP->el;

        int nSeg = mdlDim_getNumberOfSegments(dim);
        int nPts = mdlDim_getNumberOfPoints(dim);
        out << ind << "    NUM_SEGMENTS: " << nSeg << "\n";
        out << ind << "    NUM_POINTS: " << nPts << "\n";

        const int MAX_DIM_VALUES = 256; // достаточно для любых реальных измерений
        if (nSeg > 0 && nSeg <= MAX_DIM_VALUES) {
            double values[MAX_DIM_VALUES];
            memset(values, 0, sizeof(values));
            if (SUCCESS == mdlDim_getActualValues(dim, values)) {
                out << ind << "    ACTUAL_VALUES:";
                for (int i = 0; i < nSeg; ++i) out << " " << values[i];
                out << "\n";
            }
        }

        if (nPts > 0 && nPts <= 256) {
            DPoint3d pts[256];
            memset(pts, 0, sizeof(pts));
            int got = mdlDim_extractPoints(pts, dim, mdlModelRef_getActive(), 0, nPts);
            out << ind << "    DEF_POINTS: extracted=" << got << "\n";
            for (int i = 0; i < got; ++i) {
                out << ind << "      DP[" << i << "]: ("
                    << (double)pts[i].x << ", " << (double)pts[i].y << ", " << (double)pts[i].z << ")\n";
            }
        }

        RotMatrix rm;
        memset(&rm, 0, sizeof(rm));
        mdlDim_getRotMatrix(dim, &rm);
        out << ind << "    ROTATION: [[" << rm.form3d[0][0] << "," << rm.form3d[0][1] << "," << rm.form3d[0][2] << "],"
            << "[" << rm.form3d[1][0] << "," << rm.form3d[1][1] << "," << rm.form3d[1][2] << "],"
            << "[" << rm.form3d[2][0] << "," << rm.form3d[2][1] << "," << rm.form3d[2][2] << "]]\n";

        if (nSeg > 0) {
            DimStrings strs;
            DimStringConfig cfg;
            memset(&strs, 0, sizeof(strs));
            memset(&cfg, 0, sizeof(cfg));
            if (SUCCESS == mdlDim_getStrings(&strs, &cfg, dim, 0)) {
                out << ind << "    SEGMENT[0].PRIMARY_SINGLE: \"" << strs.primary.single << "\"\n";
                out << ind << "    SEGMENT[0].CONFIG: tolerance=" << cfg.tolerance
                    << " limit=" << cfg.limit << " dual=" << cfg.dual << "\n";
            }
        }

        MSWChar styleName[128] = { 0 };
        if (SUCCESS == mdlDim_getDimStyleName(styleName, 128, dim, mdlModelRef_getActive())) {
            char narrow[256] = { 0 };
            mdlCnv_convertUnicodeToMultibyte(styleName, -1, narrow, sizeof(narrow));
            out << ind << "    DIM_STYLE_NAME: \"" << narrow << "\"\n";
        }

        UInt32 textStyleID = 0;
        if (SUCCESS == mdlDim_getTextStyleID(&textStyleID, dim)) {
            out << ind << "    TEXT_STYLE_ID: " << textStyleID << "\n";
        }

        out << ind << "  }\n";
    }

    // -- Multiline ------------------------------------------------
    void DumpMultiline(MSElementDescr* edP, std::ostream& out, std::string const& ind)
    {
        out << ind << "  MULTILINE_DATA {\n";
        out << ind << "    (native mline extraction requires msmline.fdf; see mdlMline_extract)\n";
        out << ind << "  }\n";
    }

    // -- Raster ---------------------------------------------------
    void DumpRasterHdr(MSElementDescr* edP, std::ostream& out, std::string const& ind)
    {
        out << ind << "  GEOMETRY {\n";
        out << ind << "    KIND: RASTER\n";
        Raster_hdr const* rh = reinterpret_cast<Raster_hdr const*>(&edP->el);
        out << ind << "    X_EXTENT: " << rh->xextent << "\n";
        out << ind << "    Y_EXTENT: " << rh->yextent << "\n";
        out << ind << "    RESOLUTION: " << rh->resolution << "\n";
        out << ind << "    SCALE: " << rh->scale << "\n";
        out << ind << "    ORIGIN: ";
        W_V(out, (double)rh->origin.x, (double)rh->origin.y, (double)rh->origin.z);
        out << "\n";
        out << ind << "    NUM_VERTS: " << rh->numverts << "\n";
        out << ind << "    BACKGROUND: " << (int)rh->background << "\n";
        out << ind << "    FOREGROUND: " << (int)rh->foreground << "\n";
        out << ind << "    FLAGS: right=" << rh->flags.b.right
            << " lower=" << rh->flags.b.lower
            << " horizontal=" << rh->flags.b.horizontal
            << " format=" << rh->flags.b.format
            << " color=" << rh->flags.b.color
            << " transparent=" << rh->flags.b.transparent
            << " positive=" << rh->flags.b.positive << "\n";
        if (rh->numverts > 0) {
            for (UInt32 i = 0; i < rh->numverts; ++i) {
                out << ind << "    V[" << i << "]: ";
                if (IsElement3D(edP))
                    W_V(out, (double)rh->u.vert3d[i].x,
                        (double)rh->u.vert3d[i].y,
                        (double)rh->u.vert3d[i].z);
                else
                    W_V(out, (double)rh->u.vert2d[i].x,
                        (double)rh->u.vert2d[i].y, 0.0);
                out << "\n";
            }
        }
        out << ind << "  }\n";
    }

    void DumpRasterComp(MSElementDescr* edP, std::ostream& out, std::string const& ind)
    {
        out << ind << "  RASTER_COMP {\n";
        Raster_comp const* rc = reinterpret_cast<Raster_comp const*>(&edP->el);
        out << ind << "    X_OFFSET: " << rc->xoffset << "\n";
        out << ind << "    Y_OFFSET: " << rc->yoffset << "\n";
        out << ind << "    NUM_PIXELS: " << rc->numpixels << "\n";
        out << ind << "  }\n";
    }

    // -- Reference attach -----------------------------------------
    void DumpReferenceAttach(MSElementDescr* edP, std::ostream& out, std::string const& ind)
    {
        out << ind << "  REFERENCE_DATA {\n";
        ReferenceFileElm const* ref = reinterpret_cast<ReferenceFileElm const*>(&edP->el);
        out << ind << "    VERSION: " << ref->version << "\n";
        out << ind << "    GROUP_ID: " << ref->groupId << "\n";
        out << ind << "    FILE_NUMBER: " << ref->fileNumber << "\n";
        out << ind << "    PRIORITY: " << ref->priority << "\n";
        out << ind << "    NEST_DEPTH: " << ref->nestDepth << "\n";
        out << ind << "    SCALE: " << ref->scale << "\n";
        out << ind << "    REF_ORIGIN: ";
        W_V(out, (double)ref->refOrigin.x, (double)ref->refOrigin.y, (double)ref->refOrigin.z);
        out << "\n";
        out << ind << "    MASTER_ORIGIN: ";
        W_V(out, (double)ref->masterOrigin.x, (double)ref->masterOrigin.y, (double)ref->masterOrigin.z);
        out << "\n";
        out << ind << "    Z_FRONT: " << ref->zFront << "\n";
        out << ind << "    Z_BACK: " << ref->zBack << "\n";
        out << ind << "    N_CLIP_POINTS: " << ref->nClipPoints << "\n";
        out << ind << "    TRANSFORM: [[" << ref->transform.form3d[0][0] << "," << ref->transform.form3d[0][1] << "," << ref->transform.form3d[0][2] << "],"
            << "[" << ref->transform.form3d[1][0] << "," << ref->transform.form3d[1][1] << "," << ref->transform.form3d[1][2] << "],"
            << "[" << ref->transform.form3d[2][0] << "," << ref->transform.form3d[2][1] << "," << ref->transform.form3d[2][2] << "]]\n";
        if (ref->nClipPoints > 0) {
            for (UInt16 i = 0; i < ref->nClipPoints; ++i) {
                out << ind << "    CLIP[" << i << "]: ("
                    << (double)ref->clipPoints[i].x << ", "
                    << (double)ref->clipPoints[i].y << ")\n";
            }
        }
        out << ind << "  }\n";
    }

    // -- Attribute element ----------------------------------------
    void DumpAttributeElement(MSElementDescr* edP, std::ostream& out, std::string const& ind)
    {
        out << ind << "  ATTRIBUTE_ELEMENT {\n";
        short attrs[MAX_ATTRIBSIZE];
        int len = 0;
        memset(attrs, 0, sizeof(attrs));
        mdlElmdscr_extractAttributes(&len, attrs, edP);
        out << ind << "    LEN: " << len << " words\n";
        out << ind << "    DATA:";
        for (int i = 0; i < len && i < 32; ++i) out << " " << attrs[i];
        if (len > 32) out << " ...";
        out << "\n";
        out << ind << "  }\n";
    }
};

// ============================================================
// SCAN CALLBACK (native, OUTSIDE ref class)
// ============================================================

struct RawScanContext {
    RawGeometryExtractor* extractor;
    std::ostream* out;
    DgnModelRefP          modelRef;
    int                   processed;
};

static int __cdecl RawScanElmDscrCallback(MSElementDescr* edP, void* userArg,
    ScanCriteria* /*scP*/)
{
    RawScanContext* ctx = static_cast<RawScanContext*>(userArg);
    if (!ctx || !ctx->extractor || !edP) return SUCCESS;
    ctx->extractor->Extract(edP, ctx->modelRef, *ctx->out);
    ++ctx->processed;
    return SUCCESS;
}

// ============================================================
// NATIVE ENTRY POINT
// ============================================================

static void RunRawExportNative(bool selectedOnly)
{
    std::ofstream out(EXPORT_FILE_PATH, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        char msg[] = "Не удалось открыть файл экспорта.";
        mdlOutput_printf(MSG_ERROR, msg);
        return;
    }
    out << std::setprecision(15);

    out << "# RawGeometryExport — native MicroStation geometry dump\n";
    out << "# Every geometric element is dumped in its native form:\n";
    out << "#   - primitives   : native parameters (arcs, cones, text, …)\n";
    out << "#   - B-splines    : poles / knots / weights + flags\n";
    out << "#   - solids/sheets: B-rep topology + NURBS face surfaces\n";
    out << "#   - meshes       : vertices + face loops + edge flags\n";
    out << "#   - rasters      : header + component metadata\n";
    out << "#   - references   : transform + clipping\n";
    out << "#   - dimensions   : values, points, strings, rotation, style\n";
    out << "#   - attributes   : raw words + linkages (DMRS) + thickness + clip boundary\n";
    out << "#   - compound     : recursive component dump\n";
    out << "# Coordinates are in model coordinates (world/master).\n\n";

    RawGeometryExtractor ex;
    int processed = 0;

    if (selectedOnly) {
        int nSelected = mdlSelect_numSelected();
        out << "# mode = SELECTED, count = " << nSelected << "\n";
        if (nSelected == 0) {
            out << "# WARNING: selection set is empty\n";
        }

        for (int i = 0; i < nSelected; ++i) {
            ElementRef    elRef = NULL;
            DgnModelRefP  mref = NULL;
            if (SUCCESS != mdlSelect_getElement(i, &elRef, &mref)) {
                out << "\n# [WARN] mdlSelect_getElement(" << i << ") failed\n";
                continue;
            }
            if (!elRef) {
                out << "\n# [WARN] null ElementRef at index " << i << "\n";
                continue;
            }

            MSElementDescr* edP = NULL;
            UInt32 filePos = mdlElmdscr_getByElemRef(&edP, elRef, mref, TRUE, NULL);
            if (filePos == 0 || edP == NULL) {
                out << "\n# [WARN] mdlElmdscr_getByElemRef failed at index "
                    << i << " (filePos=" << filePos << ")\n";
                continue;
            }

            ex.Extract(edP, mref, out);
            mdlElmdscr_freeAll(&edP);
            ++processed;
        }
    }
    else {
        RawScanContext ctx;
        ctx.extractor = &ex;
        ctx.out = &out;
        ctx.modelRef = mdlModelRef_getActive();
        ctx.processed = 0;

        ScanCriteria* scP = mdlScanCriteria_create();
        if (!scP) {
            char msg[] = "mdlScanCriteria_create failed.";
            mdlOutput_printf(MSG_ERROR, msg);
            return;
        }
        mdlScanCriteria_setModel(scP, mdlModelRef_getActive());
        mdlScanCriteria_setElementCategory(scP, ELEMENT_CATEGORY_GRAPHICS);
        mdlScanCriteria_setReturnType(scP, MSSCANCRIT_ITERATE_ELMDSCR, FALSE, TRUE);
        mdlScanCriteria_setElmDscrCallback(scP,
            (PFScanElemDscrCallback)RawScanElmDscrCallback, &ctx);
        mdlScanCriteria_scan(scP, NULL, NULL, NULL);
        mdlScanCriteria_free(scP);
        processed = ctx.processed;
    }

    out << "\n# === SUMMARY ===\n";
    out << "# elements_processed = " << processed << "\n";
    out.close();

    char msg[256];
    sprintf_s(msg, sizeof(msg),
        "Raw geometry exported to %s (%d elements)",
        EXPORT_FILE_PATH, processed);
    mdlOutput_printf(MSG_MESSAGE, msg);
}

// ============================================================
// MANAGED WRAPPER (thin facade)
// ============================================================

namespace RawGeometryExport
{
    public ref class Export
    {
    public:
        static void WholeModel(System::String^ unused)
        {
            (void)unused;
            RunRawExportNative(false);
        }

        static void Selected(System::String^ unused)
        {
            (void)unused;
            if (mdlSelect_numSelected() == 0) {
                char msg[] = "Ничего не выбрано.";
                mdlOutput_printf(MSG_ERROR, msg);
                return;
            }
            RunRawExportNative(true);
        }
    };
}

#pragma warning(pop)
