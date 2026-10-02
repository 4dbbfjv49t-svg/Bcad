// Bcad geometry kernel over OpenCascade. See BcadKernel.h.
#include "BcadKernel.h"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_GTransform.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepFill.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepGProp.hxx>
#include <BRepLProp_SLProps.hxx>
#include <BRepLib.hxx>
#include <BRepLib_ToolTriangulatedShape.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeHalfSpace.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepPrimAPI_MakeWedge.hxx>
#include <BRepOffset_MakeOffset.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <Extrema_ExtPC.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <GProp_GProps.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom2d_Line.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>
#include <Poly_Triangulation.hxx>
#include <STEPControl_Writer.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Circ.hxx>
#include <gp_Elips.hxx>
#include <gp_GTrsf.hxx>
#include <gp_Pln.hxx>
#include <gp_Trsf.hxx>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

struct BKShape {
  TopoDS_Shape shape;
};

static thread_local std::string lastError;

static BKShape *wrap(const TopoDS_Shape &s) {
  if (s.IsNull()) return nullptr;
  return new BKShape{s};
}

template <typename F> static BKShape *guarded(const char *what, F f) {
  try {
    TopoDS_Shape s = f();
    if (s.IsNull()) {
      lastError = std::string(what) + " failed";
      return nullptr;
    }
    return wrap(s);
  } catch (Standard_Failure &e) {
    lastError = std::string(what) + ": " + (e.GetMessageString() ? e.GetMessageString() : "OpenCascade error");
  } catch (...) {
    lastError = std::string(what) + " failed";
  }
  return nullptr;
}

const char *bk_last_error(void) { return lastError.c_str(); }

// Input OpenCascade can't take is refused up front: it crashes on some degenerate sizes and never returns on NaN or infinity.
static void need(bool ok, const char *what) {
  if (!ok) throw Standard_Failure(what);
}

static bool finite(const double *v, int n) {
  for (int i = 0; i < n; i++)
    if (!std::isfinite(v[i])) return false;
  return true;
}

// MARK: - helpers

static TopoDS_Shape centred(const TopoDS_Shape &s) {
  Bnd_Box box;
  BRepBndLib::AddOptimal(s, box, false, false);
  if (box.IsVoid()) return s;
  double x0, y0, z0, x1, y1, z1;
  box.Get(x0, y0, z0, x1, y1, z1);
  gp_Trsf t;
  t.SetTranslation(gp_Vec(-(x0 + x1) / 2, -(y0 + y1) / 2, -(z0 + z1) / 2));
  return BRepBuilderAPI_Transform(s, t, true).Shape();
}

static TopoDS_Wire polygon(const std::vector<gp_Pnt> &pts) {
  BRepBuilderAPI_MakePolygon poly;
  for (const auto &p : pts) poly.Add(p);
  poly.Close();
  return poly.Wire();
}

// Regular n-gon in the XY plane at height z, corners on a circle of radius r, one flat facing -y.
static std::vector<gp_Pnt> ngon(int n, double r, double z) {
  std::vector<gp_Pnt> pts;
  double start = -M_PI / 2 + M_PI / n;
  for (int i = 0; i < n; i++) {
    double a = start + 2 * M_PI * i / n;
    pts.emplace_back(r * cos(a), r * sin(a), z);
  }
  return pts;
}

static TopoDS_Shape prism(int n, double r, double z0, double h) {
  TopoDS_Face base = BRepBuilderAPI_MakeFace(polygon(ngon(n, r, z0))).Face();
  return BRepPrimAPI_MakePrism(base, gp_Vec(0, 0, h)).Shape();
}

// Solid of revolution around Z from an (r, z) outline in the XZ plane.
static TopoDS_Shape revolve(const std::vector<std::pair<double, double>> &rz) {
  std::vector<gp_Pnt> pts;
  for (auto &p : rz) pts.emplace_back(p.first, 0, p.second);
  TopoDS_Face face = BRepBuilderAPI_MakeFace(polygon(pts)).Face();
  return BRepPrimAPI_MakeRevol(face, gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1))).Shape();
}

// A polygon tube's outline across its ring, in the (radius, height) plane around radius r: a triangle with its point up
// (sides 3) or a hexagon lying flat (6), both 2w wide and w·√3 tall.
static std::vector<std::pair<double, double>> tubeOutline(int sides, double w, double r) {
  double h = w * sqrt(3.0) / 2;
  if (sides == 3) return {{r - w, -h}, {r + w, -h}, {r, h}};
  return {{r - w, 0}, {r - w / 2, -h}, {r + w / 2, -h}, {r + w, 0}, {r + w / 2, h}, {r - w / 2, h}};
}

// The ellipse with conjugate semi-diameters u = (a, 0) and v = b·(cos t, sin t) is [u v] applied to the unit circle;
// its semi-axes are the square roots of the eigenvalues of [u v][u v]ᵀ, along the eigenvectors (phi: the major axis).
static void conjugate(double a, double b, double t, double &major, double &minor, double &phi) {
  double sxx = a * a + b * b * cos(t) * cos(t), sxy = b * b * cos(t) * sin(t), syy = b * b * sin(t) * sin(t);
  double mean = (sxx + syy) / 2, spread = hypot((sxx - syy) / 2, sxy);
  major = sqrt(mean + spread), minor = sqrt(std::max(mean - spread, 0.0)), phi = atan2(2 * sxy, sxx - syy) / 2;
}

static TopoDS_Edge oval(double major, double minor, double phi) {
  gp_Ax2 axes(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1), gp_Dir(cos(phi), sin(phi), 0));
  return major - minor <= 1e-9 * major ? BRepBuilderAPI_MakeEdge(gp_Circ(axes, major)).Edge()
                                       : BRepBuilderAPI_MakeEdge(gp_Elips(axes, major, minor)).Edge();
}

// Merges faces and edges that lie on the same surface (no seams left from the parts) and returns a lone
// solid as itself rather than wrapped in a compound, so a merge is one monolithic part.
static TopoDS_Shape unify(const TopoDS_Shape &s) {
  ShapeUpgrade_UnifySameDomain u(s, true, true, false);
  u.Build();
  TopoDS_Shape out = u.Shape();
  TopoDS_Shape solid;
  int solids = 0;
  for (TopExp_Explorer ex(out, TopAbs_SOLID); ex.More(); ex.Next()) solid = ex.Current(), solids++;
  return solids == 1 ? solid : out;
}

static int solidCount(const TopoDS_Shape &s) {
  int n = 0;
  for (TopExp_Explorer ex(s, TopAbs_SOLID); ex.More(); ex.Next()) n++;
  return n;
}

// A malloc'ed copy of v (room for at least one element), for arrays handed to the caller.
template <typename T> static T *mallocCopy(const std::vector<T> &v) {
  T *out = (T *)malloc(sizeof(T) * std::max<size_t>(1, v.size()));
  if (!v.empty()) memcpy(out, v.data(), sizeof(T) * v.size());
  return out;
}

static TopoDS_Shape fuse(const TopoDS_Shape &a, const TopoDS_Shape &b) {
  BRepAlgoAPI_Fuse op(a, b);
  if (!op.IsDone()) throw Standard_Failure("merge failed");
  return op.Shape();
}
static TopoDS_Shape cut(const TopoDS_Shape &a, const TopoDS_Shape &b) {
  BRepAlgoAPI_Cut op(a, b);
  if (!op.IsDone()) throw Standard_Failure("subtract failed");
  return op.Shape();
}
static TopoDS_Shape common(const TopoDS_Shape &a, const TopoDS_Shape &b) {
  BRepAlgoAPI_Common op(a, b);
  if (!op.IsDone()) throw Standard_Failure("intersect failed");
  return op.Shape();
}

// MARK: - primitives

BKShape *bk_primitive(int kind, const double *p) {
  return guarded("shape", [&]() -> TopoDS_Shape {
    // How many numbers each kind takes, and which must be above zero (a cone end and a ring's hole may be 0).
    static const int counts[] = {3, 2, 3, 1, 3, 3, 3, 3, 1, 2, 3, 4, 4, 5};
    static const std::vector<int> above[] = {{0, 1, 2}, {0, 1}, {2}, {0}, {1, 2}, {1, 2}, {0, 1, 2}, {1, 2}, {0}, {0, 1}, {0, 2}, {0, 1, 2, 3}, {0, 1, 3}, {1, 2, 4}};
    need(kind >= BK_BOX && kind <= BK_OVAL_TORUS, "unknown shape");
    need(finite(p, counts[kind]), "sizes must be numbers");
    for (int i = 0; i < counts[kind]; i++) need(p[i] >= 0, "sizes can't be below zero");
    for (int i : above[kind]) need(p[i] >= 0.001, "sizes must be above zero");
    need(kind != BK_CONE || std::max(p[0], p[1]) >= 0.001, "a cone needs one end wider than zero");
    switch (kind) {
    case BK_BOX:
      return centred(BRepPrimAPI_MakeBox(p[0], p[1], p[2]).Shape());
    case BK_CYLINDER:
      return centred(BRepPrimAPI_MakeCylinder(p[0] / 2, p[1]).Shape());
    case BK_CONE: {
      double r1 = p[0] / 2, r2 = p[1] / 2;
      if (fabs(r1 - r2) < 1e-6) return centred(BRepPrimAPI_MakeCylinder(r1, p[2]).Shape());
      return centred(BRepPrimAPI_MakeCone(r1, r2, p[2]).Shape());
    }
    case BK_SPHERE:
      return BRepPrimAPI_MakeSphere(p[0] / 2).Shape();
    case BK_PRISM:
      return centred(prism(std::max(3, (int)lround(p[0])), p[1] / 2, 0, p[2]));
    case BK_TORUS: {
      int sides = (int)lround(p[0]);
      need(sides == 0 || sides == 3 || sides == 6, "unknown tube shape");
      double tube = p[2] / 2, ring = p[1] / 2 - tube;
      if (sides == 0) {
        need(ring > tube * 0.05, "the tube is too thick for this torus");
        return BRepPrimAPI_MakeTorus(ring, tube).Shape();
      }
      // A polygon tube keeps a hole: crossing the axis would make the turned outline overlap itself.
      need(ring - tube > tube * 0.05, "the tube is too thick for this torus");
      return centred(revolve(tubeOutline(sides, tube, ring)));
    }
    case BK_WEDGE:
      return centred(BRepPrimAPI_MakeWedge(p[0], p[1], p[2], 0).Shape());
    case BK_PYRAMID: {
      int n = std::max(3, (int)lround(p[0]));
      BRepOffsetAPI_ThruSections loft(true, true);
      loft.AddWire(polygon(ngon(n, p[1] / 2, 0)));
      loft.AddVertex(BRepBuilderAPI_MakeVertex(gp_Pnt(0, 0, p[2])).Vertex());
      loft.Build();
      return centred(loft.Shape());
    }
    case BK_HEMISPHERE:
      return centred(BRepPrimAPI_MakeSphere(p[0] / 2, 0, M_PI / 2).Shape());
    case BK_BOWL: {
      // Lower half of a spherical shell (centre at rim height), rim flat at the top.
      double r = p[0] / 2, t = std::min(std::max(p[1], 0.05), r * 0.95), ri = r - t;
      gp_Pnt c(0, 0, r);
      BRepBuilderAPI_MakeWire w;
      w.Add(BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(gp_Pnt(0, 0, 0), gp_Pnt(r * sin(M_PI / 4), 0, r - r * cos(M_PI / 4)), gp_Pnt(r, 0, r)).Value()).Edge());
      w.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(r, 0, r), gp_Pnt(ri, 0, r)).Edge());
      w.Add(BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(gp_Pnt(ri, 0, r), gp_Pnt(ri * sin(M_PI / 4), 0, r - ri * cos(M_PI / 4)), gp_Pnt(0, 0, t)).Value()).Edge());
      w.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, t), gp_Pnt(0, 0, 0)).Edge());
      TopoDS_Face face = BRepBuilderAPI_MakeFace(w.Wire(), true).Face();
      return centred(BRepPrimAPI_MakeRevol(face, gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1))).Shape());
    }
    case BK_RING: {
      double ro = p[0] / 2, ri = std::min(std::max(p[1] / 2, 0.0), ro - 0.05), h = p[2];
      return centred(revolve({{ri, 0}, {ro, 0}, {ro, h}, {ri, h}}));
    }
    case BK_GLASS: {
      double r = p[0] / 2, h = p[1], w = std::min(std::max(p[2], 0.05), r * 0.95), b = std::min(std::max(p[3], 0.05), h * 0.95);
      return centred(revolve({{0, 0}, {r, 0}, {r, h}, {r - w, h}, {r - w, b}, {0, b}}));
    }
    case BK_OVAL: {
      double a = std::max(p[0], 0.01) / 2, b = std::max(p[1], 0.01) / 2, t = std::min(std::max(p[2], 5.0), 175.0) * M_PI / 180;
      double major, minor, phi;
      conjugate(a, b, t, major, minor, phi);
      TopoDS_Face base = BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(oval(major, minor, phi)).Wire()).Face();
      return centred(BRepPrimAPI_MakePrism(base, gp_Vec(0, 0, p[3])).Shape());
    }
    case BK_OVAL_TORUS: {
      // The tube is swept along the oval through its middle, upright all the way round (a fixed binormal along z).
      int sides = (int)lround(p[0]);
      need(sides == 0 || sides == 3 || sides == 6, "unknown tube shape");
      double w = p[4] / 2, a = p[1] / 2 - w, b = p[2] / 2 - w, t = std::min(std::max(p[3], 5.0), 175.0) * M_PI / 180;
      need(a > 0 && b > 0, "the tube is too thick for this torus");
      double major, minor, phi;
      conjugate(a, b, t, major, minor, phi);
      // The tube must fit the tightest bend (radius minor²/major) or its inner side folds over itself.
      need(w * 1.05 < minor * minor / major, "the tube is too thick for this torus");
      gp_Dir radial(cos(phi), sin(phi), 0), up(0, 0, 1);
      gp_Pnt start(major * cos(phi), major * sin(phi), 0);
      TopoDS_Wire section;
      if (sides == 0) {
        section = BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(gp_Circ(gp_Ax2(start, up.Crossed(radial), radial), w)).Edge()).Wire();
      } else {
        std::vector<gp_Pnt> pts;
        for (auto &q : tubeOutline(sides, w, 0)) pts.push_back(start.Translated(gp_Vec(radial) * q.first + gp_Vec(up) * q.second));
        section = polygon(pts);
      }
      BRepOffsetAPI_MakePipeShell pipe(BRepBuilderAPI_MakeWire(oval(major, minor, phi)).Wire());
      pipe.SetMode(up);
      pipe.Add(section);
      pipe.Build();
      need(pipe.IsDone() && pipe.MakeSolid(), "the tube can't follow this oval");
      return centred(pipe.Shape());
    }
    }
    return TopoDS_Shape();
  });
}

// MARK: - fasteners: ISO 261 coarse threads with bolt heads and nut shapes

// Per thread: the sizes each head and nut starts from, standard ones where there is a standard.
struct ThreadSize {
  const char *name;
  double d, p;
  double s, k, m, length;      // hex head ISO 4017: across flats, height; hex nut ISO 4032: height; a usual bolt length
  double dk, key, t;           // socket head ISO 4762: diameter, hex key, socket depth (its height is d)
  double cskDk, cskKey, cskT;  // countersunk socket ISO 10642 (DIN 7991 where it has none): diameter, hex key, depth
  int torx, torxCsk;           // Torx size: cap head ISO 14579, countersunk ISO 14581 (depths as for the hex sockets)
  double phDk;                 // PH countersunk ISO 7046: theoretical head diameter, PH size, recess diameter and depth
  int ph;
  double phM, phT;
  double sq, sqM;              // square nut DIN 557: across flats, height
};
static const ThreadSize sizes[] = {
    {"M3", 3, 0.5, 5.5, 2.0, 2.4, 12, 5.5, 2.5, 1.3, 6.72, 2, 1.1, 20, 10, 6.3, 1, 3.2, 1.8, 5.5, 2.4},
    {"M4", 4, 0.7, 7, 2.8, 3.2, 16, 7, 3, 2, 8.96, 2.5, 1.5, 25, 20, 9.4, 2, 4.6, 2.6, 7, 3.2},
    {"M5", 5, 0.8, 8, 3.5, 4.7, 20, 8.5, 4, 2.5, 11.2, 3, 1.9, 27, 25, 10.4, 2, 5.2, 2.9, 8, 4},
    {"M6", 6, 1.0, 10, 4.0, 5.2, 25, 10, 5, 3, 13.44, 4, 2.2, 30, 30, 12.6, 3, 6.8, 3.5, 10, 5},
    {"M8", 8, 1.25, 13, 5.3, 6.8, 30, 13, 6, 4, 17.92, 5, 3.0, 45, 40, 17.3, 4, 8.9, 4.6, 13, 6.5},
    {"M10", 10, 1.5, 16, 6.4, 8.4, 35, 16, 8, 5, 22.4, 6, 3.6, 50, 50, 20, 4, 10, 5.2, 17, 8},
    {"M12", 12, 1.75, 18, 7.5, 10.8, 40, 18, 10, 6, 26.88, 8, 4.3, 55, 55, 24, 4, 10, 5.2, 19, 10},
    {"M14", 14, 2.0, 21, 8.8, 12.8, 45, 21, 12, 7, 30.8, 10, 4.5, 60, 55, 28, 4, 10, 5.2, 22, 11},
    {"M16", 16, 2.0, 24, 10.0, 14.8, 50, 24, 14, 8, 33.6, 10, 4.8, 70, 60, 32, 4, 10, 5.2, 24, 13},
    {"M18", 18, 2.5, 27, 11.5, 15.8, 55, 27, 14, 9, 36, 12, 5.3, 80, 70, 36, 4, 10, 5.2, 27, 15},
    {"M20", 20, 2.5, 30, 12.5, 18.0, 60, 30, 17, 10, 40.32, 12, 5.6, 90, 80, 40, 4, 10, 5.2, 30, 16},
    {"M22", 22, 2.5, 34, 14.0, 19.4, 65, 33, 17, 11, 44, 14, 6.0, 90, 90, 44, 4, 10, 5.2, 32, 18},
    {"M24", 24, 3.0, 36, 15.0, 21.5, 70, 36, 19, 12, 48, 14, 6.4, 100, 90, 48, 4, 10, 5.2, 36, 19},
};
static const int sizeCount = sizeof(sizes) / sizeof(sizes[0]);

// Torx (hexalobular, ISO 10664): size, across the lobes (A) and across the valleys (B).
struct TorxSize {
  int n;
  double a, b;
};
static const TorxSize torxSizes[] = {{10, 2.80, 2.05}, {15, 3.35, 2.40}, {20, 3.95, 2.85}, {25, 4.50, 3.25}, {27, 5.10, 3.65},
                                     {30, 5.60, 4.05}, {40, 6.75, 4.85}, {45, 7.93, 5.64}, {50, 8.95, 6.45}, {55, 11.35, 8.05},
                                     {60, 13.45, 9.60}, {70, 15.70, 11.20}, {80, 17.75, 12.80}, {90, 20.20, 14.40}, {100, 22.40, 16.00}};
static const int torxCount = sizeof(torxSizes) / sizeof(torxSizes[0]);
static const TorxSize *torxOf(double n) {
  for (const auto &t : torxSizes)
    if (fabs(t.n - n) < 1e-6) return &t;
  return nullptr;
}

// A Phillips recess per PH size: the width of its wings, and the recess diameter and depth it usually has.
static const double phWing[] = {0, 0.85, 1.25, 1.8, 2.4}, phRecess[] = {0, 3.2, 5.0, 6.8, 8.9}, phDepth[] = {0, 1.8, 2.8, 3.5, 4.6};

// Basic thread depth per pitch (ISO 68-1): from the major to the minor radius.
static const double rootDepth = 0.5412658774;

int bk_thread_count(void) { return sizeCount; }
const char *bk_thread_name(int size) { return size >= 0 && size < sizeCount ? sizes[size].name : ""; }
int bk_torx_count(void) { return torxCount; }
int bk_torx_number(int i) { return i >= 0 && i < torxCount ? torxSizes[i].n : 0; }

static bool isNut(int k) { return k >= BK_SLEEVE; }
static bool countersunk(int k) { return k == BK_SOCKET_CONE || k == BK_TORX_CONE || k == BK_PH_CONE; }
static bool coneBelow(int k) { return k == BK_HEX_CONE || k == BK_TWELVE_CONE || k == BK_PH_HEX_CONE; }
static bool phillipsDrive(int k) { return k == BK_PH_HEX || k == BK_PH_HEX_CONE || k == BK_PH_CONE; }
static bool torxDrive(int k) { return k == BK_TORX || k == BK_TORX_CONE; }
static bool keyDrive(int k) { return k == BK_SOCKET || k == BK_SOCKET_CONE; }
static const ThreadSize &threadOf(int size) { return sizes[std::max(0, std::min(sizeCount - 1, size))]; }

int bk_fastener_fields(int kind) {
  const int L = 1 << BK_LENGTH, W = 1 << BK_WIDTH, H = 1 << BK_HEIGHT, A = 1 << BK_ANGLE, C = 1 << BK_SEAT, D = 1 << BK_DRIVE,
            R = 1 << BK_RECESS, T = 1 << BK_DEPTH;
  switch (kind) {
  case BK_ROD: return L;
  case BK_HEX: case BK_TWELVE: return L | W | H;
  case BK_HEX_CONE: case BK_TWELVE_CONE: return L | W | H | A;
  case BK_SOCKET: case BK_TORX: return L | W | H | D | T;
  case BK_SOCKET_CONE: case BK_TORX_CONE: return L | W | A | D | T;
  case BK_PH_HEX: return L | W | H | D | R | T;
  case BK_PH_HEX_CONE: return L | W | H | A | D | R | T;
  case BK_PH_CONE: return L | W | A | D | R | T;
  case BK_SLEEVE: case BK_SQUARE_NUT: case BK_HEX_NUT: return L | W;
  case BK_CONE_NUT: return L | W | A | C;
  default: return 0;
  }
}

// The radius where a bolt's cone meets its head (a countersunk head's rim; just inside the flats of a head above a cone).
static double coneTop(const BKFastener &f) { return countersunk(f.kind) ? f.width / 2 : 0.46 * f.width; }
static double tanHalf(double angle) { return tan(angle * M_PI / 360); }

// How far a bolt's cone reaches below where its head starts, down to the thread's outside (0 without a cone).
static double coneDrop(const BKFastener &f, double d) {
  if (!countersunk(f.kind) && !coneBelow(f.kind)) return 0;
  return (coneTop(f) - d / 2) / tanHalf(f.angle);
}

// The drive's size across its outside: a hex key's corners, a Torx's lobes, a Phillips recess; 0 without one.
static double driveSpan(const BKFastener &f) {
  if (keyDrive(f.kind)) return f.drive * 2 / sqrt(3.0);
  if (torxDrive(f.kind)) return torxOf(f.drive) ? torxOf(f.drive)->a : 0;
  if (phillipsDrive(f.kind)) return f.recess;
  return 0;
}

// How wide a hex key or Torx may be: half a millimetre of wall all round, in a countersunk head down at the drive's bottom.
static double driveRoom(const BKFastener &f) {
  return f.width - 1 - (countersunk(f.kind) ? 2 * f.depth * tanHalf(f.angle) : 0);
}

void bk_fastener_range(const BKFastener *fp, int field, double *out) {
  const BKFastener &f = *fp;
  const ThreadSize &t = threadOf(f.size);
  int k = f.kind;
  double d = t.d, p = t.p, w = f.width, L = f.length, big = 4 * d + 20, lo = 0, hi = 0;
  bool csk = countersunk(k), below = coneBelow(k), nut = isNut(k), hexed = k == BK_HEX || k == BK_HEX_CONE || k == BK_TWELVE ||
                                                                       k == BK_TWELVE_CONE || k == BK_PH_HEX || k == BK_PH_HEX_CONE;
  double th = tanHalf(f.angle), span = driveSpan(f);
  int ph = std::max(1, std::min(4, (int)lround(f.drive)));
  double bottom = phillipsDrive(k) ? phWing[ph] : span;  // the drive's width at its bottom
  bool slim = bottom / 2 + 0.5 <= d / 2 - rootDepth * p;  // it fits inside the thread's core
  switch (field) {
  case BK_LENGTH:
    lo = k == BK_CONE_NUT ? f.seat + p : nut ? 2 * p : k == BK_ROD ? 4 * p : coneDrop(f, d) + 2 * p;
    hi = 10000;
    break;
  case BK_WIDTH:
    if (k == BK_SLEEVE) {
      lo = 0.4, hi = d;
    } else if (k == BK_CONE_NUT) {
      lo = (d + 1 + 2 * f.seat * th) / 0.92, hi = big;
    } else if (nut) {
      lo = d + 1.6, hi = big;
    } else if (k != BK_ROD) {
      // Wider than the thread, with a wall around the drive (in a countersunk head, still at the drive's bottom), and a
      // cone (if any) at least a pitch tall that leaves thread.
      lo = std::max(d + 1, hexed ? span / 0.8 : span + 1);
      hi = big;
      if (csk) {
        lo = std::max({lo, bottom + 2 * f.depth * th + 1, d + 2 * th * p}), hi = std::min(hi, d + 2 * th * (L - 2 * p));
        if (!slim) lo = std::max(lo, d + 2 * f.depth * th);
      }
      if (below) lo = std::max(lo, (d / 2 + th * p) / 0.46), hi = std::min(hi, (d / 2 + th * (L - 2 * p)) / 0.46);
    }
    break;
  case BK_HEIGHT:
    lo = span > 0 ? f.depth + 0.3 : 0.5, hi = 3 * d + 5;
    break;
  case BK_ANGLE: {
    lo = k == BK_CONE_NUT ? 30 : 60, hi = 150;
    if (k == BK_CONE_NUT) {
      hi = std::min(hi, atan((0.92 * w - d - 1) / (2 * f.seat)) * 360 / M_PI);
    } else {
      // The cone at least a pitch tall, and leaving two pitches of thread.
      double rise = coneTop(f) - d / 2;
      hi = std::min(hi, atan(rise / p) * 360 / M_PI);
      lo = std::max(lo, atan(rise / std::max(L - 2 * p, 1e-9)) * 360 / M_PI);
      if (csk) hi = std::min({hi, atan((w - bottom - 1) / (2 * f.depth)) * 360 / M_PI, slim ? 180.0 : atan((w - d) / (2 * f.depth)) * 360 / M_PI});
    }
    break;
  }
  case BK_SEAT:
    lo = 0.2, hi = std::min(L - p, (0.92 * w - d - 1) / (2 * th));
    break;
  case BK_DRIVE: {
    double room = driveRoom(f);
    if (keyDrive(k)) {
      lo = 0.7, hi = room * sqrt(3.0) / 2;
    } else if (torxDrive(k)) {
      lo = torxSizes[0].n;
      for (const auto &x : torxSizes)
        if (x.a <= room) hi = x.n;
    } else if (phillipsDrive(k)) {
      lo = 1, hi = 4;
    }
    break;
  }
  case BK_RECESS:
    lo = 2 * phWing[ph] + 0.4, hi = hexed ? 0.8 * w : w - 1;
    break;
  case BK_DEPTH: {
    // In a countersunk head the cone narrows downwards: the drive's bottom keeps half a millimetre of wall in it, and
    // reaches into the thread below only when it's narrower than the thread's core.
    double drop = coneDrop(f, d);
    lo = 0.3, hi = csk ? std::min((w - bottom - 1) / (2 * th), slim ? drop + (L - drop) / 2 : drop) : f.height - 0.3;
    break;
  }
  }
  out[0] = lo;
  out[1] = hi;
}

static const char *misfit(int field) {
  switch (field) {
  case BK_LENGTH: return "the length leaves no thread";
  case BK_WIDTH: return "the head or nut doesn't fit its thread and drive";
  case BK_HEIGHT: return "the head is too low for its drive";
  case BK_ANGLE: return "the cone's angle doesn't fit";
  case BK_SEAT: return "the cone doesn't fit this nut";
  case BK_DRIVE: return "the drive doesn't fit the head";
  case BK_RECESS: return "the recess doesn't fit the head";
  default: return "the drive is deeper than the head";
  }
}

// Every size this kind has, within what the others leave it.
static void checkFit(const BKFastener &f) {
  need(f.kind >= BK_ROD && f.kind <= BK_CONE_NUT, "unknown bolt or nut");
  need(f.size >= 0 && f.size < sizeCount, "unknown thread");
  const double v[] = {f.length, f.width, f.height, f.angle, f.seat, f.drive, f.recess, f.depth};
  need(finite(v, 8), "sizes must be numbers");
  int fields = bk_fastener_fields(f.kind);
  need(!torxDrive(f.kind) || torxOf(f.drive), "unknown Torx size");
  need(!phillipsDrive(f.kind) || (f.drive == 1 || f.drive == 2 || f.drive == 3 || f.drive == 4), "unknown PH size");
  for (int i = BK_LENGTH; i <= BK_DEPTH; i++) {
    if (!(fields & (1 << i))) continue;
    double r[2];
    bk_fastener_range(&f, i, r);
    need(v[i] >= r[0] - 1e-9 && v[i] <= r[1] + 1e-9, misfit(i));
  }
}

// Each size of the kind in turn into what the others leave it (after one size was changed, the ones depending on it follow).
void bk_fastener_fit(BKFastener *f) {
  double *v[] = {&f->length, &f->width, &f->height, &f->angle, &f->seat, &f->drive, &f->recess, &f->depth};
  int fields = bk_fastener_fields(f->kind);
  for (int i : {BK_DRIVE, BK_RECESS, BK_DEPTH, BK_HEIGHT, BK_SEAT, BK_ANGLE, BK_WIDTH, BK_LENGTH}) {
    if (!(fields & (1 << i))) continue;
    double r[2];
    bk_fastener_range(f, i, r);
    if (r[0] > r[1]) continue;
    if (i == BK_DRIVE && torxDrive(f->kind)) {
      // The nearest Torx size that fits.
      int best = (int)r[0];
      for (const auto &x : torxSizes)
        if (x.n <= r[1] && fabs(x.n - *v[i]) < fabs(best - *v[i])) best = x.n;
      *v[i] = best;
    } else {
      *v[i] = std::max(r[0], std::min(r[1], *v[i]));
      if (i == BK_DRIVE && phillipsDrive(f->kind)) *v[i] = lround(*v[i]);
    }
  }
}

void bk_fastener_defaults(BKFastener *f, int withLength) {
  const ThreadSize &t = threadOf(f->size);
  int k = f->kind;
  f->width = f->height = f->angle = f->seat = f->drive = f->recess = f->depth = 0;
  switch (k) {
  case BK_HEX: case BK_HEX_CONE: case BK_TWELVE: case BK_TWELVE_CONE:
    f->width = t.s, f->height = t.k;
    break;
  case BK_PH_HEX: case BK_PH_HEX_CONE:
    f->width = t.s, f->height = t.k, f->drive = t.ph, f->recess = t.phM, f->depth = std::min(t.phT, 0.6 * t.k);
    break;
  case BK_SOCKET: f->width = t.dk, f->height = t.d, f->drive = t.key, f->depth = t.t; break;
  case BK_TORX: f->width = t.dk, f->height = t.d, f->drive = t.torx, f->depth = t.t; break;
  case BK_SOCKET_CONE: f->width = t.cskDk, f->drive = t.cskKey, f->depth = t.cskT; break;
  case BK_TORX_CONE: f->width = t.cskDk, f->drive = t.torxCsk, f->depth = t.cskT; break;
  case BK_PH_CONE: f->width = t.phDk, f->drive = t.ph, f->recess = t.phM, f->depth = t.phT; break;
  case BK_SLEEVE: f->width = 1.6; break;
  case BK_SQUARE_NUT: f->width = t.sq; break;
  case BK_HEX_NUT: f->width = t.s; break;
  case BK_CONE_NUT:
    // A 60° seat narrowing to a wall of 0.6 mm or more round the thread.
    f->width = t.s, f->angle = 60, f->seat = (0.46 * t.s - t.d / 2 - std::max(0.6, 0.06 * t.d)) / tanHalf(60);
    break;
  }
  if (coneBelow(k) || countersunk(k)) f->angle = 90;
  if (withLength) f->length = k == BK_SQUARE_NUT ? t.sqM : k == BK_CONE_NUT ? t.m + f->seat : isNut(k) ? t.m : t.length;
}

// A Phillips size changes the recess with it, to what that size usually has, as far as the head allows.
void bk_fastener_drive(BKFastener *f, double drive) {
  f->drive = drive;
  if (phillipsDrive(f->kind)) {
    int ph = std::max(1, std::min(4, (int)lround(drive)));
    f->recess = phRecess[ph], f->depth = phDepth[ph];
  }
  bk_fastener_fit(f);
}

void bk_fastener_extent(const BKFastener *fp, double clearance, double *out) {
  const BKFastener &f = *fp;
  const ThreadSize &t = threadOf(f.size);
  double c = std::isfinite(clearance) ? std::max(0.0, clearance) : 0, w = f.width, corners = 2 * w / sqrt(3.0);
  double z = f.length + (isNut(f.kind) || countersunk(f.kind) || f.kind == BK_ROD ? 0 : f.height);
  double x = w, y = w;
  switch (f.kind) {
  case BK_ROD: x = y = t.d - c; break;
  case BK_HEX: case BK_HEX_CONE: case BK_PH_HEX: case BK_PH_HEX_CONE: case BK_HEX_NUT: case BK_CONE_NUT: x = corners; break;
  case BK_TWELVE: case BK_TWELVE_CONE: x = y = corners; break;
  case BK_SLEEVE: x = y = t.d + c + 2 * w; break;
  }
  out[0] = x, out[1] = y, out[2] = z;
}

// A thread is built face by face instead of by sweeping and fusing: per turn, a crest strip on the major
// cylinder, a flank, a root strip on the minor cylinder and a flank (basic ISO 68-1 profile), sewn into one
// solid with helical ends. Small per-turn faces keep the later booleans fast.

// One helix turn on a cylinder of radius r, starting at angle 0 and height z, rising p.
static TopoDS_Edge helixTurn(double r, double z, double p) {
  Handle(Geom_CylindricalSurface) cyl = new Geom_CylindricalSurface(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), r);
  TopoDS_Edge e = BRepBuilderAPI_MakeEdge(new Geom2d_Line(gp_Pnt2d(0, z), gp_Dir2d(2 * M_PI, p)), cyl, 0, sqrt(4 * M_PI * M_PI + p * p)).Edge();
  BRepLib::BuildCurves3d(e, 1e-5, GeomAbs_C2, 8, 16);
  return e;
}

// One turn of a strip on a cylinder of radius r, between heights z0 < z1 at angle 0, rising p.
static TopoDS_Face helixStrip(double r, double z0, double z1, double p) {
  Handle(Geom_CylindricalSurface) cyl = new Geom_CylindricalSurface(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), r);
  double turn = sqrt(4 * M_PI * M_PI + p * p);
  BRepBuilderAPI_MakeWire w;
  w.Add(BRepBuilderAPI_MakeEdge(new Geom2d_Line(gp_Pnt2d(0, z0), gp_Dir2d(2 * M_PI, p)), cyl, 0, turn).Edge());
  w.Add(BRepBuilderAPI_MakeEdge(new Geom2d_Line(gp_Pnt2d(2 * M_PI, z0 + p), gp_Dir2d(0, 1)), cyl, 0, z1 - z0).Edge());
  w.Add(BRepBuilderAPI_MakeEdge(new Geom2d_Line(gp_Pnt2d(2 * M_PI, z1 + p), gp_Dir2d(-2 * M_PI, -p)), cyl, 0, turn).Edge());
  w.Add(BRepBuilderAPI_MakeEdge(new Geom2d_Line(gp_Pnt2d(0, z1), gp_Dir2d(0, -1)), cyl, 0, z1 - z0).Edge());
  TopoDS_Face f = BRepBuilderAPI_MakeFace(cyl, w.Wire(), true).Face();
  BRepLib::BuildCurves3d(f, 1e-5, GeomAbs_C2, 8, 16);
  return f;
}

// Closed threaded rod of major diameter d covering at least [zlo, zhi]; its helical ends lie outside that range.
static TopoDS_Shape threadSolid(double d, double p, double zlo, double zhi) {
  double rMaj = d / 2, rMin = rMaj - rootDepth * p, z0 = zlo - p;
  int n = (int)ceil((zhi - z0) / p) + 1;
  BRepBuilderAPI_Sewing sew(1e-4);
  for (int k = 0; k < n; k++) {
    double b = z0 + k * p;
    sew.Add(helixStrip(rMaj, b - p / 16, b + p / 16, p));
    sew.Add(BRepFill::Face(helixTurn(rMaj, b + p / 16, p), helixTurn(rMin, b + 3 * p / 8, p)));
    sew.Add(helixStrip(rMin, b + 3 * p / 8, b + 5 * p / 8, p));
    sew.Add(BRepFill::Face(helixTurn(rMin, b + 5 * p / 8, p), helixTurn(rMaj, b + 15 * p / 16, p)));
  }
  // Each end: a helicoid from the crest helix to the axis, and the flat profile at angle 0.
  for (double b : {z0, z0 + n * p}) {
    TopoDS_Edge axis = BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, b - p / 16), gp_Pnt(0, 0, b + 15 * p / 16)).Edge();
    sew.Add(BRepFill::Face(helixTurn(rMaj, b - p / 16, p), axis));
    sew.Add(BRepBuilderAPI_MakeFace(polygon({gp_Pnt(rMaj, 0, b - p / 16), gp_Pnt(rMaj, 0, b + p / 16), gp_Pnt(rMin, 0, b + 3 * p / 8),
                                             gp_Pnt(rMin, 0, b + 5 * p / 8), gp_Pnt(rMaj, 0, b + 15 * p / 16), gp_Pnt(0, 0, b + 15 * p / 16),
                                             gp_Pnt(0, 0, b - p / 16)})).Face());
  }
  sew.Perform();
  BRepBuilderAPI_MakeSolid solid;
  for (TopExp_Explorer ex(sew.SewedShape(), TopAbs_SHELL); ex.More(); ex.Next()) solid.Add(TopoDS::Shell(ex.Current()));
  if (!solid.IsDone()) throw Standard_Failure("thread failed");
  TopoDS_Solid out = solid.Solid();
  BRepLib::OrientClosedSolid(out);
  return out;
}

// The last threads made, so that changing only a head, a drive or a nut's outside doesn't make the thread again.
template <typename F> static TopoDS_Shape kept(const std::string &key, F make) {
  static thread_local std::vector<std::pair<std::string, TopoDS_Shape>> memo;
  for (size_t i = 0; i < memo.size(); i++) {
    if (memo[i].first != key) continue;
    auto hit = memo[i];
    memo.erase(memo.begin() + i);
    memo.insert(memo.begin(), hit);
    return hit.second;
  }
  TopoDS_Shape s = make();
  memo.insert(memo.begin(), {key, s});
  if (memo.size() > 6) memo.pop_back();
  return s;
}

static std::string keyOf(const char *what, std::initializer_list<double> v) {
  std::string key = what;
  char n[32];
  for (double x : v) snprintf(n, sizeof n, " %.9g", x), key += n;
  return key;
}

static TopoDS_Shape cutAll(const TopoDS_Shape &a, std::initializer_list<TopoDS_Shape> tools) {
  TopTools_ListOfShape args, with;
  args.Append(a);
  for (const auto &t : tools) with.Append(t);
  BRepAlgoAPI_Cut op;
  op.SetArguments(args);
  op.SetTools(with);
  op.Build();
  if (!op.IsDone()) throw Standard_Failure("subtract failed");
  return op.Shape();
}

static TopoDS_Shape fuseAll(const TopoDS_Shape &a, std::initializer_list<TopoDS_Shape> tools) {
  TopTools_ListOfShape args, with;
  args.Append(a);
  for (const auto &t : tools) with.Append(t);
  BRepAlgoAPI_Fuse op;
  op.SetArguments(args);
  op.SetTools(with);
  op.Build();
  if (!op.IsDone()) throw Standard_Failure("merge failed");
  return op.Shape();
}

static TopoDS_Shape raised(const TopoDS_Shape &s, double z) {
  gp_Trsf up;
  up.SetTranslation(gp_Vec(0, 0, z));
  return BRepBuilderAPI_Transform(s, up, true).Shape();
}

// External thread of exactly [0, len], chamfered 45° at the bottom and optionally the top. The raw thread's helical ends
// are cut off by a piece at each end, which only meets the end turns (one cut along the whole length is much slower).
static TopoDS_Shape threadRod(double d, double p, double len, bool chamferTop) {
  return kept(keyOf("rod", {d, p, len, (double)chamferTop}), [&] {
    double rMaj = d / 2, rMin = rMaj - rootDepth * p, c = std::min(rMaj - rMin + 0.05, len / 3), R = rMaj + 1;
    double lo = -2 * p - 0.5, hi = len + 4 * p;
    TopoDS_Shape bottom = revolve({{0, lo}, {R, lo}, {R, R - rMaj + c}, {rMaj - c, 0}, {0, 0}});
    TopoDS_Shape top = chamferTop ? revolve({{0, len}, {rMaj - c, len}, {R, len - c - (R - rMaj)}, {R, hi}, {0, hi}})
                                  : revolve({{0, len}, {R, len}, {R, hi}, {0, hi}});
    return cutAll(threadSolid(d, p, 0, len), {bottom, top});
  });
}

// A prism of this outline (in z = 0) up to h, its corners chamfered 30° at the bottom and/or top outside radius r0, as on
// nuts and bolt heads; rc reaches the outline's corners. On a thin one the chamfers start further out, so they take at
// most 0.4·h each and the corners keep their full size in between.
static TopoDS_Shape chamfered(const TopoDS_Wire &outline, double r0, double rc, double h, bool bottom, bool top) {
  TopoDS_Shape block = BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(outline).Face(), gp_Vec(0, 0, h)).Shape();
  double slope = tan(M_PI / 6), most = (bottom && top ? 0.4 : 0.8) * h;
  r0 = std::max(r0, rc - most / slope);
  // The turned outline: the chamfer lines run out to rBig, or meet (both ends) or reach the other end first.
  double rBig = rc + 1, z1 = (rBig - r0) * slope, z2 = h - z1;
  std::vector<std::pair<double, double>> rz = {{0, 0}, {bottom ? r0 : rBig, 0}};
  if (bottom && top && z1 >= z2) {
    rz.push_back({r0 + h / 2 / slope, h / 2});
  } else if (bottom && !top && z1 >= h) {
    rz.push_back({r0 + h / slope, h});
  } else if (top && !bottom && z2 <= 0) {
    rz.back() = {r0 + h / slope, 0};
  } else {
    if (bottom) rz.push_back({rBig, z1});
    rz.push_back({rBig, top ? z2 : h});
  }
  if (top) rz.push_back({r0, h});
  else if (rz.back().second < h) rz.push_back({rBig, h});
  rz.push_back({0, h});
  return common(block, revolve(rz));
}

// Hex prism (across flats s) chamfered at the bottom and/or top.
static TopoDS_Shape hexBlank(double s, double h, bool bottom, bool top) {
  return chamfered(polygon(ngon(6, s / sqrt(3.0), 0)), 0.475 * s, s / sqrt(3.0), h, bottom, top);
}

// A 12-point outline: two hexagons across flats s, turned 30° to each other.
static TopoDS_Wire twelvePoint(double s) {
  std::vector<gp_Pnt> pts;
  double tip = s / sqrt(3.0), valley = s / 2 / cos(M_PI / 12);
  for (int i = 0; i < 12; i++) {
    double a = i * M_PI / 6;
    pts.emplace_back(tip * cos(a), tip * sin(a), 0);
    pts.emplace_back(valley * cos(a + M_PI / 12), valley * sin(a + M_PI / 12), 0);
  }
  return polygon(pts);
}

// A Torx outline (in z = 0): six round lobes a across, joined by six round valleys b across. Each valley circle touches
// the lobe circles beside it, which sets its radius.
static TopoDS_Wire torxOutline(double a, double b) {
  double re = 0.1 * a, ro = a / 2 - re, h = b / 2, c30 = cos(M_PI / 6);
  double ri = (ro * ro + h * h - 2 * ro * c30 * h - re * re) / (2 * re + 2 * ro * c30 - 2 * h);
  auto at = [](double r, double angle) { return gp_Pnt(r * cos(angle), r * sin(angle), 0); };
  // Where lobe i meets the valley before it and the one after it.
  std::vector<gp_Pnt> before(6), after(6);
  for (int i = 0; i < 6; i++) {
    double a0 = i * M_PI / 3;
    gp_Pnt co = at(ro, a0);
    for (int side : {-1, 1}) {
      gp_Pnt ci = at(h + ri, a0 + side * M_PI / 6);
      gp_Pnt touch(co.XYZ() + (ci.XYZ() - co.XYZ()) * (re / (re + ri)));
      (side < 0 ? before : after)[i] = touch;
    }
  }
  BRepBuilderAPI_MakeWire wire;
  for (int i = 0; i < 6; i++) {
    double a0 = i * M_PI / 3;
    wire.Add(BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(before[i], at(a / 2, a0), after[i]).Value()).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(after[i], at(h, a0 + M_PI / 6), before[(i + 1) % 6]).Value()).Edge());
  }
  return wire.Wire();
}

// A flat-bottomed socket of this outline (around the axis, in z = 0) cut depth down from z = top.
static TopoDS_Shape socket(const TopoDS_Wire &outline, double depth, double top) {
  return raised(BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(outline).Face(), gp_Vec(0, 0, depth + 1)).Shape(), top - depth);
}

// A Phillips recess cut down from z = top: two crossed slots, each a wing wide, their ends sloping in from the recess
// diameter m at the top to the wing width at the bottom, and a cone opening up the middle.
static TopoDS_Shape phillips(int ph, double m, double depth, double top) {
  double w = phWing[ph], slope = (m - w) / 2 / depth, half = m / 2 + slope;
  TopoDS_Face side = BRepBuilderAPI_MakeFace(polygon({gp_Pnt(-half, -w / 2, top + 1), gp_Pnt(half, -w / 2, top + 1),
                                                      gp_Pnt(w / 2, -w / 2, top - depth), gp_Pnt(-w / 2, -w / 2, top - depth)})).Face();
  TopoDS_Shape slot = BRepPrimAPI_MakePrism(side, gp_Vec(0, w, 0)).Shape();
  gp_Trsf quarter;
  quarter.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), M_PI / 2);
  double rc = 0.3 * m, drop = 0.9 * depth;
  TopoDS_Shape cone = revolve({{0, top - drop}, {rc * (drop + 1) / drop, top + 1}, {0, top + 1}});
  return fuseAll(slot, {BRepBuilderAPI_Transform(slot, quarter, true).Shape(), cone});
}

// What the drive takes out of a bolt whose head is flat on top at z = top (nothing without a drive).
static TopoDS_Shape driveCut(const BKFastener &f, double top) {
  if (keyDrive(f.kind)) return socket(polygon(ngon(6, f.drive / sqrt(3.0), 0)), f.depth, top);
  if (torxDrive(f.kind)) {
    const TorxSize *x = torxOf(f.drive);
    return socket(torxOutline(x->a, x->b), f.depth, top);
  }
  if (phillipsDrive(f.kind)) return phillips((int)lround(f.drive), f.recess, f.depth, top);
  return TopoDS_Shape();
}

// A bolt head from z = 0 up to its height: hex or 12-point (chamfered, below too when a cone sits under it), or round
// with its top edge chamfered.
static TopoDS_Shape headBlock(const BKFastener &f) {
  double w = f.width, k = f.height;
  bool below = coneBelow(f.kind);
  switch (f.kind) {
  case BK_TWELVE: case BK_TWELVE_CONE: return chamfered(twelvePoint(w), 0.475 * w, w / sqrt(3.0), k, below, true);
  case BK_SOCKET: case BK_TORX: {
    double ch = std::min({0.08 * w, 0.3 * k, 0.4 * (w - driveSpan(f))});
    return revolve({{0, 0}, {w / 2, 0}, {w / 2, k - ch}, {w / 2 - ch, k}, {0, k}});
  }
  default: return hexBlank(w, k, below, true);
  }
}

// The thread runs from z = 0 to the length; a head sits on top, or a cone under the head reaches down into the thread
// (countersunk: the cone is the head). Its narrow end lies inside the thread's core and the thread ends inside the cone,
// so no faces meet edge on edge.
static TopoDS_Shape boltShape(const BKFastener &f, double clearance) {
  const ThreadSize &t = threadOf(f.size);
  double d = t.d - std::max(0.0, clearance), p = t.p, L = f.length;
  if (f.kind == BK_ROD) return threadRod(d, p, L, true);
  bool csk = countersunk(f.kind);
  TopoDS_Shape whole;
  if (csk || coneBelow(f.kind)) {
    double th = tanHalf(f.angle), rTop = coneTop(f), core = 0.9 * (d / 2 - rootDepth * p);
    double drop = (rTop - d / 2) / th, low = L - (rTop - core) / th;
    TopoDS_Shape cone = revolve({{0, low}, {core, low}, {rTop, L}, {0, L}});
    TopoDS_Shape rod = threadRod(d, p, L - drop + p / 2, false);
    whole = csk ? fuse(rod, cone) : fuseAll(rod, {cone, raised(headBlock(f), L)});
  } else {
    whole = fuse(threadRod(d, p, L, false), raised(headBlock(f), L));
  }
  TopoDS_Shape drive = driveCut(f, csk ? L : L + f.height);
  return drive.IsNull() ? whole : cut(whole, drive);
}

// The nut's outside from z = 0 up to its height, tapped through, the thread's ends countersunk (not at a cone nut's seat).
static TopoDS_Shape nutShape(const BKFastener &f, double clearance) {
  const ThreadSize &t = threadOf(f.size);
  double d = t.d + std::max(0.0, clearance), r = d / 2, L = f.length, w = f.width, face = 0.475 * w;
  TopoDS_Shape body;
  switch (f.kind) {
  case BK_SLEEVE:
    body = BRepPrimAPI_MakeCylinder(r + w, L).Shape(), face = r + w;
    break;
  case BK_SQUARE_NUT: body = chamfered(polygon(ngon(4, w / sqrt(2.0), 0)), face, w / sqrt(2.0), L, false, true); break;  // DIN 557: one side
  case BK_CONE_NUT: {
    double narrow = 0.46 * w - f.seat * tanHalf(f.angle);
    body = fuse(revolve({{0, 0}, {narrow, 0}, {0.46 * w, f.seat}, {0, f.seat}}), raised(hexBlank(w, L - f.seat, true, true), f.seat));
    break;
  }
  default: body = hexBlank(w, L, true, true);
  }
  double sink = std::min({t.p * 0.6, L / 4, 0.6 * (face - r)});
  bool seat = f.kind == BK_CONE_NUT;
  TopoDS_Shape tap = kept(keyOf("tap", {d, t.p, L, sink, (double)seat}), [&] {
    TopoDS_Shape top = revolve({{0, L - sink}, {r, L - sink}, {r + sink + 1, L + 1}, {0, L + 1}});
    if (seat) return fuse(threadSolid(d, t.p, -1, L + 1), top);
    return fuseAll(threadSolid(d, t.p, -1, L + 1), {top, revolve({{0, -1}, {r + sink + 1, -1}, {r, sink}, {0, sink}})});
  });
  return cut(body, tap);
}

BKShape *bk_fastener(const BKFastener *f, double clearance) {
  return guarded(f && isNut(f->kind) ? "nut" : "bolt", [&]() -> TopoDS_Shape {
    need(f && std::isfinite(clearance), "sizes must be numbers");
    checkFit(*f);
    return centred(isNut(f->kind) ? nutShape(*f, clearance) : boltShape(*f, clearance));
  });
}

// MARK: - transforms and operations

BKShape *bk_transform(const BKShape *s, const double *m) {
  if (!s) return nullptr;
  return guarded("transform", [&]() -> TopoDS_Shape {
    need(finite(m, 12), "placement must be numbers");
    gp_Vec c0(m[0], m[4], m[8]), c1(m[1], m[5], m[9]), c2(m[2], m[6], m[10]);
    need(fabs(c0.Dot(c1.Crossed(c2))) > 1e-12, "placement flattens the shape");
    double l0 = c0.Magnitude(), l1 = c1.Magnitude(), l2 = c2.Magnitude();
    bool uniform = fabs(l0 - l1) < 1e-9 * l0 + 1e-12 && fabs(l0 - l2) < 1e-9 * l0 + 1e-12 && fabs(c0.Dot(c1)) < 1e-9 &&
                   fabs(c0.Dot(c2)) < 1e-9 && fabs(c1.Dot(c2)) < 1e-9;
    // A mesh-free copy: transforming a shape that was already meshed can carry stale mesh records
    // (polygons on a dropped triangulation) into the result.
    TopoDS_Shape clean = BRepBuilderAPI_Copy(s->shape, true, false).Shape();
    if (uniform) {
      gp_Trsf t;
      t.SetValues(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11]);
      return BRepBuilderAPI_Transform(clean, t, false).Shape();
    }
    gp_GTrsf g;
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 4; c++) g.SetValue(r + 1, c + 1, m[r * 4 + c]);
    return BRepBuilderAPI_GTransform(clean, g, false).Shape();
  });
}

BKShape *bk_boolean(int op, const BKShape *a, const BKShape *b) {
  if (!a || !b) return a ? bk_copy(a) : (b && op == BK_UNION ? bk_copy(b) : nullptr);
  return guarded(op == BK_UNION ? "merge" : op == BK_SUBTRACT ? "subtract" : "intersect", [&]() -> TopoDS_Shape {
    TopoDS_Shape r = op == BK_UNION ? fuse(a->shape, b->shape) : op == BK_SUBTRACT ? cut(a->shape, b->shape) : common(a->shape, b->shape);
    return unify(r);
  });
}

BKShape *bk_split(const BKShape *s, const double *p, const double *n, int side) {
  if (!s) return nullptr;
  return guarded("split", [&]() -> TopoDS_Shape {
    need(finite(p, 3) && finite(n, 3), "plane must be numbers");
    gp_Pnt o(p[0], p[1], p[2]);
    gp_Dir dir(n[0], n[1], n[2]);
    TopoDS_Face plane = BRepBuilderAPI_MakeFace(gp_Pln(o, dir)).Face();
    gp_Pnt ref = o.Translated(gp_Vec(dir) * (side == 0 ? 1.0 : -1.0));
    TopoDS_Shape half = BRepPrimAPI_MakeHalfSpace(plane, ref).Solid();
    return unify(common(s->shape, half));
  });
}

int bk_piece_count(const BKShape *s) { return s ? solidCount(s->shape) : 0; }

BKShape *bk_copy(const BKShape *s) { return s ? new BKShape{s->shape} : nullptr; }
void bk_free(BKShape *s) { delete s; }

// MARK: - rounding

// Parameter of the point halfway along a curve.
static double midParameter(const BRepAdaptor_Curve &c) {
  double f = c.FirstParameter(), mid = (f + c.LastParameter()) / 2;
  try {
    GCPnts_AbscissaPoint ap(c, GCPnts_AbscissaPoint::Length(c) / 2, f);
    if (ap.IsDone()) mid = ap.Parameter();
  } catch (...) {
  }
  return mid;
}

static gp_Pnt edgeMid(const TopoDS_Edge &e, gp_Vec *tangent) {
  BRepAdaptor_Curve c(e);
  gp_Pnt pt;
  gp_Vec d;
  c.D1(midParameter(c), pt, d);
  if (tangent) *tangent = d.Magnitude() > 1e-12 ? d.Normalized() : gp_Vec(0, 0, 1);
  return pt;
}

static void faceInfo(const TopoDS_Face &f, gp_Pnt &centroid, gp_Vec &normal) {
  GProp_GProps props;
  BRepGProp::SurfaceProperties(f, props);
  centroid = props.CentreOfMass();
  normal = gp_Vec(0, 0, 1);
  if (BRep_Tool::Surface(f).IsNull()) return;
  BRepAdaptor_Surface surf(f);
  double u = (surf.FirstUParameter() + surf.LastUParameter()) / 2, v = (surf.FirstVParameter() + surf.LastVParameter()) / 2;
  Handle(Geom_Surface) g = BRep_Tool::Surface(f);
  GeomAPI_ProjectPointOnSurf proj(centroid, g);
  if (proj.NbPoints() > 0) proj.LowerDistanceParameters(u, v);
  BRepLProp_SLProps lp(surf, u, v, 1, 1e-6);
  normal = lp.IsNormalDefined() ? gp_Vec(lp.Normal()) : gp_Vec(0, 0, 1);
  if (f.Orientation() == TopAbs_REVERSED) normal.Reverse();
}

// Parameter of the point of curve c nearest to p, kept off the ends of an open curve.
static double nearestParameter(const BRepAdaptor_Curve &c, const gp_Pnt &p) {
  double f = c.FirstParameter(), l = c.LastParameter(), t = f, best = c.Value(f).SquareDistance(p);
  if (c.Value(l).SquareDistance(p) < best) t = l, best = c.Value(l).SquareDistance(p);
  Extrema_ExtPC ext(p, c);
  if (ext.IsDone())
    for (int k = 1; k <= ext.NbExt(); k++)
      if (ext.SquareDistance(k) < best) t = ext.Point(k).Parameter(), best = ext.SquareDistance(k);
  if (!c.IsClosed()) t = std::min(std::max(t, f + (l - f) * 0.02), l - (l - f) * 0.02);
  return t;
}

// How far a stored pick may drift from the rebuilt geometry and still match.
static double pickTolerance(const TopoDS_Shape &shape) {
  Bnd_Box box;
  BRepBndLib::Add(shape, box);
  double diag = box.IsVoid() ? 10 : sqrt(box.SquareExtent());
  return std::max(0.3, diag * 0.02);
}

// The face whose centroid is nearest to c with a normal facing along n; failing that, the face c lies on (one made larger
// or smaller by a rounding or bevel at its edge has its middle elsewhere); null when none is within tol.
static TopoDS_Face findFace(const TopTools_IndexedMapOfShape &faces, const gp_Vec &n, const gp_Pnt &c, double tol) {
  double best = tol;
  int hit = 0;
  std::vector<bool> facing(faces.Extent() + 1, true);
  for (int k = 1; k <= faces.Extent(); k++) {
    gp_Pnt fc;
    gp_Vec fn;
    faceInfo(TopoDS::Face(faces(k)), fc, fn);
    facing[k] = n.Magnitude() < 1e-9 || fn.Dot(n.Normalized()) >= 0.7;
    double dist = fc.Distance(c) + (facing[k] ? 0 : tol);
    if (dist < best) best = dist, hit = k;
  }
  if (!hit) {
    TopoDS_Vertex at = BRepBuilderAPI_MakeVertex(c).Vertex();
    best = std::min(tol, 0.05);
    for (int k = 1; k <= faces.Extent(); k++) {
      if (!facing[k]) continue;
      BRepExtrema_DistShapeShape d(at, faces(k));
      if (d.IsDone() && d.Value() < best) best = d.Value(), hit = k;
    }
  }
  return hit ? TopoDS::Face(faces(hit)) : TopoDS_Face();
}

static void resolvePicks(const TopoDS_Shape &shape, const int *kinds, const double *picks, int count,
                         TopTools_IndexedMapOfShape &chosen, int *missing) {
  TopTools_IndexedMapOfShape edges, faces, vertices;
  TopExp::MapShapes(shape, TopAbs_EDGE, edges);
  TopExp::MapShapes(shape, TopAbs_FACE, faces);
  TopExp::MapShapes(shape, TopAbs_VERTEX, vertices);
  TopTools_IndexedDataMapOfShapeListOfShape vertexEdges;
  TopExp::MapShapesAndAncestors(shape, TopAbs_VERTEX, TopAbs_EDGE, vertexEdges);
  double tol = pickTolerance(shape);
  TopTools_IndexedDataMapOfShapeListOfShape edgeFaces;
  TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeFaces);
  // A real corner: not a seam, and its two faces don't already meet smoothly (as after an earlier rounding).
  auto sharp = [&](const TopoDS_Edge &e) {
    if (BRep_Tool::Degenerated(e)) return false;
    int i = edgeFaces.FindIndex(e);
    if (!i || edgeFaces(i).Extent() < 2) return i != 0;
    TopoDS_Face f1 = TopoDS::Face(edgeFaces(i).First()), f2 = TopoDS::Face(edgeFaces(i).Last());
    if (f1.IsSame(f2)) return false;
    return BRepLib::ContinuityOfFaces(e, f1, f2, 0.0175) < GeomAbs_G1;
  };
  *missing = 0;
  for (int i = 0; i < count; i++) {
    const double *q = picks + i * 6;
    gp_Pnt a(q[0], q[1], q[2]), b(q[3], q[4], q[5]);
    bool found = false;
    switch (kinds[i]) {
    case BK_PICK_BODY:
      for (int k = 1; k <= edges.Extent(); k++)
        if (sharp(TopoDS::Edge(edges(k)))) chosen.Add(edges(k));
      found = true;
      break;
    case BK_PICK_EDGE: {
      // The edge running through the picked point (its middle when picked), so one made shorter or longer by a rounding
      // or bevel at its end still matches.
      gp_Vec want(q[3], q[4], q[5]);
      double best = tol;
      int hit = 0;
      for (int k = 1; k <= edges.Extent(); k++) {
        TopoDS_Edge e = TopoDS::Edge(edges(k));
        if (BRep_Tool::Degenerated(e)) continue;
        BRepAdaptor_Curve c(e);
        gp_Pnt p;
        gp_Vec t;
        c.D1(nearestParameter(c, a), p, t);
        double dist = p.Distance(a);
        if (want.Magnitude() > 1e-9 && t.Magnitude() > 1e-12 && fabs(t.Normalized().Dot(want.Normalized())) < 0.8) dist += tol;
        if (dist < best) best = dist, hit = k;
      }
      if (hit) chosen.Add(edges(hit)), found = true;
      break;
    }
    case BK_PICK_CORNER: {
      gp_Vec n(q[0], q[1], q[2]);
      double best = tol;
      int hit = 0;
      for (int k = 1; k <= vertices.Extent(); k++) {
        double dist = BRep_Tool::Pnt(TopoDS::Vertex(vertices(k))).Distance(b);
        if (dist < best) best = dist, hit = k;
      }
      if (!hit || n.Magnitude() < 1e-9) break;
      n.Normalize();
      const TopoDS_Shape &v = vertices(hit);
      int index = vertexEdges.FindIndex(v);
      if (!index) break;
      double bestDot = 0.3;
      TopoDS_Shape pick;
      for (TopTools_ListOfShape::Iterator it(vertexEdges(index)); it.More(); it.Next()) {
        TopoDS_Edge e = TopoDS::Edge(it.Value());
        if (BRep_Tool::Degenerated(e)) continue;
        gp_Vec t;
        edgeMid(e, &t);
        double dot = fabs(t.Dot(n));
        if (dot > bestDot) bestDot = dot, pick = e;
      }
      if (!pick.IsNull()) chosen.Add(pick), found = true;
      break;
    }
    case BK_PICK_FACE: {
      TopoDS_Face face = findFace(faces, gp_Vec(q[0], q[1], q[2]), b, tol);
      if (!face.IsNull()) {
        for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next())
          if (sharp(TopoDS::Edge(ex.Current()))) chosen.Add(ex.Current());
        found = true;
      }
      break;
    }
    }
    if (!found) (*missing)++;
  }
}

static TopoDS_Shape filletWith(const TopoDS_Shape &shape, const TopTools_IndexedMapOfShape &edges, double r) {
  BRepFilletAPI_MakeFillet f(shape);
  for (int k = 1; k <= edges.Extent(); k++) f.Add(r, TopoDS::Edge(edges(k)));
  f.Build();
  if (!f.IsDone()) return TopoDS_Shape();
  TopoDS_Shape out = f.Shape();
  if (!BRepCheck_Analyzer(out).IsValid()) return TopoDS_Shape();
  return out;
}

// The largest radius below r (to 0.01 mm, found by bisection) for which attempt gives a shape; 0 if none.
template <typename F> static double largestWorking(double r, F attempt) {
  double lo = 0, hi = r;
  for (int i = 0; i < 7; i++) {
    double mid = (lo + hi) / 2;
    if (attempt(mid).IsNull()) hi = mid;
    else lo = mid;
  }
  return floor(lo * 100) / 100;
}

BKShape *bk_fillet(const BKShape *s, const int *kinds, const double *picks, int count, double radius, double *maxRadius, int *missing) {
  if (maxRadius) *maxRadius = radius;
  int miss = 0;
  if (missing) *missing = 0;
  if (!s) return nullptr;
  try {
    need(std::isfinite(radius) && finite(picks, count * 6), "rounding must be numbers");
    TopTools_IndexedMapOfShape edges;
    resolvePicks(s->shape, kinds, picks, count, edges, &miss);
    if (missing) *missing = miss;
    if (edges.IsEmpty() || radius < 0.005) return bk_copy(s);
    auto attempt = [&](double r) {
      try {
        return filletWith(s->shape, edges, r);
      } catch (...) {
        return TopoDS_Shape();
      }
    };
    TopoDS_Shape out = attempt(radius);
    if (!out.IsNull()) return wrap(out);
    if (maxRadius) *maxRadius = largestWorking(radius, attempt);
    lastError = "rounding too large";
  } catch (Standard_Failure &e) {
    lastError = e.GetMessageString() ? e.GetMessageString() : "rounding failed";
  } catch (...) {
    lastError = "rounding failed";
  }
  return nullptr;
}

// MARK: - edges between two faces

// An edge between two faces, seen at one of its points. Face A is the one whose outward normal at the edge's middle
// points most up (ties: most +y, then +x); the "into" directions run from the edge across it into each face.
struct Crease {
  TopoDS_Edge edge;
  TopoDS_Face a, b;
  gp_Pnt point;
  gp_Vec tangent, normalA, normalB, intoA, intoB;
  double angle; // material angle between the faces in degrees, above 180 for a concave edge
};

// Outward unit normal of face f at parameter t of its edge e.
static gp_Vec normalOn(const TopoDS_Face &f, const TopoDS_Edge &e, double t) {
  double first, last, u, v;
  Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(e, f, first, last);
  if (!pc.IsNull()) {
    gp_Pnt2d uv = pc->Value(t);
    u = uv.X(), v = uv.Y();
  } else {
    GeomAPI_ProjectPointOnSurf proj(BRepAdaptor_Curve(e).Value(t), BRep_Tool::Surface(f));
    if (proj.NbPoints() == 0) return gp_Vec(0, 0, 1);
    proj.LowerDistanceParameters(u, v);
  }
  BRepAdaptor_Surface surf(f);
  BRepLProp_SLProps lp(surf, u, v, 1, 1e-6);
  gp_Vec n = lp.IsNormalDefined() ? gp_Vec(lp.Normal()) : gp_Vec(0, 0, 1);
  if (f.Orientation() == TopAbs_REVERSED) n.Reverse();
  return n;
}

// Orientation of edge e in face f; EXTERNAL when it isn't a plain boundary edge of it (a seam appears twice).
static TopAbs_Orientation orientationIn(const TopoDS_Edge &e, const TopoDS_Face &f) {
  TopAbs_Orientation o = TopAbs_EXTERNAL;
  int n = 0;
  for (TopExp_Explorer ex(f, TopAbs_EDGE); ex.More(); ex.Next())
    if (ex.Current().IsSame(e)) o = ex.Current().Orientation(), n++;
  return n == 1 && (o == TopAbs_FORWARD || o == TopAbs_REVERSED) ? o : TopAbs_EXTERNAL;
}

// Describes edge e at its point nearest to `at` (its middle when null); false unless it lies between two distinct faces.
static bool crease(const TopoDS_Edge &e, const TopTools_IndexedDataMapOfShapeListOfShape &edgeFaces, const gp_Pnt *at, Crease &c) {
  int i = edgeFaces.FindIndex(e);
  if (BRep_Tool::Degenerated(e) || !i || edgeFaces(i).Extent() != 2) return false;
  TopoDS_Face f1 = TopoDS::Face(edgeFaces(i).First()), f2 = TopoDS::Face(edgeFaces(i).Last());
  if (f1.IsSame(f2) || orientationIn(e, f1) == TopAbs_EXTERNAL || orientationIn(e, f2) == TopAbs_EXTERNAL) return false;
  BRepAdaptor_Curve curve(e);
  double mid = midParameter(curve), t = at ? nearestParameter(curve, *at) : mid;
  gp_Vec n1 = normalOn(f1, e, mid), n2 = normalOn(f2, e, mid), d;
  bool firstUp = fabs(n1.Z() - n2.Z()) >= 1e-6 ? n1.Z() > n2.Z() : fabs(n1.Y() - n2.Y()) >= 1e-6 ? n1.Y() > n2.Y() : n1.X() > n2.X();
  curve.D1(t, c.point, d);
  if (d.Magnitude() < 1e-12) return false;
  c.edge = e;
  c.a = firstUp ? f1 : f2;
  c.b = firstUp ? f2 : f1;
  c.tangent = d.Normalized();
  // A face lies to the left of its edges seen from outside, so normal × (edge direction as the face runs it) points into it.
  auto across = [&](const TopoDS_Face &f, const gp_Vec &n) {
    gp_Vec w = n.Crossed(orientationIn(e, f) == TopAbs_REVERSED ? c.tangent.Reversed() : c.tangent);
    return w.Magnitude() > 1e-12 ? w.Normalized() : w;
  };
  c.normalA = normalOn(c.a, e, t);
  c.normalB = normalOn(c.b, e, t);
  c.intoA = across(c.a, c.normalA);
  c.intoB = across(c.b, c.normalB);
  // Face B's heading in the end-on frame (x = -intoA, y = normalA), counted from face A's heading along -x.
  double deg = atan2(c.intoB.Dot(c.normalA), -c.intoB.Dot(c.intoA)) * 180 / M_PI - 180;
  c.angle = deg <= 0 ? deg + 360 : deg;
  return true;
}

// The given edges that lie between two faces.
static std::vector<Crease> creases(const TopoDS_Shape &shape, const TopTools_IndexedMapOfShape &edges) {
  TopTools_IndexedDataMapOfShapeListOfShape edgeFaces;
  TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeFaces);
  std::vector<Crease> out;
  for (int k = 1; k <= edges.Extent(); k++) {
    Crease c;
    if (crease(TopoDS::Edge(edges(k)), edgeFaces, nullptr, c)) out.push_back(c);
  }
  return out;
}

// MARK: - bevel

// Bevels the creases (legA along face A); corner > 0 also rounds each bevel's edges with its faces A and B.
static TopoDS_Shape chamferWith(const TopoDS_Shape &shape, const std::vector<Crease> &creases, double legA, double legB, double corner) {
  BRepFilletAPI_MakeChamfer c(shape);
  for (const Crease &k : creases) c.Add(legA, legB, k.edge, k.a);
  c.Build();
  if (!c.IsDone() || !BRepCheck_Analyzer(c.Shape()).IsValid()) throw Standard_Failure("too large for these edges");
  TopoDS_Shape out = c.Shape();
  if (corner < 0.005) return out;
  // The edges to round: those a bevel face (generated from a picked edge) shares with a face made from that edge's A or B.
  TopTools_IndexedDataMapOfShapeListOfShape edgeFaces;
  TopExp::MapShapesAndAncestors(out, TopAbs_EDGE, TopAbs_FACE, edgeFaces);
  TopTools_IndexedMapOfShape round;
  for (const Crease &k : creases) {
    TopTools_IndexedMapOfShape sides;
    for (const TopoDS_Face &f : {k.a, k.b}) {
      TopTools_ListOfShape made = c.Modified(f);
      if (made.IsEmpty()) sides.Add(f);
      for (TopTools_ListOfShape::Iterator it(made); it.More(); it.Next()) sides.Add(it.Value());
    }
    TopTools_ListOfShape bevels = c.Generated(k.edge);
    for (TopTools_ListOfShape::Iterator it(bevels); it.More(); it.Next())
      for (TopExp_Explorer ex(it.Value(), TopAbs_EDGE); ex.More(); ex.Next()) {
        int i = edgeFaces.FindIndex(ex.Current());
        if (!i) continue;
        for (TopTools_ListOfShape::Iterator f(edgeFaces(i)); f.More(); f.Next())
          if (sides.Contains(f.Value())) round.Add(ex.Current());
      }
  }
  TopoDS_Shape rounded;
  try {
    rounded = filletWith(out, round, corner);
  } catch (...) {
  }
  if (rounded.IsNull()) throw Standard_Failure("corner radius too large");
  return rounded;
}

BKShape *bk_chamfer(const BKShape *s, const int *kinds, const double *picks, int count, double legA, double legB, double cornerRadius,
                    int *missing) {
  if (missing) *missing = 0;
  if (!s) return nullptr;
  return guarded("bevel", [&]() -> TopoDS_Shape {
    need(std::isfinite(legA) && std::isfinite(legB) && std::isfinite(cornerRadius) && finite(picks, count * 6), "bevel must be numbers");
    TopTools_IndexedMapOfShape edges;
    int miss = 0;
    resolvePicks(s->shape, kinds, picks, count, edges, &miss);
    if (missing) *missing = miss;
    std::vector<Crease> sharp;
    for (const Crease &c : creases(s->shape, edges))
      if (fabs(c.angle - 180) > 1) sharp.push_back(c);
    if (sharp.empty() || std::min(legA, legB) < 0.005) return s->shape;
    return chamferWith(s->shape, sharp, legA, legB, cornerRadius);
  });
}

// MARK: - cove

// Solid tube of radius r around edge e, as long as the edge: a cylinder, a torus (segment) or a swept circle. Booleans
// cope with its caps lying flush on the faces at the edge's ends; running the tubes on past the ends only slowed them.
static TopoDS_Shape tube(const TopoDS_Edge &e, double r) {
  BRepAdaptor_Curve c(e);
  double f = c.FirstParameter(), l = c.LastParameter();
  if (c.GetType() == GeomAbs_Line) {
    gp_Pnt p0 = c.Value(f), p1 = c.Value(l);
    return BRepPrimAPI_MakeCylinder(gp_Ax2(p0, gp_Dir(gp_Vec(p0, p1))), r, p0.Distance(p1)).Shape();
  }
  if (c.GetType() == GeomAbs_Circle) {
    gp_Circ circle = c.Circle();
    if (r >= circle.Radius()) throw Standard_Failure("cove wider than its arc");
    gp_Ax2 axes = circle.Position();
    if (l - f >= 2 * M_PI - 1e-9) return BRepPrimAPI_MakeTorus(axes, circle.Radius(), r).Shape();
    axes.Rotate(axes.Axis(), f);
    return BRepPrimAPI_MakeTorus(axes, circle.Radius(), r, l - f).Shape();
  }
  gp_Pnt p;
  gp_Vec d;
  c.D1(f, p, d);
  BRepOffsetAPI_MakePipeShell pipe(BRepBuilderAPI_MakeWire(TopoDS::Edge(e.Oriented(TopAbs_FORWARD))).Wire());
  pipe.Add(BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(gp_Circ(gp_Ax2(p, gp_Dir(d)), r)).Edge()).Wire());
  pipe.Build();
  if (!pipe.IsDone() || !pipe.MakeSolid()) throw Standard_Failure("cove sweep failed");
  return pipe.Shape();
}

// The shape with coves of radius r cut along the creases, or null when they don't fit: a face beside a cove vanishes,
// a cove reaches a face that doesn't touch its edge, or the part falls apart.
static TopoDS_Shape coveWith(const TopoDS_Shape &shape, const std::vector<Crease> &creases, double r) {
  TopTools_IndexedDataMapOfShapeListOfShape vertexFaces;
  TopExp::MapShapesAndAncestors(shape, TopAbs_VERTEX, TopAbs_FACE, vertexFaces);
  TopTools_IndexedMapOfShape beside, near;
  TopTools_ListOfShape tubes;
  for (const Crease &c : creases) {
    beside.Add(c.a);
    beside.Add(c.b);
    for (TopExp_Explorer ex(c.edge, TopAbs_VERTEX); ex.More(); ex.Next()) {
      int i = vertexFaces.FindIndex(ex.Current());
      if (i)
        for (TopTools_ListOfShape::Iterator it(vertexFaces(i)); it.More(); it.Next()) near.Add(it.Value());
    }
    tubes.Append(tube(c.edge, r));
  }
  TopTools_ListOfShape args;
  args.Append(shape);
  BRepAlgoAPI_Cut op;
  op.SetArguments(args);
  op.SetTools(tubes);
  op.Build();
  if (!op.IsDone()) return TopoDS_Shape();
  TopTools_IndexedMapOfShape faces;
  TopExp::MapShapes(shape, TopAbs_FACE, faces);
  for (int k = 1; k <= faces.Extent(); k++) {
    const TopoDS_Shape &f = faces(k);
    bool touched = op.IsDeleted(f) || !op.Modified(f).IsEmpty();
    if (beside.Contains(f) ? op.IsDeleted(f) : touched && !near.Contains(f)) return TopoDS_Shape();
  }
  TopoDS_Shape out = unify(op.Shape());
  if (!BRepCheck_Analyzer(out).IsValid() || solidCount(out) != solidCount(shape)) return TopoDS_Shape();
  return out;
}

BKShape *bk_cove(const BKShape *s, const int *kinds, const double *picks, int count, double radius, double *maxRadius, int *missing) {
  if (maxRadius) *maxRadius = radius;
  if (missing) *missing = 0;
  if (!s) return nullptr;
  try {
    need(std::isfinite(radius) && finite(picks, count * 6), "rounding must be numbers");
    TopTools_IndexedMapOfShape edges;
    int miss = 0;
    resolvePicks(s->shape, kinds, picks, count, edges, &miss);
    if (missing) *missing = miss;
    std::vector<Crease> convex;
    for (const Crease &c : creases(s->shape, edges))
      if (c.angle < 179) convex.push_back(c);
    if (convex.empty() || radius < 0.005) return bk_copy(s);
    auto attempt = [&](double r) {
      try {
        return coveWith(s->shape, convex, r);
      } catch (...) {
        return TopoDS_Shape();
      }
    };
    TopoDS_Shape out = attempt(radius);
    if (!out.IsNull()) return wrap(out);
    if (maxRadius) *maxRadius = largestWorking(radius, attempt);
    lastError = "cove too large";
  } catch (Standard_Failure &e) {
    lastError = e.GetMessageString() ? e.GetMessageString() : "cove failed";
  } catch (...) {
    lastError = "cove failed";
  }
  return nullptr;
}

// MARK: - section

BKSection *bk_section(const BKShape *s, int kind, const double *pick, double radius) {
  if (!s) return nullptr;
  try {
    const TopoDS_Shape &shape = s->shape;
    const double none[6] = {0, 0, 0, 0, 0, 0};
    const double *q = pick ? pick : none;
    need(std::isfinite(radius) && finite(q, 6), "pick must be numbers");
    TopTools_IndexedMapOfShape edges;
    int miss = 0;
    resolvePicks(shape, &kind, q, 1, edges, &miss);
    TopTools_IndexedDataMapOfShapeListOfShape edgeFaces;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeFaces);
    gp_Pnt at(q[0], q[1], q[2]);
    Crease c;
    double longest = -1;
    for (int k = 1; k <= edges.Extent(); k++) {
      TopoDS_Edge e = TopoDS::Edge(edges(k));
      Crease x;
      if (!crease(e, edgeFaces, kind == BK_PICK_EDGE ? &at : nullptr, x) || (kind == BK_PICK_BODY && x.angle >= 179)) continue;
      double length = GCPnts_AbscissaPoint::Length(BRepAdaptor_Curve(e));
      if (length > longest) longest = length, c = x;
    }
    if (longest < 0) {
      lastError = "no edge between two faces here";
      return nullptr;
    }
    // The cut: the solid's common part with a piece of the plane across the edge larger than the solid, merged into
    // whole faces (a plane lying on one of the solid's faces cuts it in pieces).
    Bnd_Box box;
    BRepBndLib::Add(shape, box);
    double size = box.IsVoid() ? 1000 : sqrt(box.SquareExtent()) * 2 + 10;
    TopoDS_Face plane = BRepBuilderAPI_MakeFace(gp_Pln(c.point, gp_Dir(c.tangent)), -size, size, -size, size).Face();
    TopoDS_Shape cutFace = unify(common(shape, plane));
    gp_Vec x = -c.intoA, y = c.normalA - x * c.normalA.Dot(x);
    y.Normalize();
    double deflection = std::max((radius > 0 ? radius : size / 20) / 400, 1e-4);
    std::vector<double> points;
    std::vector<int> starts{0};
    for (TopExp_Explorer fx(cutFace, TopAbs_FACE); fx.More(); fx.Next()) {
      TopoDS_Face face = TopoDS::Face(fx.Current());
      TopoDS_Wire outer = BRepTools::OuterWire(face);
      for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        std::vector<gp_Pnt2d> loop;
        for (BRepTools_WireExplorer we(TopoDS::Wire(wx.Current()), face); we.More(); we.Next()) {
          const TopoDS_Edge &e = we.Current();
          if (BRep_Tool::Degenerated(e)) continue;
          BRepAdaptor_Curve curve(e);
          std::vector<gp_Pnt> run;
          GCPnts_QuasiUniformDeflection sample(curve, deflection);
          if (sample.IsDone())
            for (int i = 1; i <= sample.NbPoints(); i++) run.push_back(sample.Value(i));
          else run = {curve.Value(curve.FirstParameter()), curve.Value(curve.LastParameter())};
          if (e.Orientation() == TopAbs_REVERSED) std::reverse(run.begin(), run.end());
          for (size_t i = 0; i + 1 < run.size(); i++) {
            gp_Vec d(c.point, run[i]);
            loop.emplace_back(d.Dot(x), d.Dot(y));
          }
        }
        if (loop.size() < 3) continue;
        // Outlines run counter-clockwise, holes clockwise.
        double area = 0;
        for (size_t i = 0, n = loop.size(); i < n; i++) area += loop[i].X() * loop[(i + 1) % n].Y() - loop[(i + 1) % n].X() * loop[i].Y();
        if ((area > 0) != wx.Current().IsSame(outer)) std::reverse(loop.begin(), loop.end());
        for (const gp_Pnt2d &pt : loop) points.insert(points.end(), {pt.X(), pt.Y()});
        starts.push_back((int)(points.size() / 2));
      }
    }
    if (starts.size() < 2) {
      lastError = "the cut across this edge is empty";
      return nullptr;
    }
    BKSection *out = new BKSection();
    out->loopCount = (int)starts.size() - 1;
    out->pointCount = (int)(points.size() / 2);
    out->points = mallocCopy(points);
    out->loopStart = mallocCopy(starts);
    out->angle = c.angle;
    auto describe = [](const TopoDS_Face &face, double *info) {
      gp_Pnt centroid;
      gp_Vec normal;
      faceInfo(face, centroid, normal);
      for (int i = 0; i < 3; i++) info[i] = normal.Coord(i + 1), info[i + 3] = centroid.Coord(i + 1);
    };
    describe(c.a, out->faceA);
    describe(c.b, out->faceB);
    gp_Vec z = x.Crossed(y);
    for (int i = 0; i < 3; i++) out->point[i] = c.point.Coord(i + 1), out->direction[i] = z.Coord(i + 1);
    return out;
  } catch (Standard_Failure &e) {
    lastError = std::string("section: ") + (e.GetMessageString() ? e.GetMessageString() : "OpenCascade error");
  } catch (...) {
    lastError = "section failed";
  }
  return nullptr;
}

void bk_section_free(BKSection *section) {
  if (!section) return;
  free(section->points);
  free(section->loopStart);
  delete section;
}

// MARK: - hollow

static double volumeOf(const TopoDS_Shape &x) {
  GProp_GProps g;
  BRepGProp::VolumeProperties(x, g);
  return g.Mass();
}

// How far a shape reaches below a plane (point c, outward normal n), by its bounding box.
static double depth(const TopoDS_Shape &x, const gp_Pnt &c, const gp_Vec &n) {
  Bnd_Box box;
  BRepBndLib::Add(x, box);
  double x0, y0, z0, x1, y1, z1, most = 0;
  box.Get(x0, y0, z0, x1, y1, z1);
  for (double px : {x0, x1})
    for (double py : {y0, y1})
      for (double pz : {z0, z1}) most = std::max(most, gp_Vec(gp_Pnt(px, py, pz), c).Dot(n.Normalized()));
  return most;
}

// The shape hollowed with these openings and own walls (faces of it); null when no way of offsetting fits.
static TopoDS_Shape hollowWith(const TopoDS_Shape &shape, const std::vector<TopoDS_Face> &openings,
                               const std::vector<std::pair<TopoDS_Face, double>> &own, double thickness) {
  double full = volumeOf(shape);
  // Sharp inner corners first; merged shapes often only offset with arc joins and self-intersection handling.
  const std::pair<GeomAbs_JoinType, bool> modes[] = {{GeomAbs_Intersection, false}, {GeomAbs_Arc, true}, {GeomAbs_Arc, false}};
  for (auto [join, selfCross] : modes) {
    try {
      BRepOffset_MakeOffset offset;
      offset.Initialize(shape, -std::max(thickness, 0.01), 1e-4, BRepOffset_Skin, selfCross, false, join, false, true);
      for (auto &f : openings) offset.AddFace(f);
      for (auto &[f, t] : own) offset.SetOffsetOnFace(f, -t);
      TopoDS_Shape out;
      if (openings.empty()) {
        // Closed hollow: the shape minus its inward offset (the core).
        offset.MakeOffsetShape();
        if (!offset.IsDone() || offset.Shape().IsNull()) continue;
        TopoDS_Shape core = offset.Shape();
        if (core.ShapeType() == TopAbs_SHELL) core = BRepBuilderAPI_MakeSolid(TopoDS::Shell(core)).Solid();
        double inner = volumeOf(core);
        if (inner <= 0 || inner >= full) continue;
        out = cut(shape, core);
      } else {
        offset.MakeThickSolid();
        if (!offset.IsDone() || offset.Shape().IsNull()) continue;
        out = offset.Shape();
      }
      double v = volumeOf(out);
      if (v > 0 && v < full * 0.999 && BRepCheck_Analyzer(out).IsValid()) return out;
    } catch (Standard_Failure &) {
    }
  }
  return TopoDS_Shape();
}

BKShape *bk_hollow(const BKShape *s, const BKShape *const *sharp, int sharpCount, const double *open, int openCount, const double *walls,
                   const double *wallThickness, int wallCount, double thickness, int *missing) {
  if (missing) *missing = 0;
  if (!s) return nullptr;
  return guarded("hollow", [&]() -> TopoDS_Shape {
    need(std::isfinite(thickness) && finite(open, openCount * 6) && finite(walls, wallCount * 6) && finite(wallThickness, wallCount),
         "walls must be numbers");
    // The picked faces on a shape; lost counts those it doesn't have.
    auto picked = [&](const TopoDS_Shape &x, std::vector<TopoDS_Face> &openings, std::vector<std::pair<TopoDS_Face, double>> &own, int &lost) {
      TopTools_IndexedMapOfShape faces;
      TopExp::MapShapes(x, TopAbs_FACE, faces);
      double tol = pickTolerance(x);
      auto face = [&](const double *q) { return findFace(faces, gp_Vec(q[0], q[1], q[2]), gp_Pnt(q[3], q[4], q[5]), tol); };
      for (int i = 0; i < openCount; i++) {
        TopoDS_Face f = face(open + i * 6);
        if (f.IsNull()) lost++;
        else openings.push_back(f);
      }
      for (int i = 0; i < wallCount; i++) {
        TopoDS_Face f = face(walls + i * 6);
        if (f.IsNull()) lost++;
        else own.emplace_back(f, std::max(wallThickness[i], 0.01));
      }
    };
    const TopoDS_Shape &shape = s->shape;
    std::vector<TopoDS_Face> openings;
    std::vector<std::pair<TopoDS_Face, double>> own;
    int miss = 0;
    picked(shape, openings, own, miss);
    if (missing) *missing = miss;
    // Roundings no thicker than the walls leave nothing to shrink inward, and an opening edged by roundings doesn't
    // offset at all: the shape without them is hollowed instead, and only what lies inside the rounded shape is kept,
    // so the outside stays rounded.
    double full = volumeOf(shape);
    auto within = [&](const BKShape *k) -> TopoDS_Shape {
      if (!k) return TopoDS_Shape();
      std::vector<TopoDS_Face> sharpOpenings;
      std::vector<std::pair<TopoDS_Face, double>> sharpOwn;
      int lost = 0;
      picked(k->shape, sharpOpenings, sharpOwn, lost);
      if (lost > miss) return TopoDS_Shape();
      try {
        TopoDS_Shape hollowed = hollowWith(k->shape, sharpOpenings, sharpOwn, thickness);
        if (hollowed.IsNull()) return TopoDS_Shape();
        // A sharp inside corner mustn't break through a big rounding outside: what of the hollow lies outside the
        // rounded shape has to be a lowered rim (at an opening, and shallow), not a hole further down.
        TopoDS_Shape outside = cut(cut(k->shape, hollowed), shape);
        for (TopExp_Explorer ex(outside, TopAbs_SOLID); ex.More(); ex.Next()) {
          bool rim = false;
          for (auto &f : sharpOpenings) {
            gp_Pnt c;
            gp_Vec n;
            faceInfo(f, c, n);
            rim = rim || (BRepExtrema_DistShapeShape(ex.Current(), f).Value() < 1e-3 && depth(ex.Current(), c, n) < depth(k->shape, c, n) / 2);
          }
          if (!rim) return TopoDS_Shape();
        }
        TopoDS_Shape out = common(shape, hollowed);
        double v = volumeOf(out);
        if (v > 0 && v < full * 0.999 && BRepCheck_Analyzer(out).IsValid()) return out;
      } catch (Standard_Failure &) {
      }
      return TopoDS_Shape();
    };
    TopoDS_Shape out;
    if (!own.empty() && sharpCount > 0) {
      // A face with a wall of its own offsets wrongly beside any rounding (a solid comes out, but not with the walls
      // asked for): only the shape without roundings is hollowed then.
      out = within(sharp[sharpCount - 1]);
    } else {
      out = hollowWith(shape, openings, own, thickness);
      for (int k = 0; k < sharpCount && out.IsNull(); k++) out = within(sharp[k]);
    }
    if (!out.IsNull()) return out;
    throw Standard_Failure("the walls don't fit this shape");
  });
}

// MARK: - meshing

BKMesh *bk_mesh(const BKShape *s, double deflection) {
  if (!s) return nullptr;
  BKMesh *m = new BKMesh();
  try {
    const TopoDS_Shape &shape = s->shape;
    deflection = std::isfinite(deflection) ? std::max(deflection, 0.001) : 0.05;
    BRepMesh_IncrementalMesh(shape, deflection, false, 0.35, true);
    TopTools_IndexedMapOfShape faces, edges, vertices;
    TopExp::MapShapes(shape, TopAbs_FACE, faces);
    TopExp::MapShapes(shape, TopAbs_EDGE, edges);
    TopExp::MapShapes(shape, TopAbs_VERTEX, vertices);
    std::vector<float> pos, nrm;
    std::vector<uint32_t> idx, triFace;
    std::vector<double> info;
    for (int fi = 1; fi <= faces.Extent(); fi++) {
      TopoDS_Face face = TopoDS::Face(faces(fi));
      gp_Pnt c;
      gp_Vec fn;
      faceInfo(face, c, fn);
      info.insert(info.end(), {fn.X(), fn.Y(), fn.Z(), c.X(), c.Y(), c.Z()});
      TopLoc_Location loc;
      Handle(Poly_Triangulation) tri;
      if (!BRep_Tool::Surface(face).IsNull()) tri = BRep_Tool::Triangulation(face, loc);
      if (tri.IsNull()) continue;
      if (!tri->HasNormals()) BRepLib_ToolTriangulatedShape::ComputeNormals(face, tri);
      bool rev = face.Orientation() == TopAbs_REVERSED;
      gp_Trsf tr = loc.Transformation();
      uint32_t base = (uint32_t)(pos.size() / 3);
      for (int i = 1; i <= tri->NbNodes(); i++) {
        gp_Pnt p = tri->Node(i).Transformed(tr);
        gp_Dir d = tri->HasNormals() ? tri->Normal(i) : gp_Dir(0, 0, 1);
        gp_Vec n = gp_Vec(d).Transformed(tr);
        if (rev) n.Reverse();
        pos.insert(pos.end(), {(float)p.X(), (float)p.Y(), (float)p.Z()});
        nrm.insert(nrm.end(), {(float)n.X(), (float)n.Y(), (float)n.Z()});
      }
      for (int i = 1; i <= tri->NbTriangles(); i++) {
        int a, b, c2;
        tri->Triangle(i).Get(a, b, c2);
        if (rev) std::swap(b, c2);
        idx.insert(idx.end(), {base + a - 1, base + b - 1, base + c2 - 1});
        triFace.push_back(fi - 1);
      }
    }
    std::vector<float> ep;
    std::vector<uint32_t> es{0};
    std::vector<int32_t> ef;
    std::vector<double> circles;
    TopTools_IndexedDataMapOfShapeListOfShape edgeFaces;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeFaces);
    for (int ei = 1; ei <= edges.Extent(); ei++) {
      TopoDS_Edge e = TopoDS::Edge(edges(ei));
      if (!BRep_Tool::Degenerated(e)) {
        BRepAdaptor_Curve curve(e);
        GCPnts_QuasiUniformDeflection pts(curve, deflection);
        if (pts.IsDone())
          for (int i = 1; i <= pts.NbPoints(); i++) {
            gp_Pnt p = pts.Value(i);
            ep.insert(ep.end(), {(float)p.X(), (float)p.Y(), (float)p.Z()});
          }
        if (curve.GetType() == GeomAbs_Circle && curve.LastParameter() - curve.FirstParameter() >= M_PI - 1e-6) {
          gp_Circ c = curve.Circle();
          gp_Pnt o = c.Location();
          gp_Dir n = c.Axis().Direction();
          circles.insert(circles.end(), {o.X(), o.Y(), o.Z(), n.X(), n.Y(), n.Z(), c.Radius()});
        }
      }
      es.push_back((uint32_t)(ep.size() / 3));
      int32_t side[2] = {-1, -1}, k = 0;
      int i = edgeFaces.FindIndex(e);
      if (i)
        for (TopTools_ListOfShape::Iterator it(edgeFaces(i)); it.More() && k < 2; it.Next()) {
          int f = faces.FindIndex(it.Value()) - 1;
          if (k == 1 && f == side[0]) continue;
          side[k++] = f;
        }
      ef.insert(ef.end(), {side[0], side[1]});
    }
    std::vector<float> cs;
    for (int vi = 1; vi <= vertices.Extent(); vi++) {
      gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(vertices(vi)));
      cs.insert(cs.end(), {(float)p.X(), (float)p.Y(), (float)p.Z()});
    }
    m->vertexCount = (int)(pos.size() / 3);
    m->triangleCount = (int)(idx.size() / 3);
    m->positions = mallocCopy(pos);
    m->normals = mallocCopy(nrm);
    m->indices = mallocCopy(idx);
    m->triangleFace = mallocCopy(triFace);
    m->faceCount = faces.Extent();
    m->faceInfo = mallocCopy(info);
    m->edgeCount = edges.Extent();
    m->edgePointCount = (int)(ep.size() / 3);
    m->edgePoints = mallocCopy(ep);
    m->edgeStart = mallocCopy(es);
    m->edgeFaces = mallocCopy(ef);
    m->circleCount = (int)(circles.size() / 7);
    m->circles = mallocCopy(circles);
    m->cornerCount = vertices.Extent();
    m->corners = mallocCopy(cs);
    Bnd_Box box;
    BRepBndLib::AddOptimal(shape, box, false, false);
    if (!box.IsVoid()) box.Get(m->bbox[0], m->bbox[1], m->bbox[2], m->bbox[3], m->bbox[4], m->bbox[5]);
    GProp_GProps props;
    BRepGProp::VolumeProperties(shape, props);
    m->volume = props.Mass();
    m->valid = BRepCheck_Analyzer(shape).IsValid() ? 1 : 0;
  } catch (...) {
    lastError = "meshing failed";
  }
  return m;
}

void bk_mesh_free(BKMesh *m) {
  if (!m) return;
  free(m->positions);
  free(m->normals);
  free(m->indices);
  free(m->triangleFace);
  free(m->faceInfo);
  free(m->edgePoints);
  free(m->edgeStart);
  free(m->edgeFaces);
  free(m->circles);
  free(m->corners);
  delete m;
}

// MARK: - export

int bk_export_step(const BKShape *const *shapes, int count, const char *path) {
  try {
    STEPControl_Writer writer;
    Interface_Static::SetCVal("write.step.unit", "MM");
    Interface_Static::SetCVal("write.step.schema", "AP214IS");
    for (int i = 0; i < count; i++)
      if (shapes[i] && writer.Transfer(shapes[i]->shape, STEPControl_AsIs) != IFSelect_RetDone) {
        lastError = "STEP transfer failed";
        return 0;
      }
    if (writer.Write(path) != IFSelect_RetDone) {
      lastError = "STEP write failed";
      return 0;
    }
    return 1;
  } catch (...) {
    lastError = "STEP export failed";
    return 0;
  }
}
