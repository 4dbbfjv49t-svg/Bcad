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
      double tube = p[1] / 2, ring = p[0] / 2 - tube;
      if (ring <= tube * 0.05) ring = tube * 1.05;
      return BRepPrimAPI_MakeTorus(ring, tube).Shape();
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
      // The ellipse with conjugate semi-diameters u = (a, 0) and v = b·(cos t, sin t) is [u v] applied to the unit circle;
      // its semi-axes are the square roots of the eigenvalues of [u v][u v]ᵀ, along the eigenvectors.
      double a = std::max(p[0], 0.01) / 2, b = std::max(p[1], 0.01) / 2, t = std::min(std::max(p[2], 5.0), 175.0) * M_PI / 180;
      double sxx = a * a + b * b * cos(t) * cos(t), sxy = b * b * cos(t) * sin(t), syy = b * b * sin(t) * sin(t);
      double mean = (sxx + syy) / 2, spread = hypot((sxx - syy) / 2, sxy);
      double major = sqrt(mean + spread), minor = sqrt(std::max(mean - spread, 0.0)), phi = atan2(2 * sxy, sxx - syy) / 2;
      gp_Ax2 axes(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1), gp_Dir(cos(phi), sin(phi), 0));
      TopoDS_Edge rim = major - minor <= 1e-9 * major ? BRepBuilderAPI_MakeEdge(gp_Circ(axes, major)).Edge()
                                                      : BRepBuilderAPI_MakeEdge(gp_Elips(axes, major, minor)).Edge();
      TopoDS_Face base = BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(rim).Wire()).Face();
      return centred(BRepPrimAPI_MakePrism(base, gp_Vec(0, 0, p[3])).Shape());
    }
    }
    return TopoDS_Shape();
  });
}

// MARK: - threads (ISO 261 coarse, ISO 4017 hex bolts, ISO 4032 hex nuts)

struct ThreadSize {
  const char *name;
  double d, p, s, k, m, length;
};
static const ThreadSize sizes[] = {
    {"M3", 3, 0.5, 5.5, 2.0, 2.4, 12},       {"M4", 4, 0.7, 7, 2.8, 3.2, 16},
    {"M5", 5, 0.8, 8, 3.5, 4.7, 20},         {"M6", 6, 1.0, 10, 4.0, 5.2, 25},
    {"M8", 8, 1.25, 13, 5.3, 6.8, 30},       {"M10", 10, 1.5, 16, 6.4, 8.4, 35},
    {"M12", 12, 1.75, 18, 7.5, 10.8, 40},    {"M14", 14, 2.0, 21, 8.8, 12.8, 45},
    {"M16", 16, 2.0, 24, 10.0, 14.8, 50},    {"M18", 18, 2.5, 27, 11.5, 15.8, 55},
    {"M20", 20, 2.5, 30, 12.5, 18.0, 60},    {"M22", 22, 2.5, 34, 14.0, 19.4, 65},
    {"M24", 24, 3.0, 36, 15.0, 21.5, 70},
};
static const int sizeCount = sizeof(sizes) / sizeof(sizes[0]);

int bk_thread_count(void) { return sizeCount; }
const char *bk_thread_name(int size) { return size >= 0 && size < sizeCount ? sizes[size].name : ""; }
double bk_thread_default_length(int size, int nut) {
  if (size < 0 || size >= sizeCount) return 10;
  return nut ? sizes[size].m : sizes[size].length;
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
  double rMaj = d / 2, rMin = rMaj - 0.5412658774 * p, z0 = zlo - p;
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

// External thread of exactly [0, len], chamfered at the bottom and optionally the top.
static TopoDS_Shape threadRod(double d, double p, double len, bool chamferTop) {
  double rMaj = d / 2, rMin = rMaj - 0.5412658774 * p, c = std::min(rMaj - rMin + 0.05, len / 3);
  TopoDS_Shape raw = threadSolid(d, p, 0, len);
  double top = chamferTop ? len - c : len;
  TopoDS_Shape blank = revolve({{0, 0}, {rMaj - c, 0}, {rMaj + 0.01, c}, {rMaj + 0.01, top}, {chamferTop ? rMaj - c : rMaj + 0.01, len}, {0, len}});
  return common(raw, blank);
}

// Hex prism (across flats s) with a 30° chamfer at the bottom and/or top.
static TopoDS_Shape hexBlank(double s, double h, bool bottom, bool top) {
  double rc = s / sqrt(3.0);
  TopoDS_Shape hex = prism(6, rc, 0, h);
  double r0 = s / 2 * 0.95, rBig = rc + 1, rise = (rBig - r0) * tan(M_PI / 6);
  std::vector<std::pair<double, double>> outline = {{0, 0}};
  if (bottom) outline.insert(outline.end(), {{r0, 0}, {rBig, rise}});
  else outline.push_back({rBig, 0});
  if (top) outline.insert(outline.end(), {{rBig, h - rise}, {r0, h}});
  else outline.push_back({rBig, h});
  outline.push_back({0, h});
  return common(hex, revolve(outline));
}

BKShape *bk_bolt(int size, double length, int threadOnly, double clearance) {
  return guarded("bolt", [&]() -> TopoDS_Shape {
    const ThreadSize &t = sizes[std::max(0, std::min(sizeCount - 1, size))];
    double len = std::max(length, t.p * 2), d = t.d - std::max(0.0, clearance);
    if (threadOnly) return centred(threadRod(d, t.p, len, true));
    TopoDS_Shape shank = threadRod(d, t.p, len, false);
    gp_Trsf up;
    up.SetTranslation(gp_Vec(0, 0, len));
    TopoDS_Shape head = BRepBuilderAPI_Transform(hexBlank(t.s, t.k, false, true), up, true).Shape();
    return centred(fuse(shank, head));
  });
}

BKShape *bk_nut(int size, double length, int threadOnly, double clearance) {
  return guarded("nut", [&]() -> TopoDS_Shape {
    const ThreadSize &t = sizes[std::max(0, std::min(sizeCount - 1, size))];
    double len = std::max(length, t.p * 2), d = t.d + std::max(0.0, clearance);
    TopoDS_Shape body = threadOnly ? BRepPrimAPI_MakeCylinder(d / 2 + 1.6, len).Shape() : hexBlank(t.s, len, true, true);
    TopoDS_Shape tap = threadSolid(d, t.p, -1, len + 1);
    double sink = std::min({t.p * 0.6, len / 4, threadOnly ? 1.0 : t.p}), r = d / 2;
    tap = fuse(tap, revolve({{0, -1}, {r + sink + 1, -1}, {r, sink}, {0, sink}}));
    tap = fuse(tap, revolve({{0, len - sink}, {r, len - sink}, {r + sink + 1, len + 1}, {0, len + 1}}));
    return centred(cut(body, tap));
  });
}

// MARK: - transforms and operations

BKShape *bk_transform(const BKShape *s, const double *m) {
  if (!s) return nullptr;
  return guarded("transform", [&]() -> TopoDS_Shape {
    gp_Vec c0(m[0], m[4], m[8]), c1(m[1], m[5], m[9]), c2(m[2], m[6], m[10]);
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

// How far a stored pick may drift from the rebuilt geometry and still match.
static double pickTolerance(const TopoDS_Shape &shape) {
  Bnd_Box box;
  BRepBndLib::Add(shape, box);
  double diag = box.IsVoid() ? 10 : sqrt(box.SquareExtent());
  return std::max(0.3, diag * 0.02);
}

// The face whose centroid is nearest to c with a normal facing along n; null when none is within tol.
static TopoDS_Face findFace(const TopTools_IndexedMapOfShape &faces, const gp_Vec &n, const gp_Pnt &c, double tol) {
  double best = tol;
  int hit = 0;
  for (int k = 1; k <= faces.Extent(); k++) {
    gp_Pnt fc;
    gp_Vec fn;
    faceInfo(TopoDS::Face(faces(k)), fc, fn);
    double dist = fc.Distance(c);
    if (n.Magnitude() > 1e-9 && fn.Dot(n.Normalized()) < 0.7) dist += tol;
    if (dist < best) best = dist, hit = k;
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
      gp_Vec want(q[3], q[4], q[5]);
      double best = tol;
      int hit = 0;
      for (int k = 1; k <= edges.Extent(); k++) {
        TopoDS_Edge e = TopoDS::Edge(edges(k));
        if (BRep_Tool::Degenerated(e)) continue;
        gp_Vec t;
        double dist = edgeMid(e, &t).Distance(a);
        if (want.Magnitude() > 1e-9 && fabs(t.Dot(want.Normalized())) < 0.8) dist += tol;
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
        for (const gp_Pnt2d &q : loop) points.insert(points.end(), {q.X(), q.Y()});
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

BKShape *bk_hollow(const BKShape *s, const double *open, int openCount, const double *walls, const double *wallThickness, int wallCount,
                   double thickness, int *missing) {
  if (missing) *missing = 0;
  if (!s) return nullptr;
  return guarded("hollow", [&]() -> TopoDS_Shape {
    const TopoDS_Shape &shape = s->shape;
    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(shape, TopAbs_FACE, faces);
    double tol = pickTolerance(shape);
    int miss = 0;
    auto face = [&](const double *q) { return findFace(faces, gp_Vec(q[0], q[1], q[2]), gp_Pnt(q[3], q[4], q[5]), tol); };
    std::vector<TopoDS_Face> openings;
    std::vector<std::pair<TopoDS_Face, double>> own;
    for (int i = 0; i < openCount; i++) {
      TopoDS_Face f = face(open + i * 6);
      if (f.IsNull()) miss++;
      else openings.push_back(f);
    }
    for (int i = 0; i < wallCount; i++) {
      TopoDS_Face f = face(walls + i * 6);
      if (f.IsNull()) miss++;
      else own.emplace_back(f, std::max(wallThickness[i], 0.01));
    }
    if (missing) *missing = miss;
    auto volume = [](const TopoDS_Shape &x) {
      GProp_GProps g;
      BRepGProp::VolumeProperties(x, g);
      return g.Mass();
    };
    double full = volume(shape);
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
          double inner = volume(core);
          if (inner <= 0 || inner >= full) continue;
          out = cut(shape, core);
        } else {
          offset.MakeThickSolid();
          if (!offset.IsDone() || offset.Shape().IsNull()) continue;
          out = offset.Shape();
        }
        double v = volume(out);
        if (v > 0 && v < full * 0.999 && BRepCheck_Analyzer(out).IsValid()) return out;
      } catch (Standard_Failure &) {
      }
    }
    throw Standard_Failure("the walls don't fit this shape");
  });
}

// MARK: - meshing

BKMesh *bk_mesh(const BKShape *s, double deflection) {
  if (!s) return nullptr;
  BKMesh *m = new BKMesh();
  try {
    const TopoDS_Shape &shape = s->shape;
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
      }
      es.push_back((uint32_t)(ep.size() / 3));
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
