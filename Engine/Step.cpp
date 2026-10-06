// STEP files of Bcad's own shapes: each body a solid bounded by flat faces, every corner and side shared by the faces that
// meet there, so any CAD program reads it as a closed solid. A face of the shape that's flat all over goes in as one face
// on its exact plane, holes and all; a curved one as the triangles of its mesh, made as fine as an STL export's. A body
// in pieces is a solid per piece; a sealed hollow inside one is a void of it. Where two parts touch along a line, the
// faces there are told apart by the wedge of material each pair of them bounds. ISO 10303-21, AP214 ("automotive
// design"), millimetres.
#include "Engine/Step.hpp"
#include "Engine/Print.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <unordered_map>

namespace bce {

namespace {

// A number as STEP writes it: the shortest that reads back the same, always with a point (1., 0.5, 1.E-07), put on the
// end of `o` (no string of its own made).
void numTo(std::string &o, double v) {
  if (v == 0) {
    o += "0.";
    return;
  }
  char buf[64];
  char *end = std::to_chars(buf, buf + sizeof buf, v).ptr, *e = buf;
  while (e < end && *e != 'e' && *e != 'E') e++;
  o.append(buf, e);
  if (std::find(buf, e, '.') == e) o += '.';
  if (e == end) return;
  o += 'E';
  o.append(e[1] == '+' ? e + 2 : e + 1, end);
}

// A string as STEP writes it: ' and \ doubled, characters past plain ASCII as \X2\ (or, past 16 bits, \X4\) runs of
// hex, control characters as spaces.
std::string str(const std::string &s) {
  std::string out = "'";
  int mode = 0;  // 0 plain, 2 or 4 inside a \X2\ or \X4\ run
  auto to = [&](int m) {
    if (mode == m) return;
    if (mode) out += "\\X0\\";
    if (m) out += m == 2 ? "\\X2\\" : "\\X4\\";
    mode = m;
  };
  for (size_t i = 0; i < s.size();) {
    unsigned char c = (unsigned char)s[i];
    uint32_t cp = 0xFFFD;
    int n = c < 0x80 ? 1 : (c >> 5) == 6 ? 2 : (c >> 4) == 14 ? 3 : (c >> 3) == 30 ? 4 : 0;
    if (n == 0) {
      i++;
    } else {
      cp = n == 1 ? c : n == 2 ? c & 0x1F : n == 3 ? c & 0x0F : c & 0x07;
      int k = 1;
      for (; k < n && i + k < s.size() && ((unsigned char)s[i + k] & 0xC0) == 0x80; k++) cp = cp << 6 | ((unsigned char)s[i + k] & 0x3F);
      if (k < n) cp = 0xFFFD;
      i += k;
    }
    char h[9];
    if (cp >= 0x20 && cp < 0x7F) {
      to(0);
      out += cp == '\'' ? "''" : cp == '\\' ? "\\\\" : std::string(1, (char)cp);
    } else if (cp < 0x20 || cp == 0x7F) {
      to(0);
      out += ' ';
    } else if (cp < 0x10000) {
      to(2);
      snprintf(h, sizeof h, "%04X", cp);
      out += h;
    } else {
      to(4);
      snprintf(h, sizeof h, "%08X", cp);
      out += h;
    }
  }
  to(0);
  return out + "'";
}

std::string ref(int id) { return "#" + std::to_string(id); }

template <typename T> std::string list(const std::vector<T> &ids) {
  std::string s = "(";
  for (size_t i = 0; i < ids.size(); i++) s += (i ? ",#" : "#") + std::to_string(ids[i]);
  return s + ")";
}

// The DATA section as it's written, entity by entity.
struct Out {
  std::string text;
  int next = 1;
  int add(const std::string &entity) {
    int id = next++;
    text += '#';
    text += std::to_string(id);
    text += '=';
    text += entity;
    text += ";\n";
    return id;
  }
  // An entity written straight into the text, piece by piece: begun (its number given out), references, numbers and
  // lists of references put on, ended.
  int begin(const char *head) {
    int id = next++;
    text += '#';
    number(id);
    text += '=';
    text += head;
    return id;
  }
  void number(int v) {
    char b[16];
    text.append(b, std::to_chars(b, b + sizeof b, v).ptr);
  }
  void ref(int id) { text += '#', number(id); }
  void refs(const std::vector<int> &ids) {
    text += '(';
    for (size_t i = 0; i < ids.size(); i++) {
      if (i) text += ',';
      ref(ids[i]);
    }
    text += ')';
  }
  int end(const char *tail, int id) {
    text += tail;
    text += ";\n";
    return id;
  }
  int point(V3 p) { return triple("CARTESIAN_POINT('',(", p); }
  int direction(V3 d) { return triple("DIRECTION('',(", d); }
  // (Written straight into the text.)
  int triple(const char *head, V3 v) {
    int id = next++;
    text += '#';
    text += std::to_string(id);
    text += '=';
    text += head;
    numTo(text, v.x), text += ',', numTo(text, v.y), text += ',', numTo(text, v.z);
    text += "));\n";
    return id;
  }
};

// One body's solids, written into `o` (in the representation context `context`): the solids' numbers in `solids`.
bool body(const Solid &s, const std::string &name, Out &o, std::vector<int> &solids, std::string &why) {
  Shells sh;
  if (!shells(s, sh, why)) return false;
  const auto &P = sh.P;
  const auto &T = sh.T;
  const auto &mate = sh.mate;
  const auto &edgeOf = sh.edgeOf;
  const auto &edges = sh.edges;
  const auto &shellAt = sh.shellAt;
  const auto &vol = sh.vol;
  const auto &voidOf = sh.voidOf;
  size_t nt = T.size() / 3, ns = T.size(), nsh = vol.size();
  double size = sh.size;
  auto from = [&](uint32_t side) { return sh.from(side); };
  auto to = [&](uint32_t side) { return sh.to(side); };

  // Faces flat all over: each run of their triangles joined across sides one face, bounded by the run's outline (one
  // loop round it, and one round each hole), where that's plain; otherwise triangle by triangle.
  Find run(nt);
  auto flatFace = [&](uint32_t t) {
    uint32_t f = t < s.triFace.size() ? s.triFace[t] : UINT32_MAX;
    return f < s.faces.size() && s.faces[f].geom.flat ? (int)f : -1;
  };
  for (uint32_t i = 0; i < ns; i++) {
    uint32_t t = i / 3, u = mate[i] / 3;
    int f = flatFace(t);
    if (f >= 0 && f == flatFace(u)) run.join(t, u);
  }
  // Per run (by its first triangle): its loops of sides (the outer one first), or none to go triangle by triangle.
  std::unordered_map<uint32_t, std::vector<std::vector<uint32_t>>> loopsOf;
  std::unordered_map<uint32_t, V3> normalOf;
  std::unordered_map<uint32_t, std::vector<uint32_t>> members;
  {
    for (uint32_t t = 0; t < nt; t++)
      if (flatFace(t) >= 0) members[run(t)].push_back(t);
    for (auto &[r, tris] : members) {
      if (tris.size() < 2) continue;
      const FaceGeom &g = s.faces[flatFace(r)].geom;
      V3 area{0, 0, 0};
      for (uint32_t t : tris) area = area + cross(P[T[3 * t + 1]] - P[T[3 * t]], P[T[3 * t + 2]] - P[T[3 * t]]);
      V3 n = dot(g.pn, area) >= 0 ? g.pn : -g.pn;
      n = unit(n);
      // On its plane (a merge's clean-up moves points by far less than this).
      double d = dot(n, P[T[3 * tris[0]]]), off = 0;
      for (uint32_t t : tris)
        for (int k = 0; k < 3; k++) off = std::max(off, std::fabs(dot(n, P[T[3 * t + k]]) - d));
      if (off > 1e-6 + 1e-12 * size) continue;
      std::unordered_map<uint32_t, uint32_t> next;  // outline side by the point it starts from
      bool plain = true;
      for (uint32_t t : tris)
        for (int k = 0; k < 3 && plain; k++) {
          uint32_t side = 3 * t + k;
          if (run(mate[side] / 3) == r) continue;
          plain = next.emplace(from(side), side).second;  // (an outline touching itself at a point: not plain)
        }
      if (!plain) continue;
      std::vector<std::vector<uint32_t>> loops;
      std::unordered_map<uint32_t, bool> used;
      std::vector<uint32_t> starts;
      for (auto &[p, side] : next) starts.push_back(side);
      std::sort(starts.begin(), starts.end());
      int outer = -1, positive = 0;
      for (uint32_t start : starts) {
        if (used[start]) continue;
        std::vector<uint32_t> loop;
        uint32_t side = start;
        V3 newell{0, 0, 0};
        while (!used[side]) {
          used[side] = true;
          loop.push_back(side);
          newell = newell + cross(P[from(side)], P[to(side)]);
          auto it = next.find(to(side));
          if (it == next.end()) {
            plain = false;
            break;
          }
          side = it->second;
        }
        if (!plain || side != start) {
          plain = false;
          break;
        }
        if (dot(newell, n) > 0) positive++, outer = (int)loops.size();
        loops.push_back(loop);
      }
      if (!plain || positive != 1) continue;
      std::swap(loops[0], loops[outer]);
      loopsOf[r] = std::move(loops);
      normalOf[r] = n;
    }
  }

  // Written shell by shell (a void's turned inside out, then taken the other way round, as STEP has voids).
  std::vector<int> cp(P.size(), 0), vp(P.size(), 0), vpShell(P.size(), -1), curve(edges.size(), 0);
  auto pointAt = [&](uint32_t v) { return cp[v] ? cp[v] : cp[v] = o.point(P[v]); };
  auto edgeCurve = [&](uint32_t e, int shell) {
    if (curve[e]) return curve[e];
    auto vertex = [&](uint32_t v) {
      if (vpShell[v] != shell) {
        int at = pointAt(v), id = o.begin("VERTEX_POINT('',");
        o.ref(at);
        vpShell[v] = shell, vp[v] = o.end(")", id);
      }
      return vp[v];
    };
    auto [u, v] = edges[e];
    V3 d = P[v] - P[u];
    int a = vertex(u), b = vertex(v);
    int dir = o.direction(unit(d)), vec = o.begin("VECTOR('',");
    o.ref(dir), o.text += ',', numTo(o.text, norm(d));
    o.end(")", vec);
    int at = pointAt(u), line = o.begin("LINE('',");
    o.ref(at), o.text += ',', o.ref(vec);
    o.end(")", line);
    int id = o.begin("EDGE_CURVE('',");
    o.ref(a), o.text += ',', o.ref(b), o.text += ',', o.ref(line);
    return curve[e] = o.end(",.T.)", id);
  };
  std::vector<int> shellId(nsh, 0);
  for (size_t sh = 0; sh < nsh; sh++) {
    bool turned = vol[sh] < 0;
    std::vector<int> faces;
    auto face = [&](const std::vector<std::vector<uint32_t>> &loops, V3 n, uint32_t anchor) {
      std::vector<int> bounds;
      for (size_t l = 0; l < loops.size(); l++) {
        std::vector<int> oriented;
        std::vector<uint32_t> sides = loops[l];
        if (turned) std::reverse(sides.begin(), sides.end());
        for (uint32_t side : sides) {
          bool along = (from(side) == edges[edgeOf[side]].first) != turned;
          int curveId = edgeCurve(edgeOf[side], (int)sh), id = o.begin("ORIENTED_EDGE('',*,*,");
          o.ref(curveId);
          oriented.push_back(o.end(along ? ",.T.)" : ",.F.)", id));
        }
        int loop = o.begin("EDGE_LOOP('',");
        o.refs(oriented);
        o.end(")", loop);
        int bound = o.begin(l == 0 ? "FACE_OUTER_BOUND(''," : "FACE_BOUND('',");
        o.ref(loop);
        bounds.push_back(o.end(",.T.)", bound));
      }
      // (One entity per statement: within one expression, the order they're numbered in is the compiler's choice.)
      int at = pointAt(anchor), up = o.direction(turned ? -n : n);
      int ax = o.begin("AXIS2_PLACEMENT_3D('',");
      o.ref(at), o.text += ',', o.ref(up);
      o.end(",$)", ax);
      int plane = o.begin("PLANE('',");
      o.ref(ax);
      o.end(")", plane);
      int id = o.begin("ADVANCED_FACE('',");
      o.refs(bounds), o.text += ',', o.ref(plane);
      faces.push_back(o.end(",.T.)", id));
    };
    std::vector<char> done(nt, 0);
    for (uint32_t t = 0; t < nt; t++) {
      if (shellAt[t] != (int)sh || done[t]) continue;
      auto it = flatFace(t) >= 0 ? loopsOf.find(run(t)) : loopsOf.end();
      if (it != loopsOf.end()) {
        for (uint32_t u : members[it->first]) done[u] = 1;
        face(it->second, normalOf[it->first], from(it->second[0][0]));
        continue;
      }
      done[t] = 1;
      V3 a = P[T[3 * t]], b = P[T[3 * t + 1]], c = P[T[3 * t + 2]], n = cross(b - a, c - a);
      if (!(norm(n) > 0)) {
        // (No area to tell: the way its corners' normals point.)
        for (int k = 0; k < 3; k++) n = n + s.n[s.tri[3 * t + k]];
        if (!(norm(n) > 0)) n = {0, 0, 1};
      }
      face({{3 * t, 3 * t + 1, 3 * t + 2}}, unit(n), T[3 * t]);
    }
    shellId[sh] = o.add("CLOSED_SHELL(''," + list(faces) + ")");
  }
  for (size_t sh = 0; sh < nsh; sh++) {
    if (vol[sh] < 0) continue;
    std::vector<int> voids;
    for (size_t v = 0; v < nsh; v++)
      if (voidOf[v] == (int)sh) voids.push_back(o.add("ORIENTED_CLOSED_SHELL('',*," + ref(shellId[v]) + ",.F.)"));
    solids.push_back(voids.empty() ? o.add("MANIFOLD_SOLID_BREP(" + str(name) + "," + ref(shellId[sh]) + ")")
                                   : o.add("BREP_WITH_VOIDS(" + str(name) + "," + ref(shellId[sh]) + "," + list(voids) + ")"));
  }
  return true;
}

}  // namespace

bool stepText(const std::vector<Shape> &shapes, const std::vector<std::string> &names, const std::string &file, std::string &out,
              std::string &why, size_t *which) {
  // (The header first, the entities after it in the same text, which is then handed over: one copy of a file that can
  // be hundreds of megabytes, not two.)
  char stamp[32] = "";
  std::time_t now = std::time(nullptr);
  std::tm utc{};
  if (gmtime_r(&now, &utc)) std::strftime(stamp, sizeof stamp, "%Y-%m-%dT%H:%M:%S", &utc);
  Out o;
  o.text = "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('Bcad model'),'2;1');\nFILE_NAME(" + str(file) + ",'" + stamp +
           "',(''),(''),'Bcad','Bcad','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\nENDSEC;\nDATA;\n";
  int app = o.add("APPLICATION_CONTEXT('core data for automotive mechanical design processes')");
  o.add("APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000," + ref(app) + ")");
  int mm = o.add("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))");
  int rad = o.add("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))");
  int sr = o.add("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())");
  int tol = o.add("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-06)," + ref(mm) + ",'distance_accuracy_value','confusion accuracy')");
  int context = o.add("(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((" + ref(tol) + "))GLOBAL_UNIT_ASSIGNED_CONTEXT((" +
                      ref(mm) + "," + ref(rad) + "," + ref(sr) + "))REPRESENTATION_CONTEXT('','3D'))");
  int productContext = o.add("PRODUCT_CONTEXT(''," + ref(app) + ",'mechanical')");
  int definitionContext = o.add("PRODUCT_DEFINITION_CONTEXT('part definition'," + ref(app) + ",'design')");
  int zero = o.point({0, 0, 0}), z = o.direction({0, 0, 1}), x = o.direction({1, 0, 0});
  int origin = o.add("AXIS2_PLACEMENT_3D(''," + ref(zero) + "," + ref(z) + "," + ref(x) + ")");
  for (size_t i = 0; i < shapes.size(); i++) {
    std::string name = i < names.size() && !names[i].empty() ? names[i] : "Body " + std::to_string(i + 1);
    Solid s;
    mesh(shapes[i], fileDeflection, s);
    std::vector<int> items;
    if (!body(s, name, o, items, why)) {
      if (which) *which = i;
      return false;
    }
    items.push_back(origin);
    int product = o.add("PRODUCT(" + str(name) + "," + str(name) + ",''," + list(std::vector<int>{productContext}) + ")");
    o.add("PRODUCT_RELATED_PRODUCT_CATEGORY('part',$," + list(std::vector<int>{product}) + ")");
    int formation = o.add("PRODUCT_DEFINITION_FORMATION('',''," + ref(product) + ")");
    int definition = o.add("PRODUCT_DEFINITION('design',''," + ref(formation) + "," + ref(definitionContext) + ")");
    int shape = o.add("PRODUCT_DEFINITION_SHAPE('',''," + ref(definition) + ")");
    int rep = o.add("ADVANCED_BREP_SHAPE_REPRESENTATION(" + str(name) + "," + list(items) + "," + ref(context) + ")");
    o.add("SHAPE_DEFINITION_REPRESENTATION(" + ref(shape) + "," + ref(rep) + ")");
  }
  o.text += "ENDSEC;\nEND-ISO-10303-21;\n";
  out = std::move(o.text);
  return true;
}

}  // namespace bce
