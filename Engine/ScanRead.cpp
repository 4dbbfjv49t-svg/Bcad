// Reading meshes from other apps' and scanners' files (Scan.hpp): STL, OBJ, PLY and 3MF. Every count a file claims is
// checked against what's left of the file before room is made for it; numbers are read the same on every machine.
#include "Engine/Math.hpp"
#include "Engine/Scan.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstring>
#include <map>
#include <string_view>

namespace bce {

namespace {

using Text = std::string_view;

bool space(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\f' || c == '\v'; }
bool digit(char c) { return c >= '0' && c <= '9'; }
char lower(char c) { return c >= 'A' && c <= 'Z' ? (char)(c - 'A' + 'a') : c; }
Text trim(Text t) {
  while (!t.empty() && space(t.front())) t.remove_prefix(1);
  while (!t.empty() && space(t.back())) t.remove_suffix(1);
  return t;
}
// The next word of t (taken off it): false where none is left.
bool word(Text &t, Text &w) {
  size_t i = 0;
  while (i < t.size() && space(t[i])) i++;
  size_t j = i;
  while (j < t.size() && !space(t[j])) j++;
  w = t.substr(i, j - i), t.remove_prefix(j);
  return !w.empty();
}
// The whole of w a number.
bool number(Text w, double &v) {
  const char *s = w.data();
  return readNumber(s, w.data() + w.size(), v) && s == w.data() + w.size();
}
// The whole of w a whole number ([±] digits; past 10^12 taken as 10^12: no file has that many of anything).
bool whole(Text w, int64_t &v) {
  size_t i = 0;
  bool minus = false;
  if (i < w.size() && (w[i] == '+' || w[i] == '-')) minus = w[i++] == '-';
  if (i == w.size()) return false;
  v = 0;
  for (; i < w.size(); i++) {
    if (!digit(w[i])) return false;
    v = std::min<int64_t>(v * 10 + (w[i] - '0'), 1000000000000);
  }
  if (minus) v = -v;
  return true;
}

float toFloat(double v) { return v > FLT_MAX ? INFINITY : v < -FLT_MAX ? -INFINITY : (float)v; }

std::string count(uint64_t n) { return std::to_string(n); }

// Points made one where they're one to the last bit (−0 as 0), numbered as they first come: a flat table, grown as it
// fills, its slots chosen from all the bits (as weld() does).
class Welder {
 public:
  explicit Welder(std::vector<float> &p) : p(p) {}
  uint32_t add(float x, float y, float z) {
    float q[3] = {x + 0.0f, y + 0.0f, z + 0.0f};
    if ((size_t)(n + 1) * 2 > table.size()) grow();
    size_t mask = table.size() - 1;
    for (size_t k = slot(q) & mask;; k = (k + 1) & mask) {
      uint32_t at = table[k];
      if (at == UINT32_MAX) {
        table[k] = n;
        p.insert(p.end(), q, q + 3);
        return n++;
      }
      if (std::memcmp(&p[3 * (size_t)at], q, sizeof q) == 0) return at;
    }
  }
  uint32_t size() const { return n; }

 private:
  std::vector<float> &p;
  std::vector<uint32_t> table;
  uint32_t n = 0;
  static uint64_t slot(const float *q) {
    uint32_t b[3];
    std::memcpy(b, q, sizeof b);
    uint64_t h = (b[0] * 0x9E3779B97F4A7C15ull) ^ (b[1] * 0xC2B2AE3D27D4EB4Full) ^ (b[2] * 0x165667B19E3779F9ull);
    h ^= h >> 33, h *= 0xFF51AFD7ED558CCDull, h ^= h >> 33, h *= 0xC4CEB9FE1A85EC53ull, h ^= h >> 33;
    return h;
  }
  void grow() {
    table.assign(std::max<size_t>(64, table.size() * 2), UINT32_MAX);
    size_t mask = table.size() - 1;
    for (uint32_t i = 0; i < n; i++)
      for (size_t k = slot(&p[3 * (size_t)i]) & mask;; k = (k + 1) & mask)
        if (table[k] == UINT32_MAX) {
          table[k] = i;
          break;
        }
  }
};

// A face of more than three corners (point numbers into P, 3 floats each) cut into triangles in the plane it most nearly
// lies in: a quad along the diagonal that keeps both halves the right way round (the shorter where both do), a face
// turning one way all round as a fan, any other by cutting off ears (one cut anyway where none is left, and every one
// once the reading's budget of looks is spent).
void polygon(const uint32_t *c, size_t n, const float *P, std::vector<uint32_t> &out, uint64_t &budget) {
  auto at = [&](size_t i) {
    const float *q = P + 3 * (size_t)c[i];
    return V3{q[0], q[1], q[2]};
  };
  auto tri = [&](size_t a, size_t b, size_t d) { out.push_back(c[a]), out.push_back(c[b]), out.push_back(c[d]); };
  auto fan = [&] {
    for (size_t i = 1; i + 1 < n; i++) tri(0, i, i + 1);
  };
  V3 N{0, 0, 0};
  if (P)
    for (size_t i = 0; i < n; i++) {
      V3 a = at(i), b = at((i + 1) % n);
      N.x += (a.y - b.y) * (a.z + b.z), N.y += (a.z - b.z) * (a.x + b.x), N.z += (a.x - b.x) * (a.y + b.y);
    }
  double ax = std::fabs(N.x), ay = std::fabs(N.y), az = std::fabs(N.z);
  if (!P || !(ax + ay + az > 0) || !std::isfinite(ax + ay + az)) return fan();
  int k = ax >= ay && ax >= az ? 0 : ay >= az ? 1 : 2, u = (k + 1) % 3, v = (k + 2) % 3;
  if (N[k] < 0) std::swap(u, v);
  std::vector<double> x(n), y(n);
  for (size_t i = 0; i < n; i++) {
    V3 q = at(i);
    x[i] = q[u], y[i] = q[v];
  }
  auto turn = [&](size_t a, size_t b, size_t d) { return (x[b] - x[a]) * (y[d] - y[a]) - (y[b] - y[a]) * (x[d] - x[a]); };
  if (n == 4) {
    bool d02 = turn(0, 1, 2) > 0 && turn(0, 2, 3) > 0, d13 = turn(1, 2, 3) > 0 && turn(1, 3, 0) > 0;
    if (d13 && (!d02 || norm2(at(1) - at(3)) < norm2(at(0) - at(2)))) tri(1, 2, 3), tri(1, 3, 0);
    else tri(0, 1, 2), tri(0, 2, 3);
    return;
  }
  bool convex = true;
  for (size_t i = 0; i < n && convex; i++) convex = turn((i + n - 1) % n, i, (i + 1) % n) >= 0;
  if (convex) return fan();
  std::vector<size_t> next(n), prev(n);
  for (size_t i = 0; i < n; i++) next[i] = (i + 1) % n, prev[i] = (i + n - 1) % n;
  size_t left = n, i = 0, tried = 0;
  while (left > 3) {
    size_t a = prev[i], d = next[i];
    bool ear = turn(a, i, d) > 0;
    for (size_t j = next[d]; ear && budget && j != a; j = next[j]) {
      budget--;
      if (turn(a, i, j) >= 0 && turn(i, d, j) >= 0 && turn(d, a, j) >= 0) ear = false;
    }
    if (ear || tried > left || !budget) {
      tri(a, i, d);
      next[a] = d, prev[d] = a, left--, i = d, tried = 0;
    } else {
      i = next[i], tried++;
    }
  }
  tri(prev[i], i, next[i]);
}

// What a reader makes: points welded as they come, triangles in parts, every count held to the limits.
struct Build {
  Soup &s;
  const ScanLimits &limits;
  std::string &why;
  Welder weld;
  uint64_t budget = 200000000;  // looks for ears, for the whole file
  std::vector<uint32_t> cut;
  Build(Soup &s, const ScanLimits &limits, std::string &why) : s(s), limits(limits), why(why), weld(s.p) {
    s = Soup();
    s.partName.push_back("");
  }
  uint64_t triangles() const { return s.tri.size() / 3; }
  // A new part (the one so far renamed, while it has no triangles).
  void begin(Text name) {
    if (s.partStart.back() != triangles()) s.partStart.push_back((uint32_t)triangles()), s.partName.emplace_back();
    s.partName.back() = std::string(trim(name));
  }
  bool point(float x, float y, float z, uint32_t &id) {
    id = weld.add(x, y, z);
    if (weld.size() > limits.points) return why = "scan: too large: more than " + count(limits.points) + " points", false;
    return true;
  }
  bool triangle(uint32_t a, uint32_t b, uint32_t c) {
    if (triangles() >= limits.triangles) return why = "scan: too large: more than " + count(limits.triangles) + " triangles", false;
    s.tri.push_back(a), s.tri.push_back(b), s.tri.push_back(c);
    return true;
  }
  // A face's corners (into P; none: a fan).
  bool face(const uint32_t *c, size_t n, const float *P) {
    if (n < 3) return s.skipped++, true;
    if (n == 3) return triangle(c[0], c[1], c[2]);
    cut.clear();
    polygon(c, n, P, cut, budget);
    for (size_t k = 0; k < cut.size(); k += 3)
      if (!triangle(cut[k], cut[k + 1], cut[k + 2])) return false;
    return true;
  }
};

// A part closed: every side of every triangle met once by one running back along it (point numbers).
bool closedPart(const Soup &s, size_t from, size_t to) {
  std::vector<uint64_t> sides;
  sides.reserve(3 * (to - from));
  for (size_t t = from; t < to; t++)
    for (int k = 0; k < 3; k++) {
      uint32_t a = s.tri[3 * t + k], b = s.tri[3 * t + (k + 1) % 3];
      if (a == b) return false;
      sides.push_back((uint64_t)a << 32 | b);
    }
  std::sort(sides.begin(), sides.end());
  for (size_t i = 0; i < sides.size(); i++) {
    if (i && sides[i] == sides[i - 1]) return false;
    uint64_t back = sides[i] << 32 | sides[i] >> 32;
    if (!std::binary_search(sides.begin(), sides.end(), back)) return false;
  }
  return true;
}

// The parts settled once read: empty ones gone; all one where any isn't closed (a scan in patches is one body) or where
// there are more than 100.
void settle(Soup &s) {
  uint32_t total = (uint32_t)(s.tri.size() / 3);
  s.partStart.push_back(total);
  std::vector<uint32_t> start{0};
  std::vector<std::string> names;
  for (size_t k = 0; k < s.partName.size(); k++)
    if (s.partStart[k + 1] > s.partStart[k]) start.push_back(s.partStart[k + 1]), names.push_back(std::move(s.partName[k]));
  bool one = names.size() > 100;
  for (size_t k = 0; !one && names.size() > 1 && k < names.size(); k++) one = !closedPart(s, start[k], start[k + 1]);
  if (one) start = {0, total}, names = {""};
  s.partStart = std::move(start), s.partName = std::move(names);
}

// A file that says no unit, under 2 units across (its triangles' corners that are numbers): metres.
void guessUnit(Soup &s) {
  float lo[3] = {INFINITY, INFINITY, INFINITY}, hi[3] = {-INFINITY, -INFINITY, -INFINITY};
  for (uint32_t v : s.tri)
    for (int k = 0; k < 3; k++) {
      float c = s.p[3 * (size_t)v + k];
      if (std::isfinite(c)) lo[k] = std::min(lo[k], c), hi[k] = std::max(hi[k], c);
    }
  double size = 0;
  for (int k = 0; k < 3; k++)
    if (hi[k] >= lo[k]) size = std::max(size, (double)hi[k] - lo[k]);
  if (!(size > 0 && size < 2)) return;
  for (float &c : s.p) c = toFloat(c * 1000.0);
  s.scale = 1000, s.guessed = true;
}

// Every file point welded (into s.p), the triangles' file point numbers turned into the welded ones.
bool weldAll(Build &b, const std::vector<float> &fp) {
  std::vector<uint32_t> id(fp.size() / 3);
  for (size_t i = 0; i < id.size(); i++)
    if (!b.point(fp[3 * i], fp[3 * i + 1], fp[3 * i + 2], id[i])) return false;
  for (uint32_t &v : b.s.tri) {
    if (v >= id.size()) return b.why = "scan: a corner that isn't there", false;
    v = id[v];
  }
  return true;
}

uint32_t le32(const uint8_t *q) { return (uint32_t)q[0] | (uint32_t)q[1] << 8 | (uint32_t)q[2] << 16 | (uint32_t)q[3] << 24; }
float f32(uint32_t bits) {
  float f;
  std::memcpy(&f, &bits, 4);
  return f;
}

// MARK: - STL

bool stlBinary(const uint8_t *bytes, uint64_t triangles, Build &b) {
  if (triangles > b.limits.triangles) return b.why = "scan: too large: more than " + count(b.limits.triangles) + " triangles", false;
  b.s.tri.reserve(3 * triangles);
  const uint8_t *q = bytes + 84;
  for (uint64_t t = 0; t < triangles; t++, q += 50) {
    uint32_t c[3];
    for (int k = 0; k < 3; k++) {
      const uint8_t *v = q + 12 + 12 * k;
      if (!b.point(f32(le32(v)), f32(le32(v + 4)), f32(le32(v + 8)), c[k])) return false;
    }
    if (!b.triangle(c[0], c[1], c[2])) return false;
  }
  if (!triangles) return b.why = "stl: no triangles", false;
  return true;
}

bool stlText(const uint8_t *bytes, size_t n, Build &b) {
  const char *s = (const char *)bytes, *e = s + n;
  uint64_t line = 1;
  auto next = [&](Text &w) {
    for (; s < e && space(*s); s++) line += *s == '\n';
    const char *w0 = s;
    while (s < e && !space(*s)) s++;
    w = Text(w0, s - w0);
    return !w.empty();
  };
  auto restOfLine = [&] {
    const char *r = s;
    while (s < e && *s != '\n') s++;
    return Text(r, s - r);
  };
  std::vector<uint32_t> corners;
  bool any = false;
  Text w;
  while (next(w)) {
    if (w == "solid") {
      b.begin(restOfLine());
    } else if (w == "endsolid") {
      restOfLine();
    } else if (w == "facet") {
      corners.clear();
    } else if (w == "vertex") {
      double c[3];
      for (double &v : c) {
        Text num;
        if (!next(num) || !number(num, v)) return b.why = "stl: line " + count(line) + ": not a number", false;
      }
      uint32_t id;
      if (!b.point(toFloat(c[0]), toFloat(c[1]), toFloat(c[2]), id)) return false;
      corners.push_back(id);
    } else if (w == "endfacet") {
      any = true;
      if (!b.face(corners.data(), corners.size(), b.s.p.data())) return false;
      corners.clear();
    }
  }
  if (!any) return b.why = "stl: no triangles", false;
  return true;
}

// Binary where the size says so (84 bytes then 50 per triangle), even where the header starts "solid" as some apps
// write it; text where it starts "solid"; binary with bytes after its triangles where text reads nothing.
bool readStl(const uint8_t *bytes, size_t n, const ScanLimits &limits, Soup &out, std::string &why) {
  size_t i = 0;
  if (n >= 3 && bytes[0] == 0xEF && bytes[1] == 0xBB && bytes[2] == 0xBF) i = 3;
  while (i < n && space((char)bytes[i])) i++;
  bool text = n - i >= 5 && std::memcmp(bytes + i, "solid", 5) == 0;
  uint64_t triangles = n >= 84 ? le32(bytes + 80) : 0;
  bool fits = n >= 84 && 84 + 50 * triangles <= n;
  if (fits && (!text || 84 + 50 * triangles == n)) {
    Build b(out, limits, why);
    return stlBinary(bytes, triangles, b);
  }
  if (!text) return why = "stl: cut short", false;
  {
    Build b(out, limits, why);
    if (stlText(bytes + i, n - i, b)) return true;
  }
  if (!fits || !triangles) return false;
  Build b(out, limits, why);
  return stlBinary(bytes, triangles, b);
}

// MARK: - OBJ

bool readObj(const uint8_t *bytes, size_t n, const ScanLimits &limits, Soup &out, std::string &why) {
  Build b(out, limits, why);
  const char *s = (const char *)bytes, *e = s + n;
  std::vector<float> fp;  // the file's points
  std::vector<uint32_t> corners;
  std::string joined;
  uint64_t line = 0, aheadLine = 0;
  int64_t ahead = -1;  // the furthest corner named before its point
  auto take = [&](Text &t) {
    const char *nl = (const char *)std::memchr(s, '\n', e - s), *end = nl ? nl : e;
    t = Text(s, end - s), s = nl ? nl + 1 : e, line++;
    while (!t.empty() && space(t.back())) t.remove_suffix(1);
  };
  while (s < e) {
    Text t;
    take(t);
    uint64_t at = line;
    // A line going on to the next where it ends in a backslash.
    if (!t.empty() && t.back() == '\\') {
      joined.assign(t.data(), t.size() - 1);
      while (s < e) {
        Text more;
        take(more);
        bool on = !more.empty() && more.back() == '\\';
        joined += ' ', joined.append(more.data(), more.size() - on);
        if (!on) break;
      }
      t = joined;
    }
    size_t hash = t.find('#');
    if (hash != Text::npos) t = t.substr(0, hash);
    Text kw;
    if (!word(t, kw)) continue;
    if (kw == "v") {
      double c[3];
      for (double &v : c) {
        Text num;
        if (!word(t, num) || !number(num, v)) return why = "obj: line " + count(at) + ": not a number", false;
      }
      if (fp.size() / 3 >= limits.points) return why = "scan: too large: more than " + count(limits.points) + " points", false;
      fp.push_back(toFloat(c[0])), fp.push_back(toFloat(c[1])), fp.push_back(toFloat(c[2]));
    } else if (kw == "f" || kw == "fo") {
      corners.clear();
      int64_t points = (int64_t)(fp.size() / 3);
      bool known = true;
      for (Text c; word(t, c);) {
        int64_t i;
        if (!whole(c.substr(0, c.find('/')), i)) return why = "obj: line " + count(at) + ": not a number", false;
        i = i > 0 ? i - 1 : i < 0 ? points + i : -1;
        if (i < 0 || i >= (int64_t)limits.points) return why = "obj: line " + count(at) + ": a corner that isn't there", false;
        if (i >= points) {
          known = false;
          if (i > ahead) ahead = i, aheadLine = at;
        }
        corners.push_back((uint32_t)i);
      }
      if (!b.face(corners.data(), corners.size(), known ? fp.data() : nullptr)) return false;
    } else if (kw == "o" || kw == "g") {
      b.begin(t);
    }
  }
  if (ahead >= (int64_t)(fp.size() / 3)) return why = "obj: line " + count(aheadLine) + ": a corner that isn't there", false;
  if (!b.triangles()) return why = fp.empty() ? "obj: no triangles" : "obj: has points but no surface", false;
  return weldAll(b, fp);
}

// MARK: - PLY

enum PlyType { I8, U8, I16, U16, I32, U32, F32, F64 };
const int plySize[] = {1, 1, 2, 2, 4, 4, 4, 8};
bool plyType(Text t, int &type) {
  static const char *names[][2] = {{"char", "int8"},   {"uchar", "uint8"}, {"short", "int16"},  {"ushort", "uint16"},
                                   {"int", "int32"},   {"uint", "uint32"}, {"float", "float32"}, {"double", "float64"}};
  for (int k = 0; k < 8; k++)
    if (t == names[k][0] || t == names[k][1]) return type = k, true;
  return false;
}
struct PlyProp {
  std::string name;
  int type = F32, countType = -1;  // countType ≥ 0: a list, its count of that type
};
struct PlyElement {
  std::string name;
  uint64_t count = 0;
  std::vector<PlyProp> props;
};

// The values of a PLY file's elements in turn, as text or binary.
struct PlyData {
  const char *s, *e;
  int format;  // 0 text, 1 little-endian, 2 big-endian
  bool value(int type, double &v) {
    if (format == 0) {
      while (s < e && space(*s)) s++;
      const char *w = s;
      while (s < e && !space(*s)) s++;
      return number(Text(w, s - w), v);
    }
    int size = plySize[type];
    if (e - s < size) return false;
    uint64_t r = 0;
    const uint8_t *q = (const uint8_t *)s;
    if (format == 2)
      for (int i = 0; i < size; i++) r = r << 8 | q[i];
    else
      for (int i = size - 1; i >= 0; i--) r = r << 8 | q[i];
    s += size;
    switch (type) {
      case I8: v = (int8_t)r; break;
      case U8: v = (uint8_t)r; break;
      case I16: v = (int16_t)r; break;
      case U16: v = (uint16_t)r; break;
      case I32: v = (int32_t)r; break;
      case U32: v = (uint32_t)r; break;
      case F32: v = f32((uint32_t)r); break;
      default: {
        double d;
        std::memcpy(&d, &r, 8);
        v = d;
      }
    }
    return true;
  }
  // A list's count or a corner: a whole number, 0 … most.
  bool index(int type, double most, uint64_t &i) {
    double v;
    if (!value(type, v) || !(v >= 0 && v <= most) || v != std::floor(v)) return false;
    i = (uint64_t)v;
    return true;
  }
  size_t left() const { return (size_t)(e - s); }
};

bool readPly(const uint8_t *bytes, size_t n, const ScanLimits &limits, Soup &out, std::string &why) {
  Build b(out, limits, why);
  const char *s = (const char *)bytes, *e = s + n;
  auto line = [&](Text &t) {
    if (s >= e) return false;
    const char *nl = (const char *)std::memchr(s, '\n', e - s), *end = nl ? nl : e;
    t = Text(s, end - s), s = nl ? nl + 1 : e;
    if (!t.empty() && t.back() == '\r') t.remove_suffix(1);
    return true;
  };
  Text t, w;
  if (!line(t) || trim(t) != "ply") return why = "ply: not a PLY file", false;
  int format = -1;
  std::vector<PlyElement> elements;
  bool ended = false;
  while (!ended && line(t)) {
    if (!word(t, w)) continue;
    if (w == "format") {
      word(t, w);
      format = w == "ascii" ? 0 : w == "binary_little_endian" ? 1 : w == "binary_big_endian" ? 2 : -1;
      if (format < 0) return why = "ply: an unknown format", false;
    } else if (w == "element") {
      Text name, num;
      int64_t c;
      if (!word(t, name) || !word(t, num) || !whole(num, c) || c < 0) return why = "ply: a broken header", false;
      elements.push_back({std::string(name), (uint64_t)c, {}});
    } else if (w == "property") {
      PlyProp p;
      Text type, name;
      if (elements.empty() || !word(t, type)) return why = "ply: a broken header", false;
      if (type == "list") {
        Text ct;
        if (!word(t, ct) || !plyType(ct, p.countType) || p.countType >= F32 || !word(t, type)) return why = "ply: a broken header", false;
      }
      if (!plyType(type, p.type) || !word(t, name)) return why = "ply: a broken header", false;
      p.name = std::string(name);
      elements.back().props.push_back(p);
    } else if (w == "end_header") {
      ended = true;
    }
  }
  if (!ended || format < 0) return why = "ply: cut short", false;
  const PlyElement *vertex = nullptr, *faces = nullptr;
  for (const PlyElement &el : elements) {
    if (el.name == "vertex" && !vertex) vertex = &el;
    if (el.name == "face" && !faces) faces = &el;
    if (el.name == "tristrips" && el.count) return why = "ply: triangle strips", false;
  }
  if (vertex && vertex->count && (!faces || !faces->count)) return why = "ply: has points but no surface", false;
  if (!vertex || !faces) return why = "ply: no triangles", false;
  int at[3] = {-1, -1, -1};
  for (int k = 0; k < (int)vertex->props.size(); k++)
    for (int c = 0; c < 3; c++)
      if (vertex->props[k].countType < 0 && vertex->props[k].name == std::string(1, (char)('x' + c))) at[c] = k;
  if (at[0] < 0 || at[1] < 0 || at[2] < 0) return why = "ply: no x, y and z", false;
  uint64_t points = vertex->count;
  std::vector<float> fp;
  std::vector<uint32_t> corners;
  bool pointsRead = false;
  PlyData data{s, e, format};
  for (const PlyElement &el : elements) {
    if (el.props.empty() || !el.count) continue;
    // Room for every one of them in what's left: in a binary file, each takes at least its values and its lists'
    // counts; in a text file, at least a character.
    uint64_t least = 0;
    for (const PlyProp &p : el.props) least += format ? plySize[p.countType >= 0 ? p.countType : p.type] : 1;
    if (el.count > data.left() / least) return why = "ply: cut short", false;
    bool isVertex = &el == vertex, isFace = &el == faces;
    int list = -1;
    if (isVertex) {
      if (el.count > limits.points) return why = "scan: too large: more than " + count(limits.points) + " points", false;
      fp.resize(3 * el.count);
    }
    if (isFace) {
      for (int k = 0; k < (int)el.props.size() && list < 0; k++)
        if (el.props[k].countType >= 0 && (el.props[k].name == "vertex_indices" || el.props[k].name == "vertex_index")) list = k;
      for (int k = 0; k < (int)el.props.size() && list < 0; k++)
        if (el.props[k].countType >= 0) list = k;
      if (list < 0) return why = "ply: faces without corners", false;
    }
    for (uint64_t i = 0; i < el.count; i++)
      for (int k = 0; k < (int)el.props.size(); k++) {
        const PlyProp &p = el.props[k];
        if (p.countType >= 0) {
          uint64_t c;
          if (!data.index(p.countType, 4294967295.0, c)) return why = data.left() ? "ply: not a number" : "ply: cut short", false;
          if (format && c > data.left() / plySize[p.type]) return why = "ply: cut short", false;
          if (k == list) {
            corners.clear();
            for (uint64_t j = 0; j < c; j++) {
              uint64_t v;
              if (!data.index(p.type, 4294967295.0, v)) return why = data.left() ? "ply: a corner that isn't there" : "ply: cut short", false;
              if (v >= points) return why = "ply: a corner that isn't there", false;
              corners.push_back((uint32_t)v);
            }
            if (!b.face(corners.data(), corners.size(), pointsRead ? fp.data() : nullptr)) return false;
          } else if (format) {
            data.s += c * plySize[p.type];
          } else {
            for (uint64_t j = 0; j < c; j++) {
              double v;
              if (!data.value(p.type, v)) return why = "ply: cut short", false;
            }
          }
        } else {
          double v;
          if (!data.value(p.type, v)) return why = data.left() ? "ply: not a number" : "ply: cut short", false;
          if (isVertex)
            for (int c = 0; c < 3; c++)
              if (k == at[c]) fp[3 * i + c] = toFloat(v);
        }
      }
    if (isVertex) pointsRead = true;
  }
  if (!b.triangles()) return why = points ? "ply: has points but no surface" : "ply: no triangles", false;
  return weldAll(b, fp);
}

// MARK: - 3MF

// The tags of an XML file in turn: enough of XML for 3MF (declarations, comments and CDATA passed over; a name without
// its prefix).
struct Tag {
  Text name, attrs;
  bool end = false, empty = false;
};
Text local(Text name) {
  size_t colon = name.rfind(':');
  return colon == Text::npos ? name : name.substr(colon + 1);
}
class Xml {
 public:
  Xml(const uint8_t *b, size_t n) : s((const char *)b), e(s + n) {}
  bool broken = false;  // ends inside a tag
  bool next(Tag &t) {
    for (;;) {
      const char *lt = (const char *)std::memchr(s, '<', e - s);
      if (!lt) return s = e, false;
      s = lt + 1;
      if (starts("?")) {
        if (!skipPast("?>")) return false;
        continue;
      }
      if (starts("!")) {
        if (!skipPast(starts("!--") ? "-->" : starts("![CDATA[") ? "]]>" : ">")) return false;
        continue;
      }
      t.end = starts("/");
      if (t.end) s++;
      const char *n0 = s;
      while (s < e && !space(*s) && *s != '>' && *s != '/') s++;
      t.name = local(Text(n0, s - n0));
      const char *a0 = s;
      char quote = 0;
      for (; s < e; s++)
        if (quote) {
          if (*s == quote) quote = 0;
        } else if (*s == '"' || *s == '\'') {
          quote = *s;
        } else if (*s == '>') {
          break;
        }
      if (s >= e) return broken = true, false;
      t.empty = s > a0 && s[-1] == '/';
      t.attrs = Text(a0, s - a0 - t.empty);
      s++;
      return true;
    }
  }
  // Each attribute in turn: f(its name without a prefix, its value as written).
  template <class F> static void attributes(Text a, F f) {
    size_t i = 0;
    while (i < a.size()) {
      while (i < a.size() && space(a[i])) i++;
      size_t n0 = i;
      while (i < a.size() && a[i] != '=' && !space(a[i])) i++;
      Text name = a.substr(n0, i - n0);
      while (i < a.size() && space(a[i])) i++;
      if (i >= a.size() || a[i] != '=') {
        i += name.empty();
        continue;
      }
      i++;
      while (i < a.size() && space(a[i])) i++;
      if (i >= a.size() || (a[i] != '"' && a[i] != '\'')) continue;
      char q = a[i++];
      size_t v0 = i;
      while (i < a.size() && a[i] != q) i++;
      f(local(name), a.substr(v0, i - v0));
      i++;
    }
  }

 private:
  const char *s, *e;
  bool starts(const char *w) const {
    size_t n = std::strlen(w);
    return (size_t)(e - s) >= n && std::memcmp(s, w, n) == 0;
  }
  bool skipPast(const char *w) {
    size_t n = std::strlen(w);
    for (const char *q = s; (q = (const char *)std::memchr(q, w[0], e - q)) && (size_t)(e - q) >= n; q++)
      if (std::memcmp(q, w, n) == 0) return s = q + n, true;
    return s = e, broken = true, false;
  }
};

// An attribute's text as it reads: the five named and the numbered characters put back.
std::string decoded(Text v) {
  std::string out;
  for (size_t i = 0; i < v.size(); i++) {
    if (v[i] != '&') {
      out += v[i];
      continue;
    }
    size_t semi = v.find(';', i);
    if (semi == Text::npos) {
      out += v[i];
      continue;
    }
    Text ent = v.substr(i + 1, semi - i - 1);
    static const char *names[][2] = {{"amp", "&"}, {"lt", "<"}, {"gt", ">"}, {"quot", "\""}, {"apos", "'"}};
    bool done = false;
    for (auto &n : names)
      if (ent == n[0]) out += n[1], done = true;
    if (!done && ent.size() > 1 && ent[0] == '#') {
      uint32_t c = 0;
      bool hex = ent[1] == 'x' || ent[1] == 'X', ok = ent.size() > (hex ? 2u : 1u);
      for (size_t k = hex ? 2 : 1; k < ent.size() && ok; k++) {
        char d = lower(ent[k]);
        int v = digit(d) ? d - '0' : hex && d >= 'a' && d <= 'f' ? d - 'a' + 10 : -1;
        ok = v >= 0 && c < 0x110000;
        c = c * (hex ? 16 : 10) + (uint32_t)std::max(v, 0);
      }
      if (ok && c && c < 0x110000) {
        if (c < 0x80) out += (char)c;
        else if (c < 0x800) out += (char)(0xC0 | c >> 6), out += (char)(0x80 | (c & 63));
        else if (c < 0x10000) out += (char)(0xE0 | c >> 12), out += (char)(0x80 | (c >> 6 & 63)), out += (char)(0x80 | (c & 63));
        else out += (char)(0xF0 | c >> 18), out += (char)(0x80 | (c >> 12 & 63)), out += (char)(0x80 | (c >> 6 & 63)), out += (char)(0x80 | (c & 63));
        done = true;
      }
    }
    if (done) i = semi;
    else out += v[i];
  }
  return out;
}

// A placement as 3MF writes it: 12 numbers, a row vector's: x' = x m00 + y m10 + z m20 + m30 …
struct Place {
  double m[12] = {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0};
  V3 apply(double x, double y, double z) const {
    return {x * m[0] + y * m[3] + z * m[6] + m[9], x * m[1] + y * m[4] + z * m[7] + m[10], x * m[2] + y * m[5] + z * m[8] + m[11]};
  }
  // This placement, then `after`.
  Place then(const Place &after) const {
    Place out;
    for (int r = 0; r < 4; r++)
      for (int c = 0; c < 3; c++)
        out.m[3 * r + c] = m[3 * r] * after.m[c] + m[3 * r + 1] * after.m[3 + c] + m[3 * r + 2] * after.m[6 + c] + (r == 3 ? after.m[9 + c] : 0);
    return out;
  }
  double det() const {
    return m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);
  }
};
bool placement(Text v, Place &p) {
  for (double &x : p.m) {
    Text w;
    if (!word(v, w) || !number(w, x) || !std::isfinite(x)) return false;
  }
  Text more;
  return !word(v, more);
}

struct Ref {
  std::string path;  // the model file (the package's root model: "")
  uint64_t id = 0;
  Place place;
};
struct Object3 {
  std::string name;
  bool support = false;
  std::vector<float> p;  // its mesh's points, as written
  std::vector<uint32_t> t;
  std::vector<Ref> parts;  // its components
};

struct Package {
  const ScanLimits &limits;
  std::string &why;
  std::map<std::string, const PackagePart *> parts;  // by name, lower case, no leading slash
  std::map<std::string, std::map<uint64_t, Object3>> models;  // read so far, by path
  std::string root;
  double unit = 1;
  std::vector<Ref> items;
  uint64_t points = 0, triangles = 0, visits = 0;
  Package(const ScanLimits &limits, std::string &why) : limits(limits), why(why) {}

  static std::string key(Text name) {
    while (!name.empty() && name.front() == '/') name.remove_prefix(1);
    std::string k;
    for (char c : name) k += lower(c);
    return k;
  }
  bool fail(const std::string &w) { return why = "3mf: " + w, false; }

  // The model file at `path` read (the root also giving the unit and the build).
  bool read(const std::string &path) {
    if (models.count(path)) return true;
    auto found = parts.find(path);
    if (found == parts.end()) return fail(path == root ? "no model in the package" : "a part that isn't there: " + path);
    std::map<uint64_t, Object3> &objects = models[path];
    bool isRoot = path == root;
    Xml xml(found->second->bytes, found->second->n);
    Tag t;
    Object3 *obj = nullptr;
    bool inBuild = false, inMesh = false, seenModel = false;
    while (xml.next(t)) {
      Text name = t.name;
      if (t.end) {
        if (name == "object") {
          obj = nullptr;
        } else if (name == "mesh") {
          inMesh = false;
        } else if (name == "build") {
          inBuild = false;
        }
        continue;
      }
      if (name == "model") {
        seenModel = true;
        if (isRoot)
          Xml::attributes(t.attrs, [&](Text a, Text v) {
            if (a != "unit") return;
            v = trim(v);
            unit = v == "micron" ? 0.001 : v == "centimeter" ? 10 : v == "inch" ? 25.4 : v == "foot" ? 304.8 : v == "meter" ? 1000 : 1;
          });
      } else if (name == "object") {
        uint64_t id = 0;
        bool hasId = false;
        std::string objName;
        bool support = false;
        Xml::attributes(t.attrs, [&](Text a, Text v) {
          int64_t i;
          if (a == "id" && whole(trim(v), i) && i >= 0) id = (uint64_t)i, hasId = true;
          if (a == "name") objName = decoded(v);
          if (a == "type") support = trim(v) == "support";
        });
        if (!hasId) return fail("an object without its number");
        obj = objects.count(id) ? nullptr : &objects[id];
        if (obj) obj->name = objName, obj->support = support;
        if (t.empty) obj = nullptr;
      } else if (name == "mesh") {
        inMesh = !t.empty;
      } else if (name == "vertex" && inMesh && obj) {
        double c[3] = {NAN, NAN, NAN};
        bool ok = true;
        Xml::attributes(t.attrs, [&](Text a, Text v) {
          int k = a == "x" ? 0 : a == "y" ? 1 : a == "z" ? 2 : -1;
          if (k >= 0) ok = number(trim(v), c[k]) && ok;
        });
        if (!ok || std::isnan(c[0]) || std::isnan(c[1]) || std::isnan(c[2])) return fail("a point that isn't a number");
        if (++points > limits.points) return why = "scan: too large: more than " + count(limits.points) + " points", false;
        for (double v : c) obj->p.push_back(toFloat(v));
      } else if (name == "triangle" && inMesh && obj) {
        int64_t c[3] = {-1, -1, -1};
        Xml::attributes(t.attrs, [&](Text a, Text v) {
          int k = a == "v1" ? 0 : a == "v2" ? 1 : a == "v3" ? 2 : -1;
          if (k >= 0 && !whole(trim(v), c[k])) c[k] = -1;
        });
        if (c[0] < 0 || c[1] < 0 || c[2] < 0) return fail("a triangle without its corners");
        if (++triangles > limits.triangles) return why = "scan: too large: more than " + count(limits.triangles) + " triangles", false;
        for (int64_t v : c) obj->t.push_back((uint32_t)std::min<int64_t>(v, UINT32_MAX));
      } else if ((name == "component" && obj) || (name == "item" && inBuild && isRoot)) {
        Ref r;
        r.path = path;
        bool hasId = false, placed = true;
        Xml::attributes(t.attrs, [&](Text a, Text v) {
          int64_t i;
          if (a == "objectid" && whole(trim(v), i) && i >= 0) r.id = (uint64_t)i, hasId = true;
          if (a == "transform") placed = placement(v, r.place);
          if (a == "path") r.path = key(decoded(trim(v)));
        });
        if (!hasId) return fail("a part without its object");
        if (!placed) return fail("a placement that isn't 12 numbers");
        (name == "item" ? items : obj->parts).push_back(r);
      } else if (name == "build") {
        inBuild = !t.empty;
      }
    }
    if (xml.broken) return fail("cut short");
    if (!seenModel) return fail(isRoot ? "no model in the package" : "a part that isn't a model: " + path);
    return true;
  }

  // An object placed (and its components, nested at most 16 deep), its meshes as parts named after the nearest object
  // with a name.
  bool place(const Ref &r, const Place &at, const std::string &named, std::vector<std::pair<std::string, uint64_t>> &along, Build &b) {
    if (along.size() >= 16) return fail("parts nested more than 16 deep");
    if (++visits > 1000000) return fail("more than a million parts");
    for (auto &a : along)
      if (a.first == r.path && a.second == r.id) return fail("an object made of itself");
    if (!read(r.path)) return false;
    auto &objects = models[r.path];
    auto found = objects.find(r.id);
    if (found == objects.end()) return fail("an object that isn't there");
    const Object3 &o = found->second;
    if (o.support) return true;
    std::string name = o.name.empty() ? named : o.name;
    Place here = r.place.then(at);
    if (!o.t.empty()) {
      if (b.triangles() + o.t.size() / 3 > limits.triangles) return why = "scan: too large: more than " + count(limits.triangles) + " triangles", false;
      for (uint32_t v : o.t)
        if (v >= o.p.size() / 3) return fail("a corner that isn't there");
      b.begin(name);
      std::vector<uint32_t> id(o.p.size() / 3);
      for (size_t i = 0; i < id.size(); i++) {
        V3 q = here.apply(o.p[3 * i], o.p[3 * i + 1], o.p[3 * i + 2]) * unit;
        if (!b.point(toFloat(q.x), toFloat(q.y), toFloat(q.z), id[i])) return false;
      }
      bool mirrored = here.det() < 0;
      for (size_t k = 0; k < o.t.size(); k += 3)
        if (!b.triangle(id[o.t[k]], id[o.t[k + (mirrored ? 2 : 1)]], id[o.t[k + (mirrored ? 1 : 2)]])) return false;
    }
    along.push_back({r.path, r.id});
    for (const Ref &c : o.parts)
      if (!place(c, here, name, along, b)) return false;
    along.pop_back();
    return true;
  }
};

}  // namespace

bool readNumber(const char *&s, const char *end, double &out) {
  const char *q = s;
  bool minus = false;
  if (q < end && (*q == '+' || *q == '-')) minus = *q++ == '-';
  auto is = [&](const char *w) {
    size_t n = std::strlen(w);
    if ((size_t)(end - q) < n) return false;
    for (size_t i = 0; i < n; i++)
      if (lower(q[i]) != w[i]) return false;
    return q += n, true;
  };
  if (q < end && (lower(*q) == 'n' || lower(*q) == 'i')) {
    if (is("nan")) out = NAN;
    else if (is("infinity") || is("inf")) out = minus ? -INFINITY : INFINITY;
    else return false;
    s = q;
    return true;
  }
  // Up to 18 digits kept (the rest only counted), then scaled by powers of ten: the same steps on every machine.
  uint64_t m = 0;
  long exp10 = 0;
  bool any = false;
  for (; q < end && digit(*q); q++, any = true)
    if (m < 100000000000000000ull) m = m * 10 + (uint64_t)(*q - '0');
    else exp10++;
  if (q < end && *q == '.')
    for (q++; q < end && digit(*q); q++, any = true)
      if (m < 100000000000000000ull) m = m * 10 + (uint64_t)(*q - '0'), exp10--;
  if (!any) return false;
  if (q < end && (*q == 'e' || *q == 'E')) {
    const char *r = q + 1;
    bool down = false;
    if (r < end && (*r == '+' || *r == '-')) down = *r++ == '-';
    if (r < end && digit(*r)) {
      long x = 0;
      for (; r < end && digit(*r); r++)
        if (x < 100000) x = x * 10 + (*r - '0');
      exp10 += down ? -x : x;
      q = r;
    }
  }
  static const double p10[23] = {1e0,  1e1,  1e2,  1e3,  1e4,  1e5,  1e6,  1e7,  1e8,  1e9,  1e10, 1e11,
                                 1e12, 1e13, 1e14, 1e15, 1e16, 1e17, 1e18, 1e19, 1e20, 1e21, 1e22};
  double v = (double)m;
  if (m) {
    while (exp10 > 22 && v <= DBL_MAX) v *= 1e22, exp10 -= 22;
    while (exp10 < -22 && v > 0) v /= 1e22, exp10 += 22;
    exp10 = std::max(-22L, std::min(22L, exp10));
    v = exp10 >= 0 ? v * p10[exp10] : v / p10[-exp10];
  }
  out = minus ? -v : v;
  s = q;
  return true;
}

bool readScan(const uint8_t *bytes, size_t n, const std::string &extension, const ScanLimits &limits, Soup &out, std::string &why) {
  std::string ext;
  for (char c : extension) ext += lower(c);
  if (!ext.empty() && ext[0] == '.') ext.erase(0, 1);
  bool ok = ext == "stl"   ? readStl(bytes, n, limits, out, why)
            : ext == "obj" ? readObj(bytes, n, limits, out, why)
            : ext == "ply" ? readPly(bytes, n, limits, out, why)
                           : (why = "scan: not a mesh file", false);
  // (Faces there, each with fewer than three corners.)
  if (ok && out.tri.empty()) ok = false, why = ext + ": no triangles";
  if (!ok) return out = Soup(), false;
  settle(out);
  guessUnit(out);
  return true;
}

bool readScan3mf(const std::vector<PackagePart> &parts, const ScanLimits &limits, Soup &out, std::string &why) {
  Package pkg(limits, why);
  for (const PackagePart &p : parts) pkg.parts.emplace(Package::key(p.name), &p);
  pkg.root = "3d/3dmodel.model";
  // The root model as the package's relationships name it.
  auto rels = pkg.parts.find("_rels/.rels");
  if (rels != pkg.parts.end()) {
    Xml xml(rels->second->bytes, rels->second->n);
    Tag t;
    while (xml.next(t))
      if (!t.end && t.name == "Relationship") {
        std::string target, type;
        Xml::attributes(t.attrs, [&](Text a, Text v) {
          if (a == "Target") target = decoded(trim(v));
          if (a == "Type") type = std::string(trim(v));
        });
        if (type.size() >= 8 && type.compare(type.size() - 8, 8, "/3dmodel") == 0 && !target.empty()) {
          pkg.root = Package::key(target);
          break;
        }
      }
  }
  Build b(out, limits, why);
  bool ok = pkg.read(pkg.root);
  for (Ref &r : pkg.items) {
    if (!ok) break;
    if (r.path == pkg.root || r.path.empty()) r.path = pkg.root;
    std::vector<std::pair<std::string, uint64_t>> along;
    ok = pkg.place(r, Place(), "", along, b);
  }
  if (ok && !b.triangles()) ok = pkg.fail("no triangles");
  if (!ok) return out = Soup(), false;
  settle(out);
  out.scale = pkg.unit;
  return true;
}

}  // namespace bce
