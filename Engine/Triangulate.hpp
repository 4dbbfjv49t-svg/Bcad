// A triangulation in a plane that keeps given segments as edges, with exact decisions (orient2d): for cutting a triangle
// along where it meets another shape, and for filling a cut face with holes.
#pragma once
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace bce {

// A table from edge (two point numbers in one 64-bit key) to a number, in one flat array (open addressing): no
// allocation per entry, which a triangulation adding and taking out edges all the time would otherwise spend its time on.
class EdgeTable {
 public:
  int find(uint64_t k) const {
    if (keys.empty()) return -1;
    for (size_t i = slot(k);; i = (i + 1) & (keys.size() - 1)) {
      if (keys[i] == k) return vals[i];
      if (keys[i] == empty) return -1;
    }
  }
  bool has(uint64_t k) const { return find(k) >= 0; }
  void set(uint64_t k, int v) {
    if ((used + 1) * 4 >= keys.size() * 3) grow();
    size_t gone = SIZE_MAX;
    for (size_t i = slot(k);; i = (i + 1) & (keys.size() - 1)) {
      if (keys[i] == k) {
        vals[i] = v;
        return;
      }
      if (keys[i] == removed && gone == SIZE_MAX) gone = i;
      if (keys[i] == empty) {
        if (gone == SIZE_MAX) gone = i, used++;
        keys[gone] = k, vals[gone] = v;
        return;
      }
    }
  }
  bool erase(uint64_t k) {
    if (keys.empty()) return false;
    for (size_t i = slot(k);; i = (i + 1) & (keys.size() - 1)) {
      if (keys[i] == k) {
        keys[i] = removed;
        return true;
      }
      if (keys[i] == empty) return false;
    }
  }
  template <typename F> void each(F f) const {
    for (size_t i = 0; i < keys.size(); i++)
      if (keys[i] != empty && keys[i] != removed) f(keys[i], vals[i]);
  }

 private:
  static constexpr uint64_t empty = ~0ull, removed = ~0ull - 1;
  std::vector<uint64_t> keys;
  std::vector<int> vals;
  size_t used = 0;  // slots ever filled (taken-out ones too)
  size_t slot(uint64_t k) const { return (size_t)((k * 0x9E3779B97F4A7C15ull) >> 20) & (keys.size() - 1); }
  void grow() {
    std::vector<uint64_t> oldKeys;
    std::vector<int> oldVals;
    oldKeys.swap(keys), oldVals.swap(vals);
    size_t live = 0;
    for (uint64_t k : oldKeys) live += k != empty && k != removed;
    size_t cap = 16;
    while (cap * 3 <= (live + 1) * 8) cap *= 2;
    keys.assign(cap, empty), vals.assign(cap, 0), used = 0;
    for (size_t i = 0; i < oldKeys.size(); i++)
      if (oldKeys[i] != empty && oldKeys[i] != removed) set(oldKeys[i], oldVals[i]);
  }
};

class Tri2 {
 public:
  // Starts with one triangle, counter-clockwise.
  Tri2(double ax, double ay, double bx, double by, double cx, double cy);

  // Adds a point inside (or on an edge of) the triangulation; returns its number, or the number of the point already
  // there.
  int insert(double x, double y);
  // Adds a point known to lie inside: where rounding put it on the outline or a hair outside, it's moved toward (cx, cy)
  // by the least that puts it inside, so the outline (shared with neighbours) is never split by it.
  int insertWithin(double x, double y, double cx, double cy);
  // Adds a point on the edge u → v (an edge of the triangulation), splitting it, whatever rounding says about which side
  // it lies on.
  int insertOnEdge(int u, int v, double x, double y);
  // Keeps the segment a–b as an edge (points on it along the way included). Where another kept edge crosses it, both are
  // kept through a new point where they cross.
  bool keep(int a, int b);
  bool kept(int a, int b) const { return fixed.has(key(a, b)) || fixed.has(key(b, a)); }

  // The triangles as point triples, counter-clockwise.
  std::vector<int> triangles() const;
  // The triangles inside the kept edges by the even–odd rule, those touching the first three points left out.
  std::vector<int> insideKept() const;

  // Points the triangulation made itself (where two kept edges crossed): on kept edge r → l at s along it.
  struct Made {
    int r, l;
    double s;
  };
  const std::unordered_map<int, Made> &made() const { return madeAt; }

  int count() const { return (int)x.size(); }
  double px(int i) const { return x[i]; }
  double py(int i) const { return y[i]; }

 private:
  struct T {
    int v[3];
    bool alive;
  };
  std::vector<double> x, y;
  std::vector<T> tris;
  EdgeTable half, fixed;
  std::unordered_map<int, Made> madeAt;
  int last = 0, depth = 0;
  uint32_t seed = 12345;

  static uint64_t key(int a, int b) { return (uint64_t)(uint32_t)a << 32 | (uint32_t)b; }
  int orient(int a, int b, int c) const;
  int orientPt(int a, int b, double px, double py) const;
  int add(int a, int b, int c);
  void remove(int t);
  int across(int a, int b) const { return half.find(key(b, a)); }
  // The triangle holding (px, py): `where` 0 inside, 1 on the edge after vertex `which`, 2 on vertex `which`, -1 outside
  // (beyond the edge after `which`).
  int locate(double px, double py, int &where, int &which);
  void splitInside(int t, int p);
  void splitEdge(int t, int k, int p);
  void fill(std::vector<int> poly);
};

}  // namespace bce
