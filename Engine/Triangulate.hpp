// A triangulation in a plane that keeps given segments as edges, with exact decisions (orient2d): for cutting a triangle
// along where it meets another shape, and for filling a cut face with holes.
#pragma once
#include <algorithm>
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
  // What it asks about its points, where they're held elsewhere (exactly) and known by number.
  struct Points {
    // a, b, c turning left (1), in a line (0) or right (−1).
    virtual int orient(int a, int b, int c) const = 0;
    // m, on the line through a and b, strictly between them.
    virtual bool between(int a, int m, int b) const = 0;
    // A new point (its number) where kept edge a–b (kept for `tagAB`) crosses kept edge c–d (for `tagCD`); −1 for none.
    virtual int cross(int a, int b, int tagAB, int c, int d, int tagCD) = 0;

   protected:
    ~Points() = default;
  };

  // Starts with one triangle, counter-clockwise: in doubles (its points 0, 1, 2 at these places)...
  Tri2(double ax, double ay, double bx, double by, double cx, double cy);
  // ... or points 0, 1, 2 of `held`, every decision asked of it.
  explicit Tri2(Points &held);

  // In doubles: adds a point inside (or on an edge of) the triangulation; returns its number, or the number of the point
  // already there.
  int insert(double x, double y);
  // Held: adds point p; returns p, or the number of the point already in its place, or −1 where it lies outside.
  int insert(int p);
  // Held: adds point p, which lies on the edge u → v (an edge of the triangulation), splitting it.
  int insertOnEdge(int u, int v, int p);
  // Keeps the segment a–b as an edge (points on it along the way included), for `tag` (what it's kept for, passed on
  // where two kept edges cross). Where another kept edge crosses it, both are kept through a new point where they cross.
  bool keep(int a, int b, int tag = 0);
  bool kept(int a, int b) const { return fixed.has(key(a, b)) || fixed.has(key(b, a)); }

  // The triangles as point triples, counter-clockwise.
  std::vector<int> triangles() const;
  // The triangles inside the kept edges by the even–odd rule, those touching the first three points left out.
  std::vector<int> insideKept() const;

  // Points the triangulation made itself in doubles (where two kept edges crossed): on kept edge r → l at s along it.
  struct Made {
    int r, l;
    double s;
  };
  const std::unordered_map<int, Made> &made() const { return madeAt; }

  int count() const { return points; }
  double px(int i) const { return x[i]; }
  double py(int i) const { return y[i]; }

  // In doubles, points to go on an edge that wasn't there (put in where they lie instead) and kept segments led round
  // an outline corner; held, decisions that came out as they never should (a point outside, a hole that wouldn't
  // fill, a kept segment that couldn't be): counted, for measuring.
  int fallbacks = 0, detours = 0, misses = 0;

 private:
  struct T {
    int v[3];
    bool alive;
  };
  Points *held = nullptr;
  std::vector<double> x, y;
  int points = 0;
  std::vector<T> tris;
  EdgeTable half, fixed;  // fixed: kept edges, to their tag + 1
  std::unordered_map<int, Made> madeAt;
  int last = 0, depth = 0;
  uint32_t seed = 12345;
  // Keeps a–b up to the first point of the triangulation on it (`on`), or the whole of it (`on` left −1).
  bool keepFrom(int a, int b, int tag, int &on);

  static uint64_t key(int a, int b) { return (uint64_t)(uint32_t)a << 32 | (uint32_t)b; }
  int orient(int a, int b, int c) const;
  bool between(int a, int m, int b) const;
  int tagOf(int a, int b) const { return std::max(fixed.find(key(a, b)), fixed.find(key(b, a))) - 1; }
  int add(int a, int b, int c);
  void remove(int t);
  int across(int a, int b) const { return half.find(key(b, a)); }
  // The triangle holding point p: `where` 0 inside, 1 on the edge after vertex `which`, 2 on vertex `which`, -1 outside
  // (beyond the edge after `which`).
  int locate(int p, int &where, int &which);
  // p put in where it lies: p, or the point already there (−1 outside, held).
  int place(int p);
  int onEdge(int u, int v, int p);
  void splitInside(int t, int p);
  void splitEdge(int t, int k, int p);
  void fill(std::vector<int> poly);
};

}  // namespace bce
