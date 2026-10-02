// Bcad geometry kernel: a small C API over OpenCascade. All sizes are millimetres.
#ifndef BCAD_KERNEL_H
#define BCAD_KERNEL_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct BKShape BKShape;

enum { BK_BOX, BK_CYLINDER, BK_CONE, BK_SPHERE, BK_PRISM, BK_TORUS, BK_WEDGE, BK_PYRAMID, BK_HEMISPHERE, BK_BOWL, BK_RING, BK_GLASS, BK_OVAL, BK_OVAL_TORUS };
enum { BK_UNION, BK_SUBTRACT, BK_INTERSECT };
enum { BK_PICK_EDGE, BK_PICK_CORNER, BK_PICK_FACE, BK_PICK_BODY };

// Primitives are centred on the origin (bounding-box centre).
// box: w d h | cylinder: d h | cone: d1 d2 h | sphere: d | prism: sides d h (d across corners)
// torus: sides D d (D outside; the tube d wide: round for sides 0, a triangle with its point up for 3, a hexagon lying flat for 6;
// the polygons are d·√3/2 tall) | wedge: w d h (ramp rising along +x) | pyramid: sides d h (d across corners; 3, 4, 6, 8 …)
// half-sphere: d (dome up) | bowl: d wall (open top) | ring: D d h | glass: d h wall bottom (open top)
// oval: dA dB angle h (diameter dA along x, diameter dB at angle° from it, 5…175; 90 = plain ellipse)
// oval torus: sides dA dB angle d (the tube of a torus along an oval: dA and dB outside, as for oval)
BKShape *bk_primitive(int kind, const double *p);
// ISO metric coarse bolt / nut. size = index into bk_thread_sizes (M3 … M24).
BKShape *bk_bolt(int size, double length, int threadOnly, double clearance);
BKShape *bk_nut(int size, double length, int threadOnly, double clearance);
int bk_thread_count(void);
const char *bk_thread_name(int size);
double bk_thread_default_length(int size, int nut);
// The bounding size (x y z) of the bolt or nut those arguments make.
void bk_fastener_extent(int size, double length, int nut, int threadOnly, double clearance, double *out);

// m = row-major 3x4 affine matrix (rotation·scale | translation).
BKShape *bk_transform(const BKShape *s, const double *m);
BKShape *bk_boolean(int op, const BKShape *a, const BKShape *b);
// Keeps the side of the plane (point p, normal n) the normal points to (side 0) or away from (side 1).
BKShape *bk_split(const BKShape *s, const double *p, const double *n, int side);
// picks: kind + 6 numbers each (edge: midpoint, direction | corner: face normal, vertex | face: normal, centroid | body).
// Returns NULL when rounding fails; *maxRadius then holds the largest radius that works (0 if none).
BKShape *bk_fillet(const BKShape *s, const int *kinds, const double *picks, int count, double radius, double *maxRadius, int *missing);
// Bevels the picked edges (picks as for bk_fillet). Face A of an edge is the neighbour whose outward normal at the edge's
// middle points most up (ties: most +y, then +x); legA is measured along face A, legB along face B. cornerRadius > 0 also
// rounds the edges where each bevel meets faces A and B. Returns NULL when the bevel or its rounding doesn't fit.
BKShape *bk_chamfer(const BKShape *s, const int *kinds, const double *picks, int count, double legA, double legB, double cornerRadius,
                    int *missing);
// Cuts a concave quarter-round (cove) of the given radius along every picked convex edge; concave edges are skipped.
// Returns NULL when it doesn't fit; *maxRadius then holds the largest radius that works (0 if none).
BKShape *bk_cove(const BKShape *s, const int *kinds, const double *picks, int count, double radius, double *maxRadius, int *missing);

// The solid cut by the plane across one edge, drawn end-on as the top-right corner of a block: origin on the edge, face A
// leaving along -x, face B at 180° + angle, material between them. The frame is right-handed with direction as z, and
// y is face A's outward normal at the point. Loops are closed (the first point is not repeated): outlines run
// counter-clockwise, holes clockwise.
typedef struct {
  int loopCount, pointCount;
  double *points;                  // x y pairs of all outlines, concatenated
  int *loopStart;                  // loopCount + 1 offsets (in points)
  double angle;                    // material angle between faces A and B in degrees (90 for a box edge; > 180: concave)
  double faceA[6];                 // face A as a face pick: outward normal xyz, centroid xyz
  double faceB[6];
  double point[3], direction[3];   // the section point on the edge and the edge direction there
} BKSection;

// kind and pick (6 numbers) as for bk_fillet: an edge pick cuts at the pick point projected onto the edge, a corner pick
// the edge it rounds, a face pick its longest edge, a body pick its longest convex edge (those at their middle).
// Curves are sampled for a view of ±radius around the origin. Returns NULL when there is no edge between two faces.
BKSection *bk_section(const BKShape *s, int kind, const double *pick, double radius);
void bk_section_free(BKSection *section);
// Hollows a solid inward, keeping its outside. open: faces removed as openings; walls: faces with their own thickness.
// Faces are given as 6 numbers each (normal, centroid). Without openings the result is closed with an inner void.
// sharp: the same solid with roundings left out (the last without any), tried in turn when the solid itself doesn't
// offset, and the last alone when faces have walls of their own: that one is hollowed, and only what lies inside the
// solid is kept.
BKShape *bk_hollow(const BKShape *s, const BKShape *const *sharp, int sharpCount, const double *open, int openCount, const double *walls,
                   const double *wallThickness, int wallCount, double thickness, int *missing);
// Separate solids in a shape: a merge of parts that don't touch stays in pieces.
int bk_piece_count(const BKShape *s);
BKShape *bk_copy(const BKShape *s);
void bk_free(BKShape *s);

typedef struct {
  int vertexCount, triangleCount;
  float *positions, *normals;       // 3 per vertex
  uint32_t *indices;                // 3 per triangle
  uint32_t *triangleFace;           // face id per triangle
  int faceCount;
  double *faceInfo;                 // per face: normal xyz, centroid xyz
  int edgeCount, edgePointCount;
  float *edgePoints;                // xyz polylines, concatenated
  uint32_t *edgeStart;              // edgeCount + 1 offsets (in points)
  int32_t *edgeFaces;               // per edge: the faces on either side (-1 when there is none)
  int circleCount;
  double *circles;                  // per circular edge of half a turn or more: centre xyz, axis xyz, radius
  int cornerCount;
  float *corners;                   // xyz per topological vertex
  double bbox[6];                   // min xyz, max xyz
  double volume;
  int valid;
} BKMesh;

BKMesh *bk_mesh(const BKShape *s, double deflection);
void bk_mesh_free(BKMesh *m);
int bk_export_step(const BKShape *const *shapes, int count, const char *path);
const char *bk_last_error(void);

#ifdef __cplusplus
}
#endif
#endif
