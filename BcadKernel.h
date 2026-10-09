// Bcad geometry kernel: the C API of Bcad's own geometry engine (Engine/). All sizes are millimetres.
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
// Bolts and nuts on ISO metric coarse threads (M3 … M24). Bolts: a threaded rod without a head; a hex or 12-point head, on
// its own or with a cone under it; a round head with a hex or Torx socket; a countersunk head with a hex socket, Torx or
// Phillips; a hex head with a Phillips recess, with or without a cone under it. Nuts: a plain sleeve, square, hex, or a
// hex narrowing into a cone seat below.
enum { BK_ROD, BK_HEX, BK_HEX_CONE, BK_SOCKET, BK_SOCKET_CONE, BK_TWELVE, BK_TWELVE_CONE, BK_TORX, BK_TORX_CONE, BK_PH_HEX,
       BK_PH_HEX_CONE, BK_PH_CONE, BK_SLEEVE, BK_SQUARE_NUT, BK_HEX_NUT, BK_CONE_NUT };
enum { BK_LENGTH, BK_WIDTH, BK_HEIGHT, BK_ANGLE, BK_SEAT, BK_DRIVE, BK_RECESS, BK_DEPTH };
// size: the thread, M3 … M24 in turn. length: a bolt's below its head (a countersunk head and a cone under a head count
// in it, as on the box), a nut's height. width: across flats (hex, 12-point, square), the diameter of a round or
// countersunk head, a sleeve's wall. height: a head's (not countersunk). angle: a cone's included angle. seat: the height
// of a cone nut's cone. drive: hex key across flats, Torx size (10 = T10) or PH size. recess: a Phillips recess's
// diameter. depth: the socket's or recess's. All in mm and degrees; a kind uses only some of them (bk_fastener_fields).
typedef struct {
  int kind, size;
  double length, width, height, angle, seat, drive, recess, depth;
} BKFastener;
int bk_thread_count(void);
const char *bk_thread_name(int size);
int bk_torx_count(void);
int bk_torx_number(int i);
// The sizes a kind has: bit (1 << BK_…) for each.
int bk_fastener_fields(int kind);
// The kind's sizes for its thread, standard ones where there is a standard; its length too when withLength.
void bk_fastener_defaults(BKFastener *f, int withLength);
// The range one size takes with the others as they are: out[0] … out[1] (empty when out[0] > out[1]). Loose: the range it
// takes when the sizes depending on it follow (bk_fastener_fit), such as a socket getting shallower in a lower head.
void bk_fastener_range(const BKFastener *f, int field, int loose, double *out);
// Each size brought into its range, after another one changed.
void bk_fastener_fit(BKFastener *f);
// Sets the drive; a Phillips size brings the recess that goes with it.
void bk_fastener_drive(BKFastener *f, double drive);
// NULL when a size is out of its range (bk_last_error says which).
BKShape *bk_fastener(const BKFastener *f, double clearance);
// The bounding size (x y z) of what bk_fastener makes.
void bk_fastener_extent(const BKFastener *f, double clearance, double *out);

// A body that is a closed mesh as given (a sculpted body): its points (x y z each) and triangles (3 point numbers each,
// counter-clockwise seen from outside), its surface exactly, at any detail. NULL when it isn't a closed solid
// (bk_last_error says why: "mesh: open", "mesh: inside out" …).
BKShape *bk_mesh_shape(const float *positions, int vertexCount, const uint32_t *indices, int triangleCount);
// A body made ready for sculpting: what the shape (placed by m, row-major 3x4) encloses, made again as one closed mesh of
// even triangles about `detail` mm apart, its points shared (where pieces overlap or a mesh folds through itself, the
// solid they make). NULL when it can't be (bk_last_error says why: "remesh: too fine: about N triangles", "remesh:
// nothing inside" …).
typedef struct {
  int vertexCount, triangleCount;
  float *positions;   // 3 per vertex
  uint32_t *indices;  // 3 per triangle, counter-clockwise seen from outside
} BKSculptMesh;
BKSculptMesh *bk_remesh(const BKShape *s, const double *m, double detail);
// A mesh body's own mesh, placed by m (row-major 3x4): a figure's or a sculpted body's points and triangles as they are
// (not cut, merged or treated), so its finest parts stay as fine. NULL when the shape isn't one as it is.
BKSculptMesh *bk_mesh_body(const BKShape *s, const double *m);
void bk_sculpt_mesh_free(BKSculptMesh *m);

// A sculpted body being shaped (Engine/Sculpt.hpp): made from its mesh, read back at any time as floats (positions and
// normals 3 per point; the triangles change only with a detail size set, below). Strokes of a brush, each undone and redone as one: begun where it
// starts on the surface, dabbed along its way (a pen's pressure scaling each dab's strength, and its size the radius:
// 0…1, both 1 for a mouse), ended. Radius in mm; strength 0…1; mirror does the same across x = 0; invert carves instead
// of raising (and so on). NULL when the mesh is broken.
enum { BK_BRUSH_GRAB, BK_BRUSH_DRAW, BK_BRUSH_INFLATE, BK_BRUSH_SMOOTH, BK_BRUSH_FLATTEN, BK_BRUSH_PINCH, BK_BRUSH_CREASE, BK_BRUSH_DETAIL,
       BK_BRUSH_CLAY, BK_BRUSH_LAYER, BK_BRUSH_BLOB, BK_BRUSH_SCRAPE, BK_BRUSH_SMUDGE, BK_BRUSH_SNAKE_HOOK, BK_BRUSH_TWIST, BK_BRUSH_COUNT };
enum { BK_MIRROR_X = 1, BK_MIRROR_Y = 2, BK_MIRROR_Z = 4 };
// A stroke's brush in full: mirror any of BK_MIRROR_… (across those planes through `middle`, in every combination; all
// 0: the body's origin); the tip: hardness, the part of the radius at full strength (0…1); rigidity, how crisp the fade from there
// to the rim (0 soft and rounded … 1 a straight slope); oval, how wide the footprint is across as along (0.05…1) and
// angle, how far it's turned from the stroke's way (degrees); tilt, how far the push leans from straight out toward the
// stroke's way (degrees, ±85). across: the way the stroke is taken to go before it has moved (the view's right, say).
// Numbers that aren't numbers taken as 0 (oval: 1).
typedef struct {
  int brush;
  double radius, strength;
  int mirror, invert;
  double hardness, rigidity, oval, angle, tilt;
  double across[3];
  double middle[3];
} BKBrush;
typedef struct BKSculpt BKSculpt;
BKSculpt *bk_sculpt_new(const float *positions, int vertexCount, const uint32_t *indices, int triangleCount);
void bk_sculpt_free(BKSculpt *s);
// Where the ray (origin, direction: 3 each) first meets the surface: 1 with the point and the surface's normal there.
int bk_sculpt_ray(const BKSculpt *s, const double *origin, const double *direction, double *at, double *normal);
// With a detail size (mm; 0: off), every stroke begun after also makes the triangles it passes over that size (as fine
// as a remesh at that detail): long sides halved, short ones merged away, thin triangles' sides turned (grab: when its
// drag ends). BK_BRUSH_DETAIL does only
// that, leaving the surface where it is. The mesh's points and triangles are then kept in slots that come and go: counts
// and arrays below are per slot (a free triangle slot's corners all 0), and grow.
void bk_sculpt_set_detail(BKSculpt *s, double size);
void bk_sculpt_begin(BKSculpt *s, int brush, const double *at, double radius, double strength, int mirror, int invert);
void bk_sculpt_begin_brush(BKSculpt *s, const BKBrush *brush, const double *at);
void bk_sculpt_dab(BKSculpt *s, const double *at, double pressure, double size);
// A dab leant by a pen held at a slant: tilt in degrees (instead of the brush's), when a number.
void bk_sculpt_dab_tilted(BKSculpt *s, const double *at, double pressure, double size, double tilt);
void bk_sculpt_end(BKSculpt *s);
int bk_sculpt_undo(BKSculpt *s);  // 1 when there was a stroke to undo
int bk_sculpt_redo(BKSculpt *s);
// Brings the floats up to date: how many points moved or turned since last asked, and which (in order, until next asked);
// the same for the triangles' slots.
int bk_sculpt_sync(BKSculpt *s);
const uint32_t *bk_sculpt_changed(const BKSculpt *s);
int bk_sculpt_changed_triangle_count(const BKSculpt *s);
const uint32_t *bk_sculpt_changed_triangles(const BKSculpt *s);
int bk_sculpt_vertex_count(const BKSculpt *s);
int bk_sculpt_triangle_count(const BKSculpt *s);
int bk_sculpt_live_triangle_count(const BKSculpt *s);
const float *bk_sculpt_positions(const BKSculpt *s);
const float *bk_sculpt_normals(const BKSculpt *s);
const uint32_t *bk_sculpt_indices(const BKSculpt *s);
// The mesh as it is now, its points in use renumbered (free with bk_sculpt_mesh_free): what a body is made of.
BKSculptMesh *bk_sculpt_mesh(const BKSculpt *s);
// What's wrong with its links ("" when nothing): for tests.
const char *bk_sculpt_check(const BKSculpt *s);

// A human figure (Engine/Figure.hpp), standing on z = 0 facing −y (its left at +x), centred on its box: its numbers in
// this order (fewer: the rest standard for its sex). Sex 0 a man … 1 a woman; height in mm (standing straight, sole to
// crown, hair not counted); build to head as parts of the standard for its sex (1 = standard); then the pose, in
// degrees: the head's nod, turn and tilt; the torso's bend, twist and lean; each arm's raise (out to the side), forward
// and elbow; each leg's hip (forward), out and knee; each hand's wrist (bent toward the palm), curl (0 open … 100 a fist,
// in %) and spread (the fingers apart, %); last the hair: its style (as BK_HAIR_…) and volume (as a part of the
// standard). A mesh body, the same at any detail (its fingers, face and toes finer than the rest); a draft is coarser
// and quicker, for showing while a number is dragged.
enum {
  BK_FIG_SEX, BK_FIG_HEIGHT, BK_FIG_BUILD, BK_FIG_MUSCLE, BK_FIG_SHOULDERS, BK_FIG_CHEST, BK_FIG_WAIST, BK_FIG_HIPS, BK_FIG_ARMS,
  BK_FIG_LEGS, BK_FIG_HEAD, BK_FIG_NOD, BK_FIG_TURN, BK_FIG_TILT, BK_FIG_BEND, BK_FIG_TWIST, BK_FIG_LEAN, BK_FIG_LEFT_RAISE,
  BK_FIG_LEFT_FORWARD, BK_FIG_LEFT_ELBOW, BK_FIG_RIGHT_RAISE, BK_FIG_RIGHT_FORWARD, BK_FIG_RIGHT_ELBOW, BK_FIG_LEFT_HIP,
  BK_FIG_LEFT_OUT, BK_FIG_LEFT_KNEE, BK_FIG_RIGHT_HIP, BK_FIG_RIGHT_OUT, BK_FIG_RIGHT_KNEE, BK_FIG_LEFT_WRIST, BK_FIG_LEFT_CURL,
  BK_FIG_LEFT_SPREAD, BK_FIG_RIGHT_WRIST, BK_FIG_RIGHT_CURL, BK_FIG_RIGHT_SPREAD, BK_FIG_HAIR, BK_FIG_HAIR_VOLUME, BK_FIG_COUNT
};
enum { BK_HAIR_NONE, BK_HAIR_SHORT, BK_HAIR_BOB, BK_HAIR_LONG, BK_HAIR_PONYTAIL, BK_HAIR_BUN, BK_HAIR_AFRO, BK_HAIR_COUNT };
enum { BK_POSE_STAND, BK_POSE_T, BK_POSE_WALK, BK_POSE_SIT, BK_POSE_WAVE, BK_POSE_COUNT };
// The standard numbers for a man (sex 0) or a woman (1), standing (BK_FIG_COUNT of them).
void bk_figure_defaults(double sex, double *out);
// The least and the most a number may be (out[0], out[1]).
void bk_figure_range(int field, double *out);
// A pose's numbers set (the body's kept); which pose the numbers are in (-1: none of them).
void bk_figure_pose(int pose, double *params);
int bk_figure_pose_of(const double *params, int count);
BKShape *bk_figure(const double *params, int count, int draft);
// Its box (out[0…2], as its mesh's to the bit) and the point between its hips on the ground (out[3…5], from the box's
// middle), worked out at once without making it: 0 when the numbers aren't a figure's.
int bk_figure_extent(const double *params, int count, double *out);

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
// The edges picks stand for on a shape, as a rounding or bevel of them takes them: one edge pick each (6 numbers: a point
// at its middle, its direction there), at most `max` written to out (NULL to count). Returns how many there are.
int bk_pick_edges(const BKShape *s, const int *kinds, const double *picks, int count, double *out, int max);
// Hollows a solid inward, keeping its outside. open: faces removed as openings; walls: faces with their own thickness.
// Faces are given as 6 numbers each (normal, centroid). Without openings the result is closed with an inner void.
// sharp: the same solid with roundings left out (the last without any), tried in turn when the solid itself doesn't
// offset, and the last alone when faces have walls of their own: that one is hollowed, and only what lies inside the
// solid is kept.
BKShape *bk_hollow(const BKShape *s, const BKShape *const *sharp, int sharpCount, const double *open, int openCount, const double *walls,
                   const double *wallThickness, int wallCount, double thickness, int *missing);
// The shortest distance between two picked elements, each a point (x y z) or an edge or face of a shape by its index in
// bk_mesh's lists (0-based), the shape placed by m (row-major 3x4). out: the two closest points (x y z, x y z).
// -1 when it fails (bk_last_error says why).
enum { BK_END_POINT, BK_END_EDGE, BK_END_FACE };
double bk_distance(const BKShape *a, const double *ma, int kindA, int indexA, const double *pointA, const BKShape *b,
                   const double *mb, int kindB, int indexB, const double *pointB, double *out);
// Separate solids in a shape: a merge of parts that don't touch stays in pieces.
int bk_piece_count(const BKShape *s);
// The box the shape fills placed by m (row-major 3x4, turned or stretched any way): out = min xyz, max xyz. 1 when it's
// exact, 0 when part of it is only as close as the shape's display mesh, -1 when it fails (bk_last_error says why).
int bk_bounds(const BKShape *s, const double *m, double *out);
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
// A body as printers take it (3MF, STL): its mesh at 0.01 mm, points as the file holds them (float), every edge between
// exactly two triangles (where parts touch along a line or at a point, each part has its own copy of the points there).
// valid 0 when it isn't a sound solid (bk_last_error says how, as for bk_export_step); its triangles are still given.
typedef struct {
  int vertexCount, triangleCount;
  float *positions;                 // 3 per vertex
  uint32_t *indices;                // 3 per triangle
  double volume;                    // of the triangles as given
  int crowded;                      // edges still between more than two (where a part touches itself round a point)
  int slivers;                      // triangles thinner than float holds, left in (flat: corners all but in a line)
  int valid;
} BKPrintMesh;

BKPrintMesh *bk_print_mesh(const BKShape *s);
void bk_print_mesh_free(BKPrintMesh *m);
// A STEP file (AP214, millimetres) of the shapes, each a closed solid named as given (names may be NULL): flat faces as
// they are, curved ones as fine flat facets (0.01 mm). 0 when it can't be written: bk_last_error is then "file N: " and
// what's wrong with shape N ("empty", "open", "inside out"; for bk_print_mesh also "thin": a triangle float flattens or
// turns over, or two points it makes one), or "file: " and why nothing was written.
int bk_export_step(const BKShape *const *shapes, const char *const *names, int count, const char *path);

// A mesh from another app's or a scanner's file: its points (mm; those one to the last bit made one) and triangles as the
// file holds them, in parts (separate bodies where each is closed; otherwise one).
typedef struct {
  int vertexCount, triangleCount, partCount;
  float *positions;   // 3 per vertex
  uint32_t *indices;  // 3 per triangle
  int *partStart;     // partCount + 1 offsets (in triangles)
  char **partNames;   // per part ("" where the file names none)
  double scale;       // mm per unit of the file
  int unitGuessed;    // 1: the file says no unit and is under 2 units across, so taken as metres
  int skipped;        // faces left out (fewer than three corners)
} BKScanSoup;
// The most a file may hold (0: 50 million each).
typedef struct {
  double triangles, points;
} BKScanLimits;
// An STL (binary or text), OBJ or PLY file's bytes, by its extension. NULL when it can't be read (bk_last_error says why:
// "stl: cut short", "obj: line 12: not a number", "ply: has points but no surface", "scan: too large: …" …).
BKScanSoup *bk_scan_read(const uint8_t *bytes, int64_t length, const char *extension, const BKScanLimits *limits);
// A 3MF package from any app, unzipped: its parts' names (as in the package) and bytes, the model files and _rels/.rels.
// NULL when it can't be read ("3mf: …").
BKScanSoup *bk_scan_read_3mf(const char *const *names, const uint8_t *const *bytes, const int64_t *lengths, int count,
                             const BKScanLimits *limits);
void bk_scan_soup_free(BKScanSoup *s);
// How a mesh from a file is made a closed body (bk_scan_options sets the usual: 2 million triangles at most, holes
// filled, loose bits taken off, made again on a grid where nothing else will do). progress, where given, is told how far
// along it is (0 … 1) and stops it by answering 0.
typedef struct {
  int maxTriangles;       // simplified to at most this many (0: kept at any size)
  int fillHoles, removeIslands, remake;
  double islandShare;     // loose open bits of fewer of the triangles than this (and small beside the whole) taken off
  int (*progress)(void *context, double done);
  void *context;
} BKScanOptions;
void bk_scan_options(BKScanOptions *o);
// What was done to it.
typedef struct {
  int trianglesIn, pointsIn, trianglesOut, pointsOut;
  int welded;             // points made one with another a hair away
  int dropped;            // triangles left out: corners not numbers or not there, a corner twice
  int duplicates;         // triangles there twice
  int flipped;            // triangles turned round to face out
  int crowded;            // edges more than two triangles met at
  int holes, largestHole; // holes closed, the most sides one had
  int islands;            // triangles of loose bits taken off
  int remade;             // 0 kept as it was; 1 a fold mended; 2 overlapping parts made one; 3 made again on a grid; 4 made
                          // solid on a grid (it couldn't be closed as it was)
  double remadeDetail;    // the grid's spacing then (mm)
  int simplifiedFrom;     // triangles before it was simplified (0: it wasn't)
  double deviation;       // the furthest the simplified surface strays from the full one (mm)
  double offset[3];       // where its middle was: the mesh is made about the origin, to be placed back by this
  double size[3], volume;
  double detail;          // the detail to sculpt it at (mm)
} BKScanReport;
// A mesh from a file (bk_scan_read's, or a part of it) made a closed body bk_mesh_shape takes. NULL when it can't be
// (bk_last_error says why: "scan: larger than 10 m", "scan: can't be closed: …", "scan: stopped" …).
BKSculptMesh *bk_scan_repair(const float *positions, int vertexCount, const uint32_t *indices, int triangleCount, const BKScanOptions *options,
                             BKScanReport *report);

// MARK: sketches
// A sketch: points in a plane (x y each, mm), curves through them — a line from p[0] to p[1], an arc round centre p[0]
// counter-clockwise from p[1] to p[2] (its radius |p[1] − p[0]|), a circle round p[0] of `radius` — and rules on them:
// constraints and dimensions (BK_RULE_*, BK_DIM_*: the points p and curves c each uses, -1 where unused; `value` in mm or
// degrees; `side` ±1 for which of two ways it holds, as found when it was made). Construction curves bound no region;
// reference curves (projected from a face) are fixed where they are. Points marked fixed don't move.
enum { BK_CURVE_LINE = 0, BK_CURVE_ARC = 1, BK_CURVE_CIRCLE = 2 };
enum { BK_CURVE_CONSTRUCTION = 1, BK_CURVE_REFERENCE = 2 };
typedef struct {
  int kind;
  int p[3];
  double radius;
  int flags;
} BKCurve;
enum {
  BK_RULE_COINCIDENT = 0,    // p0 p1
  BK_RULE_ON = 1,            // p0 on curve c0 (its line, circle)
  BK_RULE_HORIZONTAL = 2,    // line c0, or p0 p1
  BK_RULE_VERTICAL = 3,      // line c0, or p0 p1
  BK_RULE_PARALLEL = 4,      // lines c0 c1
  BK_RULE_PERPENDICULAR = 5, // lines c0 c1
  BK_RULE_TANGENT = 6,       // c0 c1 (a line and an arc or circle, or two arcs or circles)
  BK_RULE_EQUAL = 7,         // c0 c1 (two lines' lengths, or two arcs' or circles' radii)
  BK_RULE_CONCENTRIC = 8,    // arcs or circles c0 c1
  BK_RULE_MIDPOINT = 9,      // p0 at the middle of line c0
  BK_RULE_COLLINEAR = 10,    // lines c0 c1
  BK_RULE_SYMMETRIC = 11,    // p0 p1 mirrored in line c0
  BK_RULE_FIX = 12,          // p0 held where it is
  BK_DIM_DISTANCE = 13,      // p0 p1, straight
  BK_DIM_HORIZONTAL = 14,    // p0 p1, along x
  BK_DIM_VERTICAL = 15,      // p0 p1, along y
  BK_DIM_POINT_LINE = 16,    // p0 to line c0
  BK_DIM_LINES = 17,         // line c1's middle to line c0 (parallel lines apart)
  BK_DIM_LENGTH = 18,        // line c0
  BK_DIM_RADIUS = 19,        // arc or circle c0
  BK_DIM_DIAMETER = 20,      // arc or circle c0
  BK_DIM_ANGLE = 21          // lines c0 c1 (degrees)
};
typedef struct {
  int kind;
  int p[3];
  int c[2];
  double value;
  int side;
} BKRule;
typedef struct {
  int pointCount, curveCount, ruleCount;
  double *points;               // x y each (the solver moves them)
  const unsigned char *fixed;   // per point, may be NULL
  BKCurve *curves;              // (circles' radii solved too)
  const BKRule *rules;
} BKSketch;

// The rules made to hold, moving the points (and circles' radii) as little as they can — the points `drag` names towards
// `targets` (x y each) first, as a hand dragging them. BK_SOLVE_OK with the sketch moved; BK_SOLVE_FAILED with it left as
// it was when the rules can't all hold (or the sketch isn't one: bk_last_error says why). The report: how many ways the
// sketch can still move (`freedom`); the first rule that holds nothing the others don't (`dependent`, -1 none) —
// `conflicting` when it can't hold with them; the largest any rule is off by; and, in arrays of the caller's (may be
// NULL), whether each point and curve is held where it is.
enum { BK_SOLVE_OK = 0, BK_SOLVE_FAILED = 1 };
typedef struct {
  int status, freedom, dependent, conflicting;
  double residual;
  unsigned char *pointFixed, *curveFixed;
} BKSolveReport;
int bk_sketch_solve(BKSketch *s, int dragCount, const int *drag, const double *targets, BKSolveReport *report);

// The regions a sketch's curves bound (construction curves left out): each one's area, a point inside it (`seed`), its
// sides (sorted: curve × 2, + 1 where it lies to the right of the curve's way — a line's p0 → p1, an arc's or circle's
// counter-clockwise), its outline then holes as closed polylines (first point not repeated) and triangles to show it.
typedef struct {
  int regionCount;
  double *area, *seed;            // per region; x y per region
  int *sideStart, *sides;         // regionCount + 1 offsets into sides
  int *loopStart, *pointStart;    // regionCount + 1 offsets into loops; loops + 1 offsets into points
  double *points;                 // x y each
  int *triangleStart;             // regionCount + 1 offsets (in triangles)
  double *triangles;              // x y × 3 each
} BKRegions;
BKRegions *bk_sketch_regions(const BKSketch *s, double deflection);
void bk_sketch_regions_free(BKRegions *r);
// Regions chosen earlier (each by its sides and seed: sideStart refCount + 1 offsets into sides, seeds x y each) found
// again in the sketch as it is now: regionOut[k] is its region's number now, or -1 where it's gone. 0 when the sketch
// isn't one (bk_last_error says why).
int bk_sketch_match(const BKSketch *s, const int *sideStart, const int *sides, const double *seeds, int refCount, int *regionOut);

// What's made of chosen regions: stood up from `low` to `high` mm along the sketch's +z, or turned about line curve
// `axis` from `low` to `high` degrees (the right-hand way round the line's p0 → p1).
enum { BK_FORM_EXTRUDE = 0, BK_FORM_REVOLVE = 1 };
typedef struct {
  int kind;
  double low, high;
  int axis;
} BKForm;
// The solid, in the sketch's own coordinates (not centred). NULL: bk_last_error "sketch: …".
BKShape *bk_sketch_solid(const BKSketch *s, const int *sideStart, const int *sides, const double *seeds, int refCount, const BKForm *form);

const char *bk_last_error(void);

#ifdef __cplusplus
}
#endif
#endif
