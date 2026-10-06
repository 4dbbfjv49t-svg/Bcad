// ISO metric coarse threads (ISO 261) with bolt heads and nut shapes: their sizes, and how each size fits the others.
#include "Engine/Fasteners.hpp"

#include <algorithm>
#include <cmath>

#include "Engine/Trig.hpp"

// Per thread: the sizes each head and nut starts from, standard ones where there is a standard.
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

const TorxSize torxSizes[] = {{10, 2.80, 2.05}, {15, 3.35, 2.40}, {20, 3.95, 2.85}, {25, 4.50, 3.25}, {27, 5.10, 3.65},
                                     {30, 5.60, 4.05}, {40, 6.75, 4.85}, {45, 7.93, 5.64}, {50, 8.95, 6.45}, {55, 11.35, 8.05},
                                     {60, 13.45, 9.60}, {70, 15.70, 11.20}, {80, 17.75, 12.80}, {90, 20.20, 14.40}, {100, 22.40, 16.00}};
const int torxCount = sizeof(torxSizes) / sizeof(torxSizes[0]);
const TorxSize *torxOf(double n) {
  for (const auto &t : torxSizes)
    if (fabs(t.n - n) < 1e-6) return &t;
  return nullptr;
}

// A Phillips recess per PH size: the width of its wings, and the recess diameter and depth it usually has.
const double phWing[5] = {0, 0.85, 1.25, 1.8, 2.4}, phRecess[5] = {0, 3.2, 5.0, 6.8, 8.9}, phDepth[5] = {0, 1.8, 2.8, 3.5, 4.6};


int bk_thread_count(void) { return sizeCount; }
const char *bk_thread_name(int size) { return size >= 0 && size < sizeCount ? sizes[size].name : ""; }
int bk_torx_count(void) { return torxCount; }
int bk_torx_number(int i) { return i >= 0 && i < torxCount ? torxSizes[i].n : 0; }

bool isNut(int k) { return k >= BK_SLEEVE; }
bool countersunk(int k) { return k == BK_SOCKET_CONE || k == BK_TORX_CONE || k == BK_PH_CONE; }
bool coneBelow(int k) { return k == BK_HEX_CONE || k == BK_TWELVE_CONE || k == BK_PH_HEX_CONE; }
bool phillipsDrive(int k) { return k == BK_PH_HEX || k == BK_PH_HEX_CONE || k == BK_PH_CONE; }
bool torxDrive(int k) { return k == BK_TORX || k == BK_TORX_CONE; }
bool keyDrive(int k) { return k == BK_SOCKET || k == BK_SOCKET_CONE; }
const ThreadSize &threadOf(int size) { return sizes[std::max(0, std::min(sizeCount - 1, size))]; }

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
double coneTop(const BKFastener &f) { return countersunk(f.kind) ? f.width / 2 : 0.46 * f.width; }
double tanHalf(double angle) { return trig::tan(angle * M_PI / 360); }

// How far a bolt's cone reaches below where its head starts, down to the thread's outside (0 without a cone).
double coneDrop(const BKFastener &f, double d) {
  if (!countersunk(f.kind) && !coneBelow(f.kind)) return 0;
  return (coneTop(f) - d / 2) / tanHalf(f.angle);
}

// The drive's size across its outside: a hex key's corners, a Torx's lobes, a Phillips recess; 0 without one.
double driveSpan(const BKFastener &f) {
  if (keyDrive(f.kind)) return f.drive * 2 / sqrt(3.0);
  if (torxDrive(f.kind)) return torxOf(f.drive) ? torxOf(f.drive)->a : 0;
  if (phillipsDrive(f.kind)) return f.recess;
  return 0;
}

// How wide a hex key or Torx may be: half a millimetre of wall all round, in a countersunk head down at the drive's bottom.
static double driveRoom(const BKFastener &f) {
  return f.width - 1 - (countersunk(f.kind) ? 2 * f.depth * tanHalf(f.angle) : 0);
}

// The order sizes are fitted in after a change: each one depends only on the ones fitted after it (and the angle).
static const int fitOrder[] = {BK_DRIVE, BK_RECESS, BK_DEPTH, BK_HEIGHT, BK_SEAT, BK_ANGLE, BK_WIDTH, BK_LENGTH};

void bk_fastener_range(const BKFastener *fp, int field, int loose, double *out) {
  if (!fp || !out) return;
  BKFastener f = *fp;
  // Loose: the sizes fitted before this one may shrink to their least to make room for it.
  // (A field not among them: none.)
  for (int i = 0; loose && i < (int)(sizeof fitOrder / sizeof *fitOrder) && fitOrder[i] != field; i++) {
    switch (fitOrder[i]) {
    case BK_DRIVE: if (keyDrive(f.kind)) f.drive = 0.7; else if (torxDrive(f.kind)) f.drive = torxSizes[0].n; break;
    case BK_RECESS: f.recess = 2 * phWing[std::max(1, std::min(4, (int)lround(f.drive)))] + 0.4; break;
    case BK_DEPTH: f.depth = 0.3; break;
    case BK_HEIGHT: f.height = f.depth + 0.3; break;
    case BK_SEAT: f.seat = 0.2; break;
    }
  }
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
    hi = 1000;  // (A metre: far past any printer's bed; a ten-metre thread took a minute to make.)
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
      hi = std::min(hi, trig::atan((0.92 * w - d - 1) / (2 * f.seat)) * 360 / M_PI);
    } else {
      // The cone at least a pitch tall, and leaving two pitches of thread.
      double rise = coneTop(f) - d / 2;
      hi = std::min(hi, trig::atan(rise / p) * 360 / M_PI);
      lo = std::max(lo, trig::atan(rise / std::max(L - 2 * p, 1e-9)) * 360 / M_PI);
      if (csk) hi = std::min({hi, trig::atan((w - bottom - 1) / (2 * f.depth)) * 360 / M_PI, slim ? 180.0 : trig::atan((w - d) / (2 * f.depth)) * 360 / M_PI});
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
        if (x.a <= room + 1e-6) hi = x.n;
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

const char *fastenerMisfit(const BKFastener &f) {
  if (!(f.kind >= BK_ROD && f.kind <= BK_CONE_NUT)) return "unknown bolt or nut";
  if (!(f.size >= 0 && f.size < sizeCount)) return "unknown thread";
  const double v[] = {f.length, f.width, f.height, f.angle, f.seat, f.drive, f.recess, f.depth};
  for (double x : v)
    if (!std::isfinite(x)) return "sizes must be numbers";
  int fields = bk_fastener_fields(f.kind);
  if (torxDrive(f.kind) && !torxOf(f.drive)) return "unknown Torx size";
  if (phillipsDrive(f.kind) && !(f.drive == 1 || f.drive == 2 || f.drive == 3 || f.drive == 4)) return "unknown PH size";
  for (int i = BK_LENGTH; i <= BK_DEPTH; i++) {
    if (!(fields & (1 << i))) continue;
    double r[2];
    bk_fastener_range(&f, i, 0, r);
    if (i == BK_LENGTH && v[i] > r[1] + 1e-6) return "a bolt or nut is at most 1000 mm long";
    if (!(v[i] >= r[0] - 1e-6 && v[i] <= r[1] + 1e-6)) return misfit(i);
  }
  return nullptr;
}

// Each size of the kind in turn into what the others leave it (after one size was changed, the ones depending on it follow).
void bk_fastener_fit(BKFastener *f) {
  double *v[] = {&f->length, &f->width, &f->height, &f->angle, &f->seat, &f->drive, &f->recess, &f->depth};
  int fields = bk_fastener_fields(f->kind);
  for (int i : fitOrder) {
    if (!(fields & (1 << i))) continue;
    double r[2];
    bk_fastener_range(f, i, 0, r);
    // A range that closes up at a single value comes out a hair empty or not by rounding.
    if (r[0] > r[1] + 1e-6) continue;
    r[1] = std::max(r[0], r[1]);
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
