// ISO metric coarse threads with bolt heads and nut shapes: the sizes, and how each fits the others (Fasteners.cpp).
#pragma once
#include "BcadKernel.h"

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

// Torx (hexalobular, ISO 10664): size, across the lobes (A) and across the valleys (B).
struct TorxSize {
  int n;
  double a, b;
};

extern const TorxSize torxSizes[];
extern const int torxCount;
// A Phillips recess per PH size: the width of its wings, and the recess diameter and depth it usually has.
extern const double phWing[5], phRecess[5], phDepth[5];
// Basic thread depth per pitch (ISO 68-1): from the major to the minor radius.
constexpr double rootDepth = 0.5412658774;

const TorxSize *torxOf(double n);
const ThreadSize &threadOf(int size);
bool isNut(int k);
bool countersunk(int k);
bool coneBelow(int k);
bool phillipsDrive(int k);
bool torxDrive(int k);
bool keyDrive(int k);
// The radius where a bolt's cone meets its head (a countersunk head's rim; just inside the flats of a head above a cone).
double coneTop(const BKFastener &f);
double tanHalf(double angle);
// How far a bolt's cone reaches below where its head starts, down to the thread's outside (0 without a cone).
double coneDrop(const BKFastener &f, double d);
// The drive's size across its outside: a hex key's corners, a Torx's lobes, a Phillips recess; 0 without one.
double driveSpan(const BKFastener &f);
// Why a size of f doesn't fit what the others leave it (or the kind, thread or drive is unknown); null when all fit.
const char *fastenerMisfit(const BKFastener &f);
