// Human figures (Figure.cpp): a man or a woman, its sizes, pose and hair set by numbers (as BK_FIG_…), made as one closed
// mesh body: smooth parts round a skeleton (each blended into the next only near the joint they share, so a hand hanging
// by the hip stays apart from it, and the fingers from each other), its surface found on a grid, finer at the hands, the
// face and the toes.
#pragma once
#include "Engine/Model.hpp"

#include <string>

namespace bce {

enum { FigureNumbers = 37 };  // BK_FIG_COUNT

struct FigureSpec {
  double v[FigureNumbers];
};

// The standard numbers for a sex (0 a man … 1 a woman), standing.
void figureDefaults(double sex, double *out);
void figureRange(int field, double &lo, double &hi);
// A pose's numbers set in v (as BK_POSE_…); which pose v is in (-1: none).
void figurePose(int pose, double *v);
int figurePoseOf(const double *v);
// The numbers given (`count` of them, the rest standard for the sex given) checked: false with `why` when one isn't a
// number or is out of its range.
bool figureSpec(const double *p, int count, FigureSpec &out, std::string &why);
// Its box's size and the point between its hips on the ground (from the box's middle), from its parts at once: its
// mesh's box is exactly this.
void figureBox(const FigureSpec &s, V3 &size, V3 &anchor);
// The figure as a body centred on its box: at its full detail, or a quicker draft. (The last few made are kept.)
bool figure(const FigureSpec &s, bool draft, Shape &out, std::string &why);

}  // namespace bce
