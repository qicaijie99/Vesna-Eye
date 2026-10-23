// Validation metrics / plots for tracker output (placeholder).
//
// Owned by the validation workstream.
//
//   glowtrack_eval --track results/track.csv [--gt data/synthetic/gt.csv]
//                  --out results/eval [--label name]
//
// Produces: metrics.csv / metrics.md, error plots (PNG) and a printed summary.

#include <cstdio>

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  std::fprintf(stderr, "glowtrack_eval: not implemented yet\n");
  return 1;
}
