// Synthetic validation dataset generator (placeholder).
//
// Owned by the validation workstream. The real tool renders the green glowing
// target at a known pose into image sequences and writes a ground-truth CSV:
//
//   frame,timestamp_s,yaw_deg,pitch_deg,distance_m,roll_deg,tilt_deg,visible
//
// It is used for closed-loop accuracy validation of detector + PnP + filter.

#include <cstdio>

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  std::fprintf(stderr, "glowtrack_synth: not implemented yet\n");
  return 1;
}
