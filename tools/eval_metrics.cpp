// Validation metrics + plots for Vesna-Eye CSV logs.
//
//   glowtrack_eval --track <tracker.csv> [--gt <gt.csv>] [--config <yaml>]
//                  --out <dir> [--label <name>] [--no-plots]
//
// With --gt (synthetic closed loop) it computes accuracy (yaw/pitch/distance
// errors vs ground truth), detection/valid coverage, prediction-through-dropout
// error, filter jitter reduction and disc-centre pixel accuracy, and renders
// time-series / histogram / jitter-bar plots.
//
// Without --gt (real video) it computes detection/valid/measured rates, state
// distribution, jitter, rate statistics, gap statistics, reprojection RMS stats
// and -- when --config supplies the camera intrinsics -- the deviation between
// the raw PnP line-of-sight and the bearing of the fitted ellipse centre.
//
// Outputs (in <out>):
//   metrics.csv  machine readable metric table
//   metrics.md   human readable report section (the Lead copies it into the
//                final validation report)
//   *.png        plots
//   matched_series.csv  frame-by-frame comparison (only with --gt)
//
// Tracker CSV schema (from src/output_sink.cpp):
//   frame,timestamp_s,state,detected,valid,measured,predicted,yaw_deg,pitch_deg,
//   yaw_rate_dps,pitch_rate_dps,distance_m,quality,reproj_rmse_px,track_id,
//   raw_yaw_deg,raw_pitch_deg,ellipse_cx,ellipse_cy,semi_major,semi_minor,
//   ellipse_angle_deg
// GT CSV schema (from tools/synth_dataset.cpp):
//   frame,timestamp_s,yaw_deg,pitch_deg,distance_m,roll_deg,tilt_deg,visible,
//   disc_cx,disc_cy,disc_radius_px
//
// Owned by the validation workstream; see docs/validation_synthetic.md.

#include "glowtrack/util.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kPi = 3.14159265358979323846;
constexpr double kRad2Deg = 180.0 / kPi;

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------
std::string trim(const std::string& s) {
  size_t b = 0;
  size_t e = s.size();
  while (b < e && static_cast<unsigned char>(s[b]) <= ' ') ++b;
  while (e > b && static_cast<unsigned char>(s[e - 1]) <= ' ') --e;
  return s.substr(b, e - b);
}

std::string lower(std::string s) {
  for (char& c : s) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return s;
}

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : line) {
    if (c == ',') {
      out.push_back(trim(cur));
      cur.clear();
    } else if (c != '\r') {
      cur.push_back(c);
    }
  }
  out.push_back(trim(cur));
  return out;
}

bool parseDouble(const std::string& text, double* value) {
  if (text.empty()) return false;
  char* end = nullptr;
  const double v = std::strtod(text.c_str(), &end);
  if (end == text.c_str()) return false;
  if (*end != '\0' && *end != ' ' && *end != '\r' && *end != '\t') return false;
  *value = v;
  return true;
}

double cellDouble(const std::vector<std::string>& row, int col) {
  if (col < 0 || col >= static_cast<int>(row.size())) return kNaN;
  double v = kNaN;
  if (!parseDouble(row[static_cast<size_t>(col)], &v)) return kNaN;
  return v;
}

bool cellBool(const std::vector<std::string>& row, int col) {
  if (col < 0 || col >= static_cast<int>(row.size())) return false;
  const std::string v = lower(trim(row[static_cast<size_t>(col)]));
  if (v.empty()) return false;
  if (v == "true" || v == "yes") return true;
  if (v == "false" || v == "no") return false;
  double d = kNaN;
  if (parseDouble(v, &d)) return d != 0.0;
  return false;
}

std::string fmtValue(double v, int precision = 6) {
  if (!std::isfinite(v)) return "nan";
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.*g", precision, v);
  return buf;
}

std::string fmtFixed(double v, int decimals = 4) {
  if (!std::isfinite(v)) return "n/a";
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.*f", decimals, v);
  return buf;
}

double wrapDeg(double deg) {
  double d = std::fmod(deg + 180.0, 360.0);
  if (d < 0.0) d += 360.0;
  return d - 180.0;
}

double percentage(double num, double den) {
  return den > 0.0 ? 100.0 * num / den : kNaN;
}

// Error statistics of a signed error sequence.
struct Stat {
  int n = 0;
  double rmse = kNaN;
  double mean = kNaN;     // bias
  double mean_abs = kNaN;
  double median_abs = kNaN;
  double max_abs = kNaN;
  double p95_abs = kNaN;
  double stddev = kNaN;
  double min = kNaN;
  double max = kNaN;
};

Stat statsOf(std::vector<double> values) {
  Stat s;
  values.erase(std::remove_if(values.begin(), values.end(),
                              [](double v) { return !std::isfinite(v); }),
               values.end());
  s.n = static_cast<int>(values.size());
  if (values.empty()) return s;
  double sum = 0.0;
  double sum2 = 0.0;
  double sum_abs = 0.0;
  s.min = values[0];
  s.max = values[0];
  for (double v : values) {
    sum += v;
    sum2 += v * v;
    sum_abs += std::fabs(v);
    s.min = std::min(s.min, v);
    s.max = std::max(s.max, v);
  }
  const double n = static_cast<double>(values.size());
  s.mean = sum / n;
  s.rmse = std::sqrt(sum2 / n);
  s.mean_abs = sum_abs / n;
  double var = 0.0;
  for (double v : values) var += (v - s.mean) * (v - s.mean);
  s.stddev = values.size() > 1 ? std::sqrt(var / (n - 1.0)) : 0.0;
  std::vector<double> abs_values = values;
  for (double& v : abs_values) v = std::fabs(v);
  std::sort(abs_values.begin(), abs_values.end());
  s.median_abs = abs_values[abs_values.size() / 2];
  const size_t p95_idx =
      std::min(abs_values.size() - 1,
               static_cast<size_t>(std::ceil(0.95 * n) - 1.0));
  s.p95_abs = abs_values[p95_idx];
  s.max_abs = abs_values.back();
  return s;
}

// Standard deviation / mean absolute value of the 2nd / 1st discrete
// differences of a signal with holes. `frames` gives the frame index of each
// sample; differences are only taken over consecutive frames.
struct Jitter {
  int n2 = 0;
  double second_diff_std = kNaN;
  int n1 = 0;
  double first_diff_mean_abs = kNaN;
};

Jitter jitterOf(const std::vector<double>& values,
                const std::vector<int>& frames) {
  std::vector<double> d2;
  std::vector<double> d1;
  for (size_t i = 1; i + 1 < values.size(); ++i) {
    if (!std::isfinite(values[i - 1]) || !std::isfinite(values[i]) ||
        !std::isfinite(values[i + 1])) {
      continue;
    }
    if (frames[i] != frames[i - 1] + 1 || frames[i + 1] != frames[i] + 1) {
      continue;
    }
    d2.push_back(values[i + 1] - 2.0 * values[i] + values[i - 1]);
    d1.push_back(values[i] - values[i - 1]);
  }
  Jitter j;
  const Stat s2 = statsOf(d2);
  j.n2 = s2.n;
  j.second_diff_std = s2.n > 1 ? s2.stddev : kNaN;
  const Stat s1 = statsOf(d1);
  j.n1 = s1.n;
  j.first_diff_mean_abs = s1.n > 0 ? s1.mean_abs : kNaN;
  return j;
}

// ---------------------------------------------------------------------------
// Generic CSV table
// ---------------------------------------------------------------------------
struct Table {
  std::vector<std::string> header;  // lowercase, trimmed (empty if no header)
  std::vector<std::vector<std::string>> rows;
  bool has_header = false;
};

bool loadTable(const std::string& path, Table* table, std::string* error) {
  std::ifstream in(path);
  if (!in) {
    *error = "cannot open " + path;
    return false;
  }
  std::string line;
  std::vector<std::pair<size_t, std::string>> non_empty;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    const std::string t = trim(line);
    if (!t.empty()) {
      non_empty.emplace_back(non_empty.size(), t);
    }
  }
  if (non_empty.empty()) {
    *error = "file is empty: " + path;
    return false;
  }
  // Header detection: first non-empty line contains letters and a known token.
  const std::string first_lower = lower(non_empty.front().second);
  if (first_lower.find("frame") != std::string::npos &&
      first_lower.find(',') != std::string::npos) {
    table->has_header = true;
    for (const auto& token : splitCsv(non_empty.front().second)) {
      table->header.push_back(lower(token));
    }
    non_empty.erase(non_empty.begin());
  }
  table->rows.reserve(non_empty.size());
  for (const auto& item : non_empty) {
    auto cells = splitCsv(item.second);
    // Skip a duplicate header that may appear if the file was appended twice.
    if (!table->header.empty() && cells.size() == table->header.size() &&
        lower(trim(cells[0])) == table->header[0]) {
      continue;
    }
    table->rows.push_back(std::move(cells));
  }
  return true;
}

// Returns the column index for the first alias found in the header (or -1).
int findColumn(const Table& table,
               std::initializer_list<const char*> aliases) {
  if (!table.has_header) return -1;
  for (const char* alias : aliases) {
    const std::string name = lower(alias);
    for (size_t i = 0; i < table.header.size(); ++i) {
      if (table.header[i] == name) return static_cast<int>(i);
    }
  }
  return -1;
}

// ---------------------------------------------------------------------------
// Tracker / ground-truth rows
// ---------------------------------------------------------------------------
struct TrackRow {
  int frame = -1;
  double timestamp = 0.0;
  std::string state = "UNKNOWN";
  bool detected = false;
  bool valid = false;      // effective: flag && finite angles
  bool measured = false;
  bool predicted = false;
  double yaw = kNaN;
  double pitch = kNaN;
  double yaw_rate = kNaN;
  double pitch_rate = kNaN;
  double distance = kNaN;
  double quality = kNaN;
  double reproj = kNaN;
  int track_id = -1;
  double raw_yaw = kNaN;
  double raw_pitch = kNaN;
  double ellipse_cx = kNaN;
  double ellipse_cy = kNaN;
  double semi_major = kNaN;
  double semi_minor = kNaN;
  double ellipse_angle = kNaN;
};

struct GtRow {
  int frame = -1;
  double timestamp = 0.0;
  double yaw = kNaN;
  double pitch = kNaN;
  double distance = kNaN;
  double roll = kNaN;
  double tilt = kNaN;
  bool visible = true;
  double disc_cx = kNaN;
  double disc_cy = kNaN;
  double disc_radius = kNaN;
};

bool parseTrackTable(const Table& table, std::vector<TrackRow>* rows,
                     std::string* warning) {
  const bool h = table.has_header;
  const int c_frame = findColumn(table, {"frame", "frame_index", "idx"});
  const int c_time = findColumn(table, {"timestamp_s", "timestamp", "time_s"});
  const int c_state = findColumn(table, {"state", "track_state"});
  const int c_detected = findColumn(table, {"detected", "det"});
  const int c_valid = findColumn(table, {"valid"});
  const int c_measured = findColumn(table, {"measured", "meas"});
  const int c_predicted = findColumn(table, {"predicted", "pred"});
  const int c_yaw = findColumn(table, {"yaw_deg", "yaw"});
  const int c_pitch = findColumn(table, {"pitch_deg", "pitch"});
  const int c_yaw_rate =
      findColumn(table, {"yaw_rate_dps", "yaw_rate_deg_s", "yaw_rate"});
  const int c_pitch_rate =
      findColumn(table, {"pitch_rate_dps", "pitch_rate_deg_s", "pitch_rate"});
  const int c_distance = findColumn(table, {"distance_m", "distance", "range_m"});
  const int c_quality = findColumn(table, {"quality"});
  const int c_reproj =
      findColumn(table, {"reproj_rmse_px", "reproj_rmse", "reproj"});
  const int c_track_id = findColumn(table, {"track_id", "track"});
  const int c_raw_yaw = findColumn(table, {"raw_yaw_deg", "raw_yaw"});
  const int c_raw_pitch = findColumn(table, {"raw_pitch_deg", "raw_pitch"});
  const int c_ex = findColumn(table, {"ellipse_cx", "disc_cx"});
  const int c_ey = findColumn(table, {"ellipse_cy", "disc_cy"});
  const int c_ax = findColumn(table, {"semi_major"});
  const int c_ay = findColumn(table, {"semi_minor"});
  const int c_angle = findColumn(table, {"ellipse_angle_deg", "ellipse_angle"});

  // Positional fallback (old stub schema order, no header).
  if (!h) {
    *warning =
        "tracker CSV has no header row: assuming the documented column order";
  }
  int frame_value = 0;
  for (const auto& cells : table.rows) {
    if (cells.empty()) continue;
    TrackRow r;
    if (h) {
      const double f = cellDouble(cells, c_frame);
      r.frame = std::isfinite(f) ? static_cast<int>(std::lround(f)) : frame_value;
      r.timestamp = cellDouble(cells, c_time);
      if (c_state >= 0 && c_state < static_cast<int>(cells.size())) {
        r.state = trim(cells[static_cast<size_t>(c_state)]);
      }
      r.detected = cellBool(cells, c_detected);
      const bool valid_flag = c_valid >= 0 ? cellBool(cells, c_valid)
                                           : std::isfinite(cellDouble(cells, c_yaw));
      r.measured = cellBool(cells, c_measured);
      r.predicted = cellBool(cells, c_predicted);
      r.yaw = cellDouble(cells, c_yaw);
      r.pitch = cellDouble(cells, c_pitch);
      r.yaw_rate = cellDouble(cells, c_yaw_rate);
      r.pitch_rate = cellDouble(cells, c_pitch_rate);
      r.distance = cellDouble(cells, c_distance);
      r.quality = cellDouble(cells, c_quality);
      r.reproj = cellDouble(cells, c_reproj);
      const double id = cellDouble(cells, c_track_id);
      r.track_id = std::isfinite(id) ? static_cast<int>(std::lround(id)) : -1;
      r.raw_yaw = cellDouble(cells, c_raw_yaw);
      r.raw_pitch = cellDouble(cells, c_raw_pitch);
      r.ellipse_cx = cellDouble(cells, c_ex);
      r.ellipse_cy = cellDouble(cells, c_ey);
      r.semi_major = cellDouble(cells, c_ax);
      r.semi_minor = cellDouble(cells, c_ay);
      r.ellipse_angle = cellDouble(cells, c_angle);
      r.valid = valid_flag && std::isfinite(r.yaw) && std::isfinite(r.pitch);
    } else {
      r.frame = static_cast<int>(std::lround(cellDouble(cells, 0)));
      r.timestamp = cellDouble(cells, 1);
      if (cells.size() > 2) r.state = trim(cells[2]);
      r.detected = cellBool(cells, 3);
      const bool valid_flag = cellBool(cells, 4);
      r.measured = cellBool(cells, 5);
      r.predicted = cellBool(cells, 6);
      r.yaw = cellDouble(cells, 7);
      r.pitch = cellDouble(cells, 8);
      r.yaw_rate = cellDouble(cells, 9);
      r.pitch_rate = cellDouble(cells, 10);
      r.distance = cellDouble(cells, 11);
      r.quality = cellDouble(cells, 12);
      r.reproj = cellDouble(cells, 13);
      r.track_id = static_cast<int>(std::lround(cellDouble(cells, 14)));
      r.raw_yaw = cellDouble(cells, 15);
      r.raw_pitch = cellDouble(cells, 16);
      r.ellipse_cx = cellDouble(cells, 17);
      r.ellipse_cy = cellDouble(cells, 18);
      r.semi_major = cellDouble(cells, 19);
      r.semi_minor = cellDouble(cells, 20);
      r.ellipse_angle = cellDouble(cells, 21);
      r.valid = valid_flag && std::isfinite(r.yaw) && std::isfinite(r.pitch);
    }
    if (!std::isfinite(r.timestamp)) r.timestamp = 0.0;
    if (r.frame < 0) r.frame = frame_value;
    ++frame_value;
    rows->push_back(r);
  }
  if (rows->empty()) {
    *warning = "no data rows found in tracker CSV";
    return false;
  }
  return true;
}

bool parseGtTable(const Table& table, std::vector<GtRow>* rows,
                  std::string* warning) {
  const bool h = table.has_header;
  const int c_frame = findColumn(table, {"frame", "frame_index"});
  const int c_time = findColumn(table, {"timestamp_s", "timestamp"});
  const int c_yaw = findColumn(table, {"yaw_deg", "yaw"});
  const int c_pitch = findColumn(table, {"pitch_deg", "pitch"});
  const int c_dist = findColumn(table, {"distance_m", "distance"});
  const int c_roll = findColumn(table, {"roll_deg", "roll"});
  const int c_tilt = findColumn(table, {"tilt_deg", "tilt"});
  const int c_visible = findColumn(table, {"visible"});
  const int c_cx = findColumn(table, {"disc_cx", "cx"});
  const int c_cy = findColumn(table, {"disc_cy", "cy"});
  const int c_r = findColumn(table, {"disc_radius_px", "radius_px", "disc_radius"});

  if (h && (c_yaw < 0 || c_pitch < 0)) {
    *warning = "GT CSV does not contain yaw_deg/pitch_deg columns";
    return false;
  }
  int order = 0;
  for (const auto& cells : table.rows) {
    if (cells.empty()) continue;
    GtRow r;
    if (h) {
      const double f = cellDouble(cells, c_frame);
      r.frame = std::isfinite(f) ? static_cast<int>(std::lround(f)) : order;
      r.timestamp = cellDouble(cells, c_time);
      r.yaw = cellDouble(cells, c_yaw);
      r.pitch = cellDouble(cells, c_pitch);
      r.distance = cellDouble(cells, c_dist);
      r.roll = cellDouble(cells, c_roll);
      r.tilt = cellDouble(cells, c_tilt);
      r.visible = c_visible >= 0 ? cellBool(cells, c_visible) : true;
      r.disc_cx = cellDouble(cells, c_cx);
      r.disc_cy = cellDouble(cells, c_cy);
      r.disc_radius = cellDouble(cells, c_r);
    } else {
      r.frame = static_cast<int>(std::lround(cellDouble(cells, 0)));
      r.timestamp = cellDouble(cells, 1);
      r.yaw = cellDouble(cells, 2);
      r.pitch = cellDouble(cells, 3);
      r.distance = cellDouble(cells, 4);
      r.roll = cellDouble(cells, 5);
      r.tilt = cellDouble(cells, 6);
      r.visible = cellBool(cells, 7);
      r.disc_cx = cellDouble(cells, 8);
      r.disc_cy = cellDouble(cells, 9);
      r.disc_radius = cellDouble(cells, 10);
    }
    if (!std::isfinite(r.yaw) || !std::isfinite(r.pitch)) {
      // Rows without a pose are not usable as ground truth.
      continue;
    }
    if (!std::isfinite(r.timestamp)) r.timestamp = 0.0;
    if (r.distance <= 0.0) r.distance = kNaN;
    rows->push_back(r);
    ++order;
  }
  if (rows->empty()) {
    *warning = "no usable data rows found in GT CSV";
    return false;
  }
  return true;
}

// output_sink writes raw_yaw_deg/raw_pitch_deg as 0/0 when the PnP pose was
// invalid, so this pair is the validity marker for the raw pose columns.
// (reproj_rmse_px cannot be used: on noise-free synthetic data it is genuinely
// ~0 px.)
bool rawPoseValid(const TrackRow& r) {
  if (!r.detected) return false;
  if (!std::isfinite(r.raw_yaw) || !std::isfinite(r.raw_pitch)) return false;
  return !(r.raw_yaw == 0.0 && r.raw_pitch == 0.0);
}

// ---------------------------------------------------------------------------
// Optional config (camera intrinsics for the LOS cross-check)
// ---------------------------------------------------------------------------
struct Intrinsics {
  int width = 640;
  int height = 480;
  double fx = 500.0;
  double fy = 500.0;
  double cx = 320.0;
  double cy = 240.0;
  std::vector<double> dist;
  bool loaded = false;
};

struct ConfigValues {
  Intrinsics camera;
  double max_reproj_rmse_px = 4.0;
  bool loaded = false;
};

bool loadConfig(const std::string& path, ConfigValues* out,
                std::string* warning) {
  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    *warning = "cannot open config " + path;
    return false;
  }
  cv::FileNode camera = fs["camera"];
  if (!camera.isMap()) {
    camera = fs.root();
  }
  auto readNum = [&](const cv::FileNode& node, const char* key,
                     double* dst) -> bool {
    if (node.empty()) return false;
    const cv::FileNode child = node[key];
    if (child.empty()) return false;
    if (child.isReal()) {
      *dst = static_cast<double>(child);
      return true;
    }
    if (child.isInt()) {
      *dst = static_cast<double>(static_cast<int>(child));
      return true;
    }
    return false;
  };
  double width = 0, height = 0;
  bool any = false;
  any |= readNum(camera, "width", &width);
  any |= readNum(camera, "height", &height);
  any |= readNum(camera, "fx", &out->camera.fx);
  any |= readNum(camera, "fy", &out->camera.fy);
  any |= readNum(camera, "cx", &out->camera.cx);
  any |= readNum(camera, "cy", &out->camera.cy);
  if (width > 0.0) out->camera.width = static_cast<int>(std::lround(width));
  if (height > 0.0) out->camera.height = static_cast<int>(std::lround(height));
  if (!camera.empty()) {
    const cv::FileNode dist = camera["dist"];
    if (dist.isSeq()) {
      out->camera.dist.clear();
      for (auto it = dist.begin(); it != dist.end(); ++it) {
        out->camera.dist.push_back(static_cast<double>(*it));
      }
    }
  }
  // Calibration YAML form (glowtrack --mode calibrate output).
  const cv::FileNode cam_matrix = fs["camera_matrix"];
  if (!cam_matrix.empty()) {
    cv::Mat m;
    cam_matrix >> m;
    if (m.rows >= 3 && m.cols >= 3) {
      out->camera.fx = m.at<double>(0, 0);
      out->camera.fy = m.at<double>(1, 1);
      out->camera.cx = m.at<double>(0, 2);
      out->camera.cy = m.at<double>(1, 2);
      any = true;
    }
  }
  const cv::FileNode distortion = fs["distortion_coefficients"];
  if (!distortion.empty()) {
    cv::Mat d;
    distortion >> d;
    if (!d.empty()) {
      cv::Mat flat = d.reshape(1, 1);
      out->camera.dist.clear();
      for (int i = 0; i < flat.cols; ++i) {
        out->camera.dist.push_back(flat.at<double>(0, i));
      }
    }
  }
  const cv::FileNode pose = fs["pose"];
  if (!pose.empty()) {
    readNum(pose, "max_reproj_rmse_px", &out->max_reproj_rmse_px);
  }
  out->camera.loaded = any;
  out->loaded = any;
  if (!any) {
    *warning = "no camera intrinsics found in " + path;
  }
  return any;
}

// ---------------------------------------------------------------------------
// Plot rendering (OpenCV primitives only)
// ---------------------------------------------------------------------------
const cv::Scalar kColorGt(255, 120, 0);      // blue-ish
const cv::Scalar kColorEst(0, 0, 235);       // red
const cv::Scalar kColorRaw(0, 170, 0);       // green
const cv::Scalar kColorAux(180, 90, 20);     // teal-ish
const cv::Scalar kColorErrYaw(0, 0, 220);    // red
const cv::Scalar kColorErrPitch(220, 80, 0); // blue-ish
const cv::Scalar kAxis(60, 60, 60);
const cv::Scalar kGrid(225, 225, 225);
const cv::Scalar kText(30, 30, 30);

struct LineSeries {
  std::vector<double> x;
  std::vector<double> y;
  std::string label;
  cv::Scalar color = kColorEst;
  int thickness = 2;
};

struct Canvas {
  cv::Mat img;
  cv::Rect area;
  double x0 = 0, x1 = 1, y0 = 0, y1 = 1;
  std::string xlabel, ylabel, title;
  bool draw_x_numbers = true;

  Canvas(int w, int h, const std::string& t, const std::string& xl,
         const std::string& yl)
      : img(h, w, CV_8UC3, cv::Scalar(255, 255, 255)),
        area(84, 52, w - 84 - 26, h - 52 - 70),
        xlabel(xl),
        ylabel(yl),
        title(t) {}

  void finish() {
    cv::putText(img, title, cv::Point(area.x + 4, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.62, kText, 2, cv::LINE_AA);
    cv::putText(img, xlabel,
                cv::Point(area.x + area.width / 2 - 20,
                          area.y + area.height + 54),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, kText, 1, cv::LINE_AA);
    cv::putText(img, ylabel, cv::Point(8, 48),
                cv::FONT_HERSHEY_SIMPLEX, 0.42, kText, 1, cv::LINE_AA);
    cv::rectangle(img, area, kAxis, 2);
  }
};

void drawTicks(Canvas& c) {
  for (int k = 0; k <= 5; ++k) {
    const double fx = static_cast<double>(k) / 5.0;
    const double fy = static_cast<double>(k) / 5.0;
    // vertical grid / x ticks
    const int px = c.area.x + static_cast<int>(std::lround(fx * c.area.width));
    cv::line(c.img, cv::Point(px, c.area.y),
             cv::Point(px, c.area.y + c.area.height), kGrid, 1, cv::LINE_AA);
    if (c.draw_x_numbers) {
      const double xv = c.x0 + fx * (c.x1 - c.x0);
      const std::string xs = fmtValue(xv, 4);
      int baseline = 0;
      const cv::Size ts = cv::getTextSize(xs, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1,
                                          &baseline);
      cv::putText(c.img, xs,
                  cv::Point(px - ts.width / 2, c.area.y + c.area.height + 20),
                  cv::FONT_HERSHEY_SIMPLEX, 0.4, kText, 1, cv::LINE_AA);
    }
    // horizontal grid / y ticks
    const int py = c.area.y + c.area.height -
                   static_cast<int>(std::lround(fy * c.area.height));
    cv::line(c.img, cv::Point(c.area.x, py),
             cv::Point(c.area.x + c.area.width, py), kGrid, 1, cv::LINE_AA);
    const double yv = c.y0 + fy * (c.y1 - c.y0);
    const std::string ys = fmtValue(yv, 4);
    int baseline = 0;
    const cv::Size ysz =
        cv::getTextSize(ys, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
    cv::putText(c.img, ys,
                cv::Point(c.area.x - 8 - ysz.width, py + 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, kText, 1, cv::LINE_AA);
  }
}

void drawSeries(Canvas& c, const LineSeries& s) {
  if (s.x.size() < 2 && s.y.size() < 2) return;
  const size_t n = std::min(s.x.size(), s.y.size());
  auto mapPoint = [&](size_t i) {
    const double px = c.area.x + (s.x[i] - c.x0) / (c.x1 - c.x0) * c.area.width;
    const double py =
        c.area.y + c.area.height -
        (s.y[i] - c.y0) / (c.y1 - c.y0) * c.area.height;
    return cv::Point(static_cast<int>(std::lround(px)),
                     static_cast<int>(std::lround(py)));
  };
  bool pen_down = false;
  cv::Point prev;
  for (size_t i = 0; i < n; ++i) {
    if (!std::isfinite(s.x[i]) || !std::isfinite(s.y[i])) {
      pen_down = false;
      continue;
    }
    const cv::Point pt = mapPoint(i);
    if (pen_down) {
      cv::line(c.img, prev, pt, s.color, s.thickness, cv::LINE_AA);
    }
    prev = pt;
    pen_down = true;
  }
}

void drawLegend(Canvas& c, const std::vector<LineSeries>& series) {
  std::vector<const LineSeries*> entries;
  for (const auto& s : series) {
    bool has_data = false;
    for (double v : s.y) {
      if (std::isfinite(v)) {
        has_data = true;
        break;
      }
    }
    if (has_data) entries.push_back(&s);
  }
  if (entries.empty()) return;
  int max_width = 0;
  for (const auto* s : entries) {
    int baseline = 0;
    const cv::Size ts = cv::getTextSize(s->label, cv::FONT_HERSHEY_SIMPLEX, 0.42,
                                        1, &baseline);
    max_width = std::max(max_width, ts.width);
  }
  const int line_h = 20;
  const int box_w = max_width + 44;
  const int box_h = line_h * static_cast<int>(entries.size()) + 10;
  const int x = c.area.x + c.area.width - box_w - 10;
  const int y = c.area.y + 10;
  cv::rectangle(c.img, cv::Rect(x, y, box_w, box_h), cv::Scalar(255, 255, 255),
                cv::FILLED);
  cv::rectangle(c.img, cv::Rect(x, y, box_w, box_h), cv::Scalar(150, 150, 150),
                1);
  for (size_t i = 0; i < entries.size(); ++i) {
    const int yy = y + 8 + static_cast<int>(i) * line_h;
    cv::line(c.img, cv::Point(x + 8, yy + 5), cv::Point(x + 34, yy + 5),
             entries[i]->color, 3, cv::LINE_AA);
    cv::putText(c.img, entries[i]->label, cv::Point(x + 40, yy + 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.42, kText, 1, cv::LINE_AA);
  }
}

double finiteMinMax(const std::vector<LineSeries>& series, bool x_axis,
                    double* out_min, double* out_max) {
  double mn = std::numeric_limits<double>::infinity();
  double mx = -std::numeric_limits<double>::infinity();
  for (const auto& s : series) {
    const std::vector<double>& v = x_axis ? s.x : s.y;
    for (double value : v) {
      if (std::isfinite(value)) {
        mn = std::min(mn, value);
        mx = std::max(mx, value);
      }
    }
  }
  if (!std::isfinite(mn) || !std::isfinite(mx)) return false;
  *out_min = mn;
  *out_max = mx;
  return true;
}

void expandRange(double* mn, double* mx, double min_span) {
  if (*mx - *mn < min_span) {
    const double mid = 0.5 * (*mn + *mx);
    *mn = mid - 0.5 * min_span;
    *mx = mid + 0.5 * min_span;
  }
}

void renderEmptyPlot(const std::string& path, const std::string& title) {
  cv::Mat img(320, 800, CV_8UC3, cv::Scalar(255, 255, 255));
  cv::putText(img, title, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.6,
              kText, 2, cv::LINE_AA);
  cv::putText(img, "no data", cv::Point(20, 90), cv::FONT_HERSHEY_SIMPLEX, 0.7,
              cv::Scalar(0, 0, 200), 2, cv::LINE_AA);
  cv::imwrite(path, img);
}

void renderLinePlot(const std::string& path, const std::string& title,
                    const std::string& xlabel, const std::string& ylabel,
                    const std::vector<LineSeries>& series) {
  double x0 = 0, x1 = 1, y0 = 0, y1 = 1;
  if (!finiteMinMax(series, true, &x0, &x1) ||
      !finiteMinMax(series, false, &y0, &y1)) {
    renderEmptyPlot(path, title);
    return;
  }
  expandRange(&x0, &x1, 1e-6);
  expandRange(&y0, &y1, 1e-6);
  const double ypad = 0.06 * (y1 - y0);
  y0 -= ypad;
  y1 += ypad;
  Canvas c(980, 560, title, xlabel, ylabel);
  c.x0 = x0;
  c.x1 = x1;
  c.y0 = y0;
  c.y1 = y1;
  drawTicks(c);
  for (const auto& s : series) drawSeries(c, s);
  drawLegend(c, series);
  c.finish();
  cv::imwrite(path, c.img);
}

void renderHistogram(const std::string& path, const std::string& title,
                     const std::string& xlabel,
                     const std::vector<double>& values, int bins,
                     const cv::Scalar& color) {
  std::vector<double> v;
  v.reserve(values.size());
  for (double value : values) {
    if (std::isfinite(value)) v.push_back(value);
  }
  if (v.empty()) {
    renderEmptyPlot(path, title);
    return;
  }
  std::sort(v.begin(), v.end());
  double x0 = v.front();
  double x1 = v.back();
  expandRange(&x0, &x1, 1e-3);
  std::vector<double> counts(static_cast<size_t>(bins), 0.0);
  for (double value : v) {
    int idx = static_cast<int>((value - x0) / (x1 - x0) * bins);
    idx = std::clamp(idx, 0, bins - 1);
    counts[static_cast<size_t>(idx)] += 1.0;
  }
  double ymax = 0.0;
  for (double cnt : counts) ymax = std::max(ymax, cnt);
  ymax = ymax > 0 ? ymax * 1.12 : 1.0;
  Canvas c(980, 520, title, xlabel, "count");
  c.x0 = x0;
  c.x1 = x1;
  c.y0 = 0.0;
  c.y1 = ymax;
  drawTicks(c);
  for (int i = 0; i < bins; ++i) {
    const double xa = x0 + (x1 - x0) * i / bins;
    const double xb = x0 + (x1 - x0) * (i + 1) / bins;
    if (counts[static_cast<size_t>(i)] <= 0.0) continue;
    const int px0 = c.area.x +
                    static_cast<int>(std::lround((xa - x0) / (x1 - x0) * c.area.width));
    const int px1 = c.area.x +
                    static_cast<int>(std::lround((xb - x0) / (x1 - x0) * c.area.width));
    const int py = c.area.y + c.area.height -
                   static_cast<int>(std::lround(
                       counts[static_cast<size_t>(i)] / ymax * c.area.height));
    cv::rectangle(c.img, cv::Rect(px0 + 1, py, std::max(1, px1 - px0 - 1),
                                  c.area.y + c.area.height - py),
                  color, cv::FILLED);
  }
  // mean marker
  double sum = 0.0;
  for (double value : v) sum += value;
  const double mean = sum / static_cast<double>(v.size());
  const int mx = c.area.x + static_cast<int>(
                                 std::lround((mean - x0) / (x1 - x0) * c.area.width));
  cv::line(c.img, cv::Point(mx, c.area.y), cv::Point(mx, c.area.y + c.area.height),
           cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
  cv::putText(c.img, "mean " + fmtFixed(mean, 3) + "  n=" + std::to_string(v.size()),
              cv::Point(c.area.x + 10, c.area.y + 20),
              cv::FONT_HERSHEY_SIMPLEX, 0.45, kText, 1, cv::LINE_AA);
  c.finish();
  cv::imwrite(path, c.img);
}

struct BarItem {
  std::string label;
  double value = kNaN;
  cv::Scalar color = kColorEst;
};

void renderBarPlot(const std::string& path, const std::string& title,
                   const std::string& ylabel, const std::vector<BarItem>& bars) {
  double ymax = 0.0;
  bool any = false;
  for (const auto& b : bars) {
    if (std::isfinite(b.value)) {
      ymax = std::max(ymax, b.value);
      any = true;
    }
  }
  if (!any) {
    renderEmptyPlot(path, title);
    return;
  }
  ymax = ymax > 0.0 ? ymax * 1.2 : 1.0;
  Canvas c(980, 520, title, "", ylabel);
  c.x0 = 0.0;
  c.x1 = static_cast<double>(bars.size());
  c.y0 = 0.0;
  c.y1 = ymax;
  c.draw_x_numbers = false;
  drawTicks(c);
  const int n = static_cast<int>(bars.size());
  const double step = c.area.width / static_cast<double>(std::max(n, 1));
  for (int i = 0; i < n; ++i) {
    const int px0 = c.area.x + static_cast<int>(std::lround(step * i + step * 0.18));
    const int px1 = c.area.x + static_cast<int>(std::lround(step * i + step * 0.82));
    int py = c.area.y + c.area.height;
    if (std::isfinite(bars[static_cast<size_t>(i)].value)) {
      py = c.area.y + c.area.height -
           static_cast<int>(std::lround(bars[static_cast<size_t>(i)].value /
                                        ymax * c.area.height));
      cv::rectangle(c.img, cv::Rect(px0, py, std::max(1, px1 - px0),
                                    c.area.y + c.area.height - py),
                    bars[static_cast<size_t>(i)].color, cv::FILLED);
      const std::string vs = fmtValue(bars[static_cast<size_t>(i)].value, 4);
      int baseline = 0;
      const cv::Size ts =
          cv::getTextSize(vs, cv::FONT_HERSHEY_SIMPLEX, 0.42, 1, &baseline);
      cv::putText(c.img, vs,
                  cv::Point((px0 + px1) / 2 - ts.width / 2, py - 6),
                  cv::FONT_HERSHEY_SIMPLEX, 0.42, kText, 1, cv::LINE_AA);
    }
    const std::string& label = bars[static_cast<size_t>(i)].label;
    int baseline = 0;
    const cv::Size ts =
        cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
    cv::putText(c.img, label,
                cv::Point((px0 + px1) / 2 - ts.width / 2,
                          c.area.y + c.area.height + 22),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, kText, 1, cv::LINE_AA);
  }
  c.finish();
  cv::imwrite(path, c.img);
}

// ---------------------------------------------------------------------------
// Evaluation results
// ---------------------------------------------------------------------------
struct Evaluation {
  std::string label;
  std::string track_path;
  std::string gt_path;
  std::string config_path;
  bool has_gt = false;
  int track_rows = 0;
  int gt_rows = 0;
  int matched_rows = 0;
  int gt_visible = 0;
  int gt_invisible = 0;
  int gt_unmatched = 0;

  double detection_rate_visible = kNaN;
  double valid_rate_visible = kNaN;
  double measured_rate_visible = kNaN;
  double predicted_rate_visible = kNaN;
  int visible_detected = 0;
  int visible_valid = 0;
  int visible_measured = 0;
  int visible_predicted = 0;

  double detection_rate_all = kNaN;
  double valid_rate_all = kNaN;
  double measured_rate_all = kNaN;
  int total_detected = 0;
  int total_valid = 0;
  int total_measured = 0;

  double false_detection_rate_invisible = kNaN;

  Stat yaw;                  // measured-only accuracy (if GT)
  Stat pitch;
  Stat yaw_all;
  Stat pitch_all;
  Stat yaw_predicted;
  Stat pitch_predicted;
  Stat yaw_raw;              // raw PnP accuracy (detector + pose, no filter)
  Stat pitch_raw;
  int measured_accuracy_frames = 0;
  int predicted_accuracy_frames = 0;
  int raw_accuracy_frames = 0;

  Stat distance;
  Stat distance_rel;
  Stat center_err_px;
  Stat radius_rel_err_pct;

  // Prediction through dropouts.
  int gt_visible_predicted_frames = 0;
  Stat dropout_yaw;
  Stat dropout_pitch;
  int blind_prediction_frames = 0;
  Stat blind_yaw;
  Stat blind_pitch;
  int hold_frames = 0;
  Stat blind_hold_yaw;
  Stat blind_hold_pitch;

  Jitter jitter_yaw;
  Jitter jitter_pitch;
  Jitter jitter_raw_yaw;
  Jitter jitter_raw_pitch;
  double jitter_ratio_yaw = kNaN;
  double jitter_ratio_pitch = kNaN;

  Stat yaw_rate_abs;
  Stat pitch_rate_abs;
  double max_abs_yaw_rate = kNaN;
  double max_abs_pitch_rate = kNaN;
  Stat quality;
  Stat reproj;
  double reproj_pass_rate = kNaN;

  // Deviation of the filtered output from the raw PnP measurement on frames
  // where both exist (filter lag / smoothing / prediction bias).
  Stat filter_raw_yaw;
  Stat filter_raw_pitch;
  int filter_raw_frames = 0;

  std::map<std::string, int> state_counts;
  int detection_gap_count = 0;
  double detection_gap_mean = kNaN;
  double detection_gap_max = 0.0;
  int prediction_run_count = 0;
  double prediction_run_mean = kNaN;
  double prediction_run_max = 0.0;

  // LOS cross-check (no-GT mode).
  int los_frames = 0;
  Stat los_yaw_dev;
  Stat los_pitch_dev;
  double los_yaw_rmse = kNaN;
  double los_pitch_rmse = kNaN;

  ConfigValues config;
  bool config_loaded = false;
};

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------
struct Args {
  std::string track;
  std::string gt;
  std::string config;
  std::string out = "results/eval";
  std::string label;
  bool no_plots = false;
};

void printUsage() {
  std::printf(
      "glowtrack_eval -- validation metrics and plots for tracker CSV logs\n\n"
      "Usage:\n"
      "  glowtrack_eval --track <tracker.csv> [--gt <gt.csv>] [--config <yaml>]\n"
      "                 --out <dir> [--label <name>] [--no-plots]\n\n"
      "  --track    tracker CSV produced by glowtrack (required)\n"
      "  --gt       synthetic ground-truth CSV from glowtrack_synth\n"
      "  --config   tracker config YAML (camera intrinsics for the LOS check)\n"
      "  --out      output directory for metrics + plots (default results/eval)\n"
      "  --label    name used in plot titles / metrics.md (default: track stem)\n"
      "  --no-plots skip PNG plot rendering\n");
}

bool parseArgs(int argc, char** argv, Args* args) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "error: missing value for %s\n", name);
        return nullptr;
      }
      return argv[++i];
    };
    if (arg == "--help" || arg == "-h") {
      printUsage();
      std::exit(0);
    } else if (arg == "--track") {
      const char* v = next("--track");
      if (!v) return false;
      args->track = v;
    } else if (arg == "--gt") {
      const char* v = next("--gt");
      if (!v) return false;
      args->gt = v;
    } else if (arg == "--config") {
      const char* v = next("--config");
      if (!v) return false;
      args->config = v;
    } else if (arg == "--out") {
      const char* v = next("--out");
      if (!v) return false;
      args->out = v;
    } else if (arg == "--label") {
      const char* v = next("--label");
      if (!v) return false;
      args->label = v;
    } else if (arg == "--no-plots") {
      args->no_plots = true;
    } else {
      std::fprintf(stderr, "error: unknown argument '%s' (try --help)\n",
                   arg.c_str());
      return false;
    }
  }
  if (args->track.empty()) {
    std::fprintf(stderr, "error: --track <tracker.csv> is required\n");
    return false;
  }
  if (args->out.empty()) {
    std::fprintf(stderr, "error: --out <dir> is required\n");
    return false;
  }
  if (args->label.empty()) {
    args->label = std::filesystem::path(args->track).stem().string();
  }
  return true;
}

// ---------------------------------------------------------------------------
// Metric table helpers
// ---------------------------------------------------------------------------
struct MetricRow {
  std::string name;
  double value = kNaN;
  std::string unit;
  int count = 0;
};

void addMetric(std::vector<MetricRow>* rows, const std::string& name,
               double value, const std::string& unit, int count = 0) {
  rows->push_back(MetricRow{name, value, unit, count});
}

std::string valueOrNa(double v, int precision = 6) {
  return std::isfinite(v) ? fmtValue(v, precision) : "nan";
}

// ---------------------------------------------------------------------------
// Main evaluation
// ---------------------------------------------------------------------------
Evaluation evaluate(const Args& args) {
  Evaluation ev;
  ev.label = args.label;
  ev.track_path = args.track;
  ev.gt_path = args.gt;
  ev.config_path = args.config;

  std::string warning;
  Table track_table;
  if (!loadTable(args.track, &track_table, &warning)) {
    throw std::runtime_error(warning);
  }
  std::vector<TrackRow> track_rows;
  if (!parseTrackTable(track_table, &track_rows, &warning)) {
    throw std::runtime_error(warning);
  }
  if (!warning.empty()) {
    std::fprintf(stderr, "warning: %s\n", warning.c_str());
  }
  ev.track_rows = static_cast<int>(track_rows.size());

  std::vector<GtRow> gt_rows;
  std::unordered_map<int, size_t> gt_by_frame;
  bool matched_by_frame = false;
  if (!args.gt.empty()) {
    Table gt_table;
    if (!loadTable(args.gt, &gt_table, &warning)) {
      throw std::runtime_error(warning);
    }
    if (!parseGtTable(gt_table, &gt_rows, &warning)) {
      throw std::runtime_error(warning);
    }
    if (!warning.empty()) {
      std::fprintf(stderr, "warning: %s\n", warning.c_str());
    }
    ev.has_gt = true;
    ev.gt_rows = static_cast<int>(gt_rows.size());
    for (size_t i = 0; i < gt_rows.size(); ++i) {
      gt_by_frame[gt_rows[i].frame] = i;
    }
    int intersect = 0;
    for (const auto& r : track_rows) {
      if (gt_by_frame.count(r.frame)) ++intersect;
    }
    matched_by_frame = intersect > 0;
    if (!matched_by_frame) {
      std::fprintf(stderr,
                   "warning: no overlapping frame indices between tracker and "
                   "GT CSV; falling back to row order\n");
    }
  }

  // Effective flags (tolerate older CSV variants where a flag is missing).
  auto isPredicted = [](const TrackRow& r) {
    return r.predicted || lower(r.state) == "predict";
  };
  auto isMeasured = [&](const TrackRow& r) {
    return r.measured || (!isPredicted(r) && r.detected);
  };
  auto isDetected = [](const TrackRow& r) { return r.detected; };

  // Pair tracker rows with GT rows.
  struct Pair {
    const TrackRow* track = nullptr;
    const GtRow* gt = nullptr;
  };
  std::vector<Pair> pairs;
  pairs.reserve(track_rows.size());
  int gt_matched = 0;
  for (size_t i = 0; i < track_rows.size(); ++i) {
    Pair p;
    p.track = &track_rows[i];
    if (ev.has_gt) {
      if (matched_by_frame) {
        auto it = gt_by_frame.find(track_rows[i].frame);
        if (it != gt_by_frame.end()) p.gt = &gt_rows[it->second];
      } else if (i < gt_rows.size()) {
        p.gt = &gt_rows[i];
      }
      if (p.gt) ++gt_matched;
    }
    pairs.push_back(p);
  }
  ev.matched_rows = gt_matched;
  ev.gt_unmatched = ev.has_gt ? ev.gt_rows - gt_matched : 0;

  // Per-row series (for jitter / state / gaps), independent of GT matching.
  std::vector<double> row_yaw(track_rows.size(), kNaN);
  std::vector<double> row_pitch(track_rows.size(), kNaN);
  std::vector<double> row_raw_yaw(track_rows.size(), kNaN);
  std::vector<double> row_raw_pitch(track_rows.size(), kNaN);
  std::vector<int> row_frames(track_rows.size(), 0);
  std::vector<double> rate_yaw_abs;
  std::vector<double> rate_pitch_abs;
  for (size_t i = 0; i < track_rows.size(); ++i) {
    const TrackRow& r = track_rows[i];
    row_frames[i] = r.frame;
    if (r.valid) {
      row_yaw[i] = r.yaw;
      row_pitch[i] = r.pitch;
    }
    // output_sink writes raw_yaw_deg = raw_pitch_deg = 0 when the PnP pose was
    // invalid, so this pair is the validity marker for the raw pose columns
    // (reproj_rmse_px is legitimately ~0 px on noise-free synthetic data).
    const bool raw_ok = rawPoseValid(r);
    if (raw_ok) {
      row_raw_yaw[i] = r.raw_yaw;
      row_raw_pitch[i] = r.raw_pitch;
    }
    if (std::isfinite(r.yaw_rate)) rate_yaw_abs.push_back(std::fabs(r.yaw_rate));
    if (std::isfinite(r.pitch_rate)) {
      rate_pitch_abs.push_back(std::fabs(r.pitch_rate));
    }
    if (r.valid) {
      ++ev.total_valid;
      if (isMeasured(r)) ++ev.total_measured;
    }
    if (isDetected(r)) ++ev.total_detected;
  }
  ev.detection_rate_all = percentage(ev.total_detected, ev.track_rows);
  ev.valid_rate_all = percentage(ev.total_valid, ev.track_rows);
  ev.measured_rate_all = percentage(ev.total_measured, ev.track_rows);

  // Jitter (filtered vs raw).
  ev.jitter_yaw = jitterOf(row_yaw, row_frames);
  ev.jitter_pitch = jitterOf(row_pitch, row_frames);
  ev.jitter_raw_yaw = jitterOf(row_raw_yaw, row_frames);
  ev.jitter_raw_pitch = jitterOf(row_raw_pitch, row_frames);
  if (std::isfinite(ev.jitter_raw_yaw.second_diff_std) &&
      ev.jitter_raw_yaw.second_diff_std > 1e-12 &&
      std::isfinite(ev.jitter_yaw.second_diff_std)) {
    ev.jitter_ratio_yaw =
        ev.jitter_yaw.second_diff_std / ev.jitter_raw_yaw.second_diff_std;
  }
  if (std::isfinite(ev.jitter_raw_pitch.second_diff_std) &&
      ev.jitter_raw_pitch.second_diff_std > 1e-12 &&
      std::isfinite(ev.jitter_pitch.second_diff_std)) {
    ev.jitter_ratio_pitch =
        ev.jitter_pitch.second_diff_std / ev.jitter_raw_pitch.second_diff_std;
  }

  // Rate / quality / reprojection stats.
  ev.yaw_rate_abs = statsOf(rate_yaw_abs);
  ev.pitch_rate_abs = statsOf(rate_pitch_abs);
  ev.max_abs_yaw_rate = ev.yaw_rate_abs.max_abs;
  ev.max_abs_pitch_rate = ev.pitch_rate_abs.max_abs;
  {
    std::vector<double> quality_values;
    std::vector<double> reproj_values;
    for (const auto& r : track_rows) {
      if (std::isfinite(r.quality)) quality_values.push_back(r.quality);
      if (rawPoseValid(r) && std::isfinite(r.reproj)) {
        reproj_values.push_back(r.reproj);
      }
    }
    ev.quality = statsOf(quality_values);
    ev.reproj = statsOf(reproj_values);
  }

  // State distribution + gap statistics.
  int gap_len = 0;
  int run_len = 0;
  double gap_sum = 0.0;
  double run_sum = 0.0;
  for (const auto& r : track_rows) {
    ev.state_counts[r.state.empty() ? "UNKNOWN" : r.state] += 1;
    const bool gap = !isDetected(r);
    if (gap) {
      ++gap_len;
    } else if (gap_len > 0) {
      ++ev.detection_gap_count;
      gap_sum += gap_len;
      ev.detection_gap_max = std::max(ev.detection_gap_max, static_cast<double>(gap_len));
      gap_len = 0;
    }
    const bool pred_run = r.valid && isPredicted(r) && !isMeasured(r);
    if (pred_run) {
      ++run_len;
    } else if (run_len > 0) {
      ++ev.prediction_run_count;
      run_sum += run_len;
      ev.prediction_run_max =
          std::max(ev.prediction_run_max, static_cast<double>(run_len));
      run_len = 0;
    }
  }
  if (gap_len > 0) {
    ++ev.detection_gap_count;
    gap_sum += gap_len;
    ev.detection_gap_max = std::max(ev.detection_gap_max, static_cast<double>(gap_len));
  }
  if (run_len > 0) {
    ++ev.prediction_run_count;
    run_sum += run_len;
    ev.prediction_run_max = std::max(ev.prediction_run_max, static_cast<double>(run_len));
  }
  if (ev.detection_gap_count > 0) {
    ev.detection_gap_mean = gap_sum / ev.detection_gap_count;
  }
  if (ev.prediction_run_count > 0) {
    ev.prediction_run_mean = run_sum / ev.prediction_run_count;
  }

  // Config (camera intrinsics + acceptance thresholds), then the LOS
  // cross-check that depends on it.
  if (!args.config.empty()) {
    std::string config_warning;
    ev.config_loaded = loadConfig(args.config, &ev.config, &config_warning);
    if (!ev.config_loaded) {
      std::fprintf(stderr, "warning: %s\n", config_warning.c_str());
    }
  }
  {
    // Share of PnP poses below the configured reprojection-RMS limit.
    int pass = 0;
    int total = 0;
    for (const auto& r : track_rows) {
      if (rawPoseValid(r) && std::isfinite(r.reproj)) {
        ++total;
        if (r.reproj <= ev.config.max_reproj_rmse_px) ++pass;
      }
    }
    ev.reproj_pass_rate = percentage(pass, total);
  }
  {
    std::vector<double> yaw_dev;
    std::vector<double> pitch_dev;
    if (ev.config_loaded) {
      for (const auto& r : track_rows) {
        if (!rawPoseValid(r)) continue;
        if (!(r.semi_major > 0.0) || !std::isfinite(r.ellipse_cx)) continue;
        const double nx = (r.ellipse_cx - ev.config.camera.cx) / ev.config.camera.fx;
        const double ny = (r.ellipse_cy - ev.config.camera.cy) / ev.config.camera.fy;
        const double bearing_yaw = std::atan(nx) * kRad2Deg;
        const double bearing_pitch = -std::atan(ny) * kRad2Deg;
        yaw_dev.push_back(r.raw_yaw - bearing_yaw);
        pitch_dev.push_back(r.raw_pitch - bearing_pitch);
      }
      ev.los_yaw_dev = statsOf(yaw_dev);
      ev.los_pitch_dev = statsOf(pitch_dev);
      ev.los_frames = ev.los_yaw_dev.n;
      ev.los_yaw_rmse = ev.los_yaw_dev.rmse;
      ev.los_pitch_rmse = ev.los_pitch_dev.rmse;
    }
  }
  {
    // Filtered output vs raw PnP measurement (works with and without GT).
    std::vector<double> yaw_diff;
    std::vector<double> pitch_diff;
    for (const auto& r : track_rows) {
      if (!r.valid || !rawPoseValid(r)) continue;
      yaw_diff.push_back(wrapDeg(r.yaw - r.raw_yaw));
      pitch_diff.push_back(r.pitch - r.raw_pitch);
    }
    ev.filter_raw_yaw = statsOf(yaw_diff);
    ev.filter_raw_pitch = statsOf(pitch_diff);
    ev.filter_raw_frames = ev.filter_raw_yaw.n;
  }

  if (!ev.has_gt) {
    return ev;
  }

  // -------------------------------------------------------------------------
  // GT-matched metrics
  // -------------------------------------------------------------------------
  std::vector<double> yaw_err_measured;
  std::vector<double> pitch_err_measured;
  std::vector<double> yaw_err_all;
  std::vector<double> pitch_err_all;
  std::vector<double> yaw_err_pred;
  std::vector<double> pitch_err_pred;
  std::vector<double> yaw_err_raw;
  std::vector<double> pitch_err_raw;
  std::vector<double> dist_err;
  std::vector<double> dist_rel;
  std::vector<double> center_err;
  std::vector<double> radius_err;
  std::vector<double> dropout_yaw;
  std::vector<double> dropout_pitch;
  std::vector<double> blind_yaw;
  std::vector<double> blind_pitch;
  std::vector<double> hold_yaw;
  std::vector<double> hold_pitch;

  double last_measured_yaw = kNaN;
  double last_measured_pitch = kNaN;

  // The blind-prediction baseline needs "last measured value before the gap",
  // so walk the tracker rows in order and update it as we go.
  for (size_t i = 0; i < track_rows.size(); ++i) {
    const TrackRow& r = track_rows[i];
    const GtRow* gt = pairs[i].gt;
    if (!gt) continue;

    if (gt->visible) {
      ++ev.gt_visible;
      if (isDetected(r)) ++ev.visible_detected;
      if (r.valid) ++ev.visible_valid;
      if (r.valid && isMeasured(r)) ++ev.visible_measured;
      if (r.valid && isPredicted(r)) ++ev.visible_predicted;
    } else {
      ++ev.gt_invisible;
      if (isDetected(r)) {
        // False detection on a frame where the target was not rendered.
      }
    }

    if (gt->visible && r.valid && std::isfinite(r.yaw) && std::isfinite(r.pitch)) {
      const double ye = wrapDeg(r.yaw - gt->yaw);
      const double pe = r.pitch - gt->pitch;
      yaw_err_all.push_back(ye);
      pitch_err_all.push_back(pe);
      if (isMeasured(r)) {
        yaw_err_measured.push_back(ye);
        pitch_err_measured.push_back(pe);
      } else {
        yaw_err_pred.push_back(ye);
        pitch_err_pred.push_back(pe);
        if (isPredicted(r)) {
          dropout_yaw.push_back(ye);
          dropout_pitch.push_back(pe);
        }
      }
      if (rawPoseValid(r)) {
        yaw_err_raw.push_back(wrapDeg(r.raw_yaw - gt->yaw));
        pitch_err_raw.push_back(r.raw_pitch - gt->pitch);
      }
      if (std::isfinite(r.distance) && r.distance > 0.0 && std::isfinite(gt->distance) &&
          gt->distance > 0.0) {
        const double de = r.distance - gt->distance;
        dist_err.push_back(de);
        dist_rel.push_back(100.0 * de / gt->distance);
      }
    }

    if (gt->visible && isDetected(r) && std::isfinite(r.ellipse_cx) &&
        std::isfinite(r.ellipse_cy) && std::isfinite(gt->disc_cx) &&
        gt->disc_radius > 0.0 && r.semi_major > 0.0 && r.semi_minor > 0.0) {
      center_err.push_back(std::hypot(r.ellipse_cx - gt->disc_cx,
                                      r.ellipse_cy - gt->disc_cy));
      const double fitted_radius =
          std::sqrt(r.semi_major * r.semi_minor);
      radius_err.push_back(100.0 * (fitted_radius - gt->disc_radius) /
                           gt->disc_radius);
    }

    // Prediction through a real dropout: target not rendered but the tracker
    // still reports a valid (predicted) output.
    if (!gt->visible && r.valid && std::isfinite(r.yaw) && std::isfinite(r.pitch)) {
      ++ev.blind_prediction_frames;
      const double ye = wrapDeg(r.yaw - gt->yaw);
      const double pe = r.pitch - gt->pitch;
      blind_yaw.push_back(ye);
      blind_pitch.push_back(pe);
      if (std::isfinite(last_measured_yaw) && std::isfinite(last_measured_pitch)) {
        ++ev.hold_frames;
        hold_yaw.push_back(wrapDeg(last_measured_yaw - gt->yaw));
        hold_pitch.push_back(last_measured_pitch - gt->pitch);
      }
    }

    if (isMeasured(r) && r.valid && std::isfinite(r.yaw)) {
      last_measured_yaw = r.yaw;
      last_measured_pitch = r.pitch;
    }
  }
  ev.detection_rate_visible = percentage(ev.visible_detected, ev.gt_visible);
  ev.valid_rate_visible = percentage(ev.visible_valid, ev.gt_visible);
  ev.measured_rate_visible = percentage(ev.visible_measured, ev.gt_visible);
  ev.predicted_rate_visible = percentage(ev.visible_predicted, ev.gt_visible);
  {
    // Detection on frames where the synthetic target was not rendered.
    int false_detections = 0;
    for (const auto& p : pairs) {
      if (p.gt && !p.gt->visible && isDetected(*p.track)) ++false_detections;
    }
    ev.false_detection_rate_invisible =
        percentage(false_detections, ev.gt_invisible);
  }

  ev.yaw = statsOf(yaw_err_measured);
  ev.pitch = statsOf(pitch_err_measured);
  ev.yaw_all = statsOf(yaw_err_all);
  ev.pitch_all = statsOf(pitch_err_all);
  ev.yaw_predicted = statsOf(yaw_err_pred);
  ev.pitch_predicted = statsOf(pitch_err_pred);
  ev.yaw_raw = statsOf(yaw_err_raw);
  ev.pitch_raw = statsOf(pitch_err_raw);
  ev.measured_accuracy_frames = ev.yaw.n;
  ev.predicted_accuracy_frames = ev.yaw_predicted.n;
  ev.raw_accuracy_frames = ev.yaw_raw.n;
  ev.distance = statsOf(dist_err);
  ev.distance_rel = statsOf(dist_rel);
  ev.center_err_px = statsOf(center_err);
  ev.radius_rel_err_pct = statsOf(radius_err);
  ev.dropout_yaw = statsOf(dropout_yaw);
  ev.dropout_pitch = statsOf(dropout_pitch);
  ev.gt_visible_predicted_frames = ev.dropout_yaw.n;
  ev.blind_yaw = statsOf(blind_yaw);
  ev.blind_pitch = statsOf(blind_pitch);
  ev.blind_hold_yaw = statsOf(hold_yaw);
  ev.blind_hold_pitch = statsOf(hold_pitch);
  return ev;
}

// ---------------------------------------------------------------------------
// Reporting
// ---------------------------------------------------------------------------
std::vector<MetricRow> buildMetrics(const Evaluation& ev) {
  std::vector<MetricRow> m;
  addMetric(&m, "frames_total_track", ev.track_rows, "frames", ev.track_rows);
  if (ev.has_gt) {
    addMetric(&m, "frames_total_gt", ev.gt_rows, "frames", ev.gt_rows);
    addMetric(&m, "frames_matched", ev.matched_rows, "frames", ev.matched_rows);
    addMetric(&m, "gt_visible_frames", ev.gt_visible, "frames", ev.gt_visible);
    addMetric(&m, "gt_invisible_frames", ev.gt_invisible, "frames", ev.gt_invisible);
    addMetric(&m, "gt_unmatched_frames", ev.gt_unmatched, "frames", ev.gt_unmatched);
  }
  addMetric(&m, "detection_rate_all", ev.detection_rate_all, "%", ev.track_rows);
  addMetric(&m, "valid_rate_all", ev.valid_rate_all, "%", ev.track_rows);
  addMetric(&m, "measured_rate_all", ev.measured_rate_all, "%", ev.track_rows);
  if (ev.has_gt) {
    addMetric(&m, "detection_rate_visible", ev.detection_rate_visible, "%",
              ev.gt_visible);
    addMetric(&m, "valid_rate_visible", ev.valid_rate_visible, "%", ev.gt_visible);
    addMetric(&m, "measured_rate_visible", ev.measured_rate_visible, "%",
              ev.gt_visible);
    addMetric(&m, "predicted_rate_visible", ev.predicted_rate_visible, "%",
              ev.gt_visible);
    addMetric(&m, "false_detection_rate_invisible",
              ev.false_detection_rate_invisible, "%", ev.gt_invisible);

    addMetric(&m, "yaw_rmse_measured_deg", ev.yaw.rmse, "deg", ev.yaw.n);
    addMetric(&m, "yaw_bias_measured_deg", ev.yaw.mean, "deg", ev.yaw.n);
    addMetric(&m, "yaw_mae_measured_deg", ev.yaw.mean_abs, "deg", ev.yaw.n);
    addMetric(&m, "yaw_p95_abs_err_measured_deg", ev.yaw.p95_abs, "deg", ev.yaw.n);
    addMetric(&m, "yaw_max_abs_err_measured_deg", ev.yaw.max_abs, "deg", ev.yaw.n);
    addMetric(&m, "pitch_rmse_measured_deg", ev.pitch.rmse, "deg", ev.pitch.n);
    addMetric(&m, "pitch_bias_measured_deg", ev.pitch.mean, "deg", ev.pitch.n);
    addMetric(&m, "pitch_mae_measured_deg", ev.pitch.mean_abs, "deg", ev.pitch.n);
    addMetric(&m, "pitch_p95_abs_err_measured_deg", ev.pitch.p95_abs, "deg",
              ev.pitch.n);
    addMetric(&m, "pitch_max_abs_err_measured_deg", ev.pitch.max_abs, "deg",
              ev.pitch.n);
    addMetric(&m, "yaw_rmse_all_deg", ev.yaw_all.rmse, "deg", ev.yaw_all.n);
    addMetric(&m, "pitch_rmse_all_deg", ev.pitch_all.rmse, "deg", ev.pitch_all.n);
    addMetric(&m, "yaw_rmse_predicted_deg", ev.yaw_predicted.rmse, "deg",
              ev.yaw_predicted.n);
    addMetric(&m, "pitch_rmse_predicted_deg", ev.pitch_predicted.rmse, "deg",
              ev.pitch_predicted.n);
    addMetric(&m, "yaw_rmse_raw_deg", ev.yaw_raw.rmse, "deg", ev.yaw_raw.n);
    addMetric(&m, "yaw_bias_raw_deg", ev.yaw_raw.mean, "deg", ev.yaw_raw.n);
    addMetric(&m, "yaw_p95_abs_err_raw_deg", ev.yaw_raw.p95_abs, "deg",
              ev.yaw_raw.n);
    addMetric(&m, "pitch_rmse_raw_deg", ev.pitch_raw.rmse, "deg",
              ev.pitch_raw.n);
    addMetric(&m, "pitch_bias_raw_deg", ev.pitch_raw.mean, "deg",
              ev.pitch_raw.n);
    addMetric(&m, "pitch_p95_abs_err_raw_deg", ev.pitch_raw.p95_abs, "deg",
              ev.pitch_raw.n);
    addMetric(&m, "accuracy_frames_measured", ev.measured_accuracy_frames,
              "frames", ev.measured_accuracy_frames);
    addMetric(&m, "accuracy_frames_predicted", ev.predicted_accuracy_frames,
              "frames", ev.predicted_accuracy_frames);
    addMetric(&m, "accuracy_frames_raw", ev.raw_accuracy_frames, "frames",
              ev.raw_accuracy_frames);

    addMetric(&m, "distance_rmse_m", ev.distance.rmse, "m", ev.distance.n);
    addMetric(&m, "distance_bias_m", ev.distance.mean, "m", ev.distance.n);
    addMetric(&m, "distance_mae_m", ev.distance.mean_abs, "m", ev.distance.n);
    addMetric(&m, "distance_p95_abs_err_m", ev.distance.p95_abs, "m",
              ev.distance.n);
    addMetric(&m, "distance_max_abs_err_m", ev.distance.max_abs, "m",
              ev.distance.n);
    addMetric(&m, "distance_rel_rmse_pct", ev.distance_rel.rmse, "%",
              ev.distance_rel.n);
    addMetric(&m, "distance_rel_max_abs_pct", ev.distance_rel.max_abs, "%",
              ev.distance_rel.n);

    addMetric(&m, "disc_center_err_rmse_px", ev.center_err_px.rmse, "px",
              ev.center_err_px.n);
    addMetric(&m, "disc_center_err_p95_px", ev.center_err_px.p95_abs, "px",
              ev.center_err_px.n);
    addMetric(&m, "disc_center_err_max_px", ev.center_err_px.max_abs, "px",
              ev.center_err_px.n);
    addMetric(&m, "disc_radius_rel_rmse_pct", ev.radius_rel_err_pct.rmse, "%",
              ev.radius_rel_err_pct.n);

    addMetric(&m, "dropout_visible_predicted_frames",
              ev.gt_visible_predicted_frames, "frames",
              ev.gt_visible_predicted_frames);
    addMetric(&m, "dropout_pred_yaw_rmse_deg", ev.dropout_yaw.rmse, "deg",
              ev.dropout_yaw.n);
    addMetric(&m, "dropout_pred_pitch_rmse_deg", ev.dropout_pitch.rmse, "deg",
              ev.dropout_pitch.n);
    addMetric(&m, "blind_prediction_frames", ev.blind_prediction_frames, "frames",
              ev.blind_prediction_frames);
    addMetric(&m, "blind_pred_yaw_rmse_deg", ev.blind_yaw.rmse, "deg",
              ev.blind_yaw.n);
    addMetric(&m, "blind_pred_pitch_rmse_deg", ev.blind_pitch.rmse, "deg",
              ev.blind_pitch.n);
    addMetric(&m, "blind_hold_yaw_rmse_deg", ev.blind_hold_yaw.rmse, "deg",
              ev.blind_hold_yaw.n);
    addMetric(&m, "blind_hold_pitch_rmse_deg", ev.blind_hold_pitch.rmse, "deg",
              ev.blind_hold_pitch.n);
  }

  addMetric(&m, "jitter_filtered_yaw_2nd_std_deg", ev.jitter_yaw.second_diff_std,
            "deg", ev.jitter_yaw.n2);
  addMetric(&m, "jitter_raw_yaw_2nd_std_deg", ev.jitter_raw_yaw.second_diff_std,
            "deg", ev.jitter_raw_yaw.n2);
  addMetric(&m, "jitter_ratio_yaw", ev.jitter_ratio_yaw, "-", ev.jitter_yaw.n2);
  addMetric(&m, "jitter_filtered_pitch_2nd_std_deg",
            ev.jitter_pitch.second_diff_std, "deg", ev.jitter_pitch.n2);
  addMetric(&m, "jitter_raw_pitch_2nd_std_deg",
            ev.jitter_raw_pitch.second_diff_std, "deg", ev.jitter_raw_pitch.n2);
  addMetric(&m, "jitter_ratio_pitch", ev.jitter_ratio_pitch, "-",
            ev.jitter_pitch.n2);
  addMetric(&m, "jitter_filtered_yaw_1st_mad_deg",
            ev.jitter_yaw.first_diff_mean_abs, "deg", ev.jitter_yaw.n1);
  addMetric(&m, "jitter_raw_yaw_1st_mad_deg", ev.jitter_raw_yaw.first_diff_mean_abs,
            "deg", ev.jitter_raw_yaw.n1);
  addMetric(&m, "jitter_filtered_pitch_1st_mad_deg",
            ev.jitter_pitch.first_diff_mean_abs, "deg", ev.jitter_pitch.n1);
  addMetric(&m, "jitter_raw_pitch_1st_mad_deg",
            ev.jitter_raw_pitch.first_diff_mean_abs, "deg", ev.jitter_raw_pitch.n1);

  addMetric(&m, "max_abs_yaw_rate_dps", ev.max_abs_yaw_rate, "deg/s",
            ev.yaw_rate_abs.n);
  addMetric(&m, "p95_abs_yaw_rate_dps", ev.yaw_rate_abs.p95_abs, "deg/s",
            ev.yaw_rate_abs.n);
  addMetric(&m, "mean_abs_yaw_rate_dps", ev.yaw_rate_abs.mean_abs, "deg/s",
            ev.yaw_rate_abs.n);
  addMetric(&m, "max_abs_pitch_rate_dps", ev.max_abs_pitch_rate, "deg/s",
            ev.pitch_rate_abs.n);
  addMetric(&m, "p95_abs_pitch_rate_dps", ev.pitch_rate_abs.p95_abs, "deg/s",
            ev.pitch_rate_abs.n);
  addMetric(&m, "mean_abs_pitch_rate_dps", ev.pitch_rate_abs.mean_abs, "deg/s",
            ev.pitch_rate_abs.n);

  addMetric(&m, "quality_mean", ev.quality.mean, "-", ev.quality.n);
  addMetric(&m, "quality_min", ev.quality.min, "-", ev.quality.n);
  addMetric(&m, "reproj_rmse_mean_px", ev.reproj.mean, "px", ev.reproj.n);
  addMetric(&m, "reproj_rmse_median_px", ev.reproj.median_abs, "px", ev.reproj.n);
  addMetric(&m, "reproj_rmse_p95_px", ev.reproj.p95_abs, "px", ev.reproj.n);
  addMetric(&m, "reproj_rmse_max_px", ev.reproj.max_abs, "px", ev.reproj.n);
  addMetric(&m, "reproj_pass_rate_pct", ev.reproj_pass_rate, "%", ev.reproj.n);
  addMetric(&m, "filter_raw_yaw_diff_rmse_deg", ev.filter_raw_yaw.rmse, "deg",
            ev.filter_raw_yaw.n);
  addMetric(&m, "filter_raw_yaw_diff_mean_deg", ev.filter_raw_yaw.mean, "deg",
            ev.filter_raw_yaw.n);
  addMetric(&m, "filter_raw_yaw_diff_max_abs_deg", ev.filter_raw_yaw.max_abs,
            "deg", ev.filter_raw_yaw.n);
  addMetric(&m, "filter_raw_pitch_diff_rmse_deg", ev.filter_raw_pitch.rmse,
            "deg", ev.filter_raw_pitch.n);
  addMetric(&m, "filter_raw_pitch_diff_mean_deg", ev.filter_raw_pitch.mean,
            "deg", ev.filter_raw_pitch.n);
  addMetric(&m, "filter_raw_pitch_diff_max_abs_deg",
            ev.filter_raw_pitch.max_abs, "deg", ev.filter_raw_pitch.n);
  addMetric(&m, "filter_raw_frames", ev.filter_raw_frames, "frames",
            ev.filter_raw_frames);

  addMetric(&m, "detection_gap_count", ev.detection_gap_count, "count");
  addMetric(&m, "detection_gap_mean_frames", ev.detection_gap_mean, "frames");
  addMetric(&m, "detection_gap_max_frames",
            ev.detection_gap_count > 0 ? ev.detection_gap_max : kNaN, "frames");
  addMetric(&m, "prediction_run_count", ev.prediction_run_count, "count");
  addMetric(&m, "prediction_run_mean_frames", ev.prediction_run_mean, "frames");
  addMetric(&m, "prediction_run_max_frames",
            ev.prediction_run_count > 0 ? ev.prediction_run_max : kNaN, "frames");

  for (const auto& kv : ev.state_counts) {
    addMetric(&m, "state_" + lower(kv.first) + "_frames", kv.second, "frames",
              kv.second);
  }

  if (ev.config_loaded) {
    addMetric(&m, "intrinsics_fx", ev.config.camera.fx, "px");
    addMetric(&m, "intrinsics_fy", ev.config.camera.fy, "px");
    addMetric(&m, "intrinsics_cx", ev.config.camera.cx, "px");
    addMetric(&m, "intrinsics_cy", ev.config.camera.cy, "px");
    addMetric(&m, "los_crosscheck_frames", ev.los_frames, "frames", ev.los_frames);
    addMetric(&m, "los_yaw_dev_bias_deg", ev.los_yaw_dev.mean, "deg",
              ev.los_yaw_dev.n);
    addMetric(&m, "los_yaw_dev_std_deg", ev.los_yaw_dev.stddev, "deg",
              ev.los_yaw_dev.n);
    addMetric(&m, "los_yaw_dev_rmse_deg", ev.los_yaw_dev.rmse, "deg",
              ev.los_yaw_dev.n);
    addMetric(&m, "los_pitch_dev_bias_deg", ev.los_pitch_dev.mean, "deg",
              ev.los_pitch_dev.n);
    addMetric(&m, "los_pitch_dev_std_deg", ev.los_pitch_dev.stddev, "deg",
              ev.los_pitch_dev.n);
    addMetric(&m, "los_pitch_dev_rmse_deg", ev.los_pitch_dev.rmse, "deg",
              ev.los_pitch_dev.n);
  }
  return m;
}

void writeMetricsCsv(const Evaluation& ev, const std::vector<MetricRow>& metrics,
                     const std::string& path) {
  (void)ev;
  std::ofstream out(path);
  out << "metric,value,unit,count\n";
  for (const auto& row : metrics) {
    out << row.name << ',' << valueOrNa(row.value) << ',' << row.unit << ','
        << row.count << '\n';
  }
}

void writeMatchedSeries(const Evaluation& ev,
                        const std::vector<TrackRow>& track_rows,
                        const std::vector<GtRow>& gt_rows,
                        const std::unordered_map<int, size_t>& gt_by_frame,
                        bool matched_by_frame, const std::string& path) {
  if (!ev.has_gt) return;
  std::ofstream out(path);
  out << "frame,timestamp_s,gt_visible,gt_yaw_deg,est_yaw_deg,raw_yaw_deg,"
         "gt_pitch_deg,est_pitch_deg,raw_pitch_deg,yaw_err_deg,pitch_err_deg,"
         "gt_distance_m,est_distance_m,detected,valid,measured,predicted,state\n";
  for (size_t i = 0; i < track_rows.size(); ++i) {
    const TrackRow& r = track_rows[i];
    const GtRow* gt = nullptr;
    if (matched_by_frame) {
      auto it = gt_by_frame.find(r.frame);
      if (it != gt_by_frame.end()) gt = &gt_rows[it->second];
    } else if (i < gt_rows.size()) {
      gt = &gt_rows[i];
    }
    if (!gt) continue;
    auto num = [](double v) { return std::isfinite(v) ? fmtValue(v, 8) : std::string("nan"); };
    out << r.frame << ',' << valueOrNa(r.timestamp) << ',' << (gt->visible ? 1 : 0)
        << ',' << num(gt->yaw) << ',' << num(r.valid ? r.yaw : kNaN) << ','
        << num(r.raw_yaw) << ',' << num(gt->pitch) << ','
        << num(r.valid ? r.pitch : kNaN) << ',' << num(r.raw_pitch) << ','
        << num(r.valid ? wrapDeg(r.yaw - gt->yaw) : kNaN) << ','
        << num(r.valid ? r.pitch - gt->pitch : kNaN) << ','
        << num(gt->distance) << ',' << num(r.distance) << ','
        << (r.detected ? 1 : 0) << ',' << (r.valid ? 1 : 0) << ','
        << (r.measured ? 1 : 0) << ',' << (r.predicted ? 1 : 0) << ',' << r.state
        << '\n';
  }
}

struct PlotEntry {
  std::string path;
  std::string note;
};

std::vector<PlotEntry> renderPlots(const Evaluation& ev,
                                   const std::vector<TrackRow>& track_rows,
                                   const std::vector<GtRow>& gt_rows,
                                   const std::unordered_map<int, size_t>& gt_by_frame,
                                   bool matched_by_frame,
                                   const std::string& out_dir) {
  std::vector<PlotEntry> plots;
  auto path = [&](const std::string& name) {
    return (std::filesystem::path(out_dir) / name).string();
  };
  // Match rows again (same ordering logic as evaluate()).
  std::vector<const GtRow*> matched_gt;
  for (size_t i = 0; i < track_rows.size(); ++i) {
    const TrackRow& r = track_rows[i];
    const GtRow* gt = nullptr;
    if (matched_by_frame) {
      auto it = gt_by_frame.find(r.frame);
      if (it != gt_by_frame.end()) gt = &gt_rows[it->second];
    } else if (i < gt_rows.size()) {
      gt = &gt_rows[i];
    }
    matched_gt.push_back(gt);
  }
  std::vector<double> xs;
  xs.reserve(track_rows.size());
  bool use_time = true;
  for (size_t i = 0; i < track_rows.size(); ++i) {
    if (i > 0 && !(track_rows[i].timestamp > track_rows[i - 1].timestamp)) {
      use_time = false;
      break;
    }
  }
  for (size_t i = 0; i < track_rows.size(); ++i) {
    xs.push_back(use_time ? track_rows[i].timestamp
                          : static_cast<double>(track_rows[i].frame));
  }
  const std::string xlabel = use_time ? "time [s]" : "frame";

  if (ev.has_gt) {
    LineSeries gt_yaw, est_yaw, raw_yaw;
    LineSeries gt_pitch, est_pitch, raw_pitch;
    LineSeries err_yaw, err_pitch;
    LineSeries err_yaw_raw, err_pitch_raw;
    LineSeries gt_dist, est_dist;
    gt_yaw.label = "GT yaw";
    gt_yaw.color = kColorGt;
    gt_yaw.thickness = 3;
    est_yaw.label = "filtered yaw";
    est_yaw.color = kColorEst;
    raw_yaw.label = "raw PnP yaw";
    raw_yaw.color = kColorRaw;
    raw_yaw.thickness = 1;
    gt_pitch.label = "GT pitch";
    gt_pitch.color = kColorGt;
    gt_pitch.thickness = 3;
    est_pitch.label = "filtered pitch";
    est_pitch.color = kColorEst;
    raw_pitch.label = "raw PnP pitch";
    raw_pitch.color = kColorRaw;
    raw_pitch.thickness = 1;
    err_yaw.label = "yaw error (filtered)";
    err_yaw.color = kColorErrYaw;
    err_pitch.label = "pitch error (filtered)";
    err_pitch.color = kColorErrPitch;
    err_yaw_raw.label = "yaw error (raw PnP)";
    err_yaw_raw.color = cv::Scalar(0, 140, 255);
    err_yaw_raw.thickness = 1;
    err_pitch_raw.label = "pitch error (raw PnP)";
    err_pitch_raw.color = cv::Scalar(180, 0, 180);
    err_pitch_raw.thickness = 1;
    gt_dist.label = "GT distance";
    gt_dist.color = kColorGt;
    gt_dist.thickness = 3;
    est_dist.label = "filtered distance";
    est_dist.color = kColorEst;
    std::vector<double> yaw_errors;
    std::vector<double> pitch_errors;
    for (size_t i = 0; i < track_rows.size(); ++i) {
      const TrackRow& r = track_rows[i];
      const GtRow* gt = matched_gt[i];
      const double t = xs[i];
      if (gt && gt->visible) {
        gt_yaw.x.push_back(t);
        gt_yaw.y.push_back(gt->yaw);
        gt_pitch.x.push_back(t);
        gt_pitch.y.push_back(gt->pitch);
        gt_dist.x.push_back(t);
        gt_dist.y.push_back(gt->distance);
        if (r.valid) {
          err_yaw.x.push_back(t);
          err_yaw.y.push_back(wrapDeg(r.yaw - gt->yaw));
          err_pitch.x.push_back(t);
          err_pitch.y.push_back(r.pitch - gt->pitch);
          yaw_errors.push_back(wrapDeg(r.yaw - gt->yaw));
          pitch_errors.push_back(r.pitch - gt->pitch);
        }
        if (rawPoseValid(r)) {
          raw_yaw.x.push_back(t);
          raw_yaw.y.push_back(r.raw_yaw);
          raw_pitch.x.push_back(t);
          raw_pitch.y.push_back(r.raw_pitch);
          err_yaw_raw.x.push_back(t);
          err_yaw_raw.y.push_back(wrapDeg(r.raw_yaw - gt->yaw));
          err_pitch_raw.x.push_back(t);
          err_pitch_raw.y.push_back(r.raw_pitch - gt->pitch);
        }
      }
      if (r.valid) {
        est_yaw.x.push_back(t);
        est_yaw.y.push_back(r.yaw);
        est_pitch.x.push_back(t);
        est_pitch.y.push_back(r.pitch);
        if (std::isfinite(r.distance)) {
          est_dist.x.push_back(t);
          est_dist.y.push_back(r.distance);
        }
      }
    }
    std::string p = path("yaw_timeseries.png");
    renderLinePlot(p, ev.label + ": yaw vs ground truth", xlabel, "yaw [deg]",
                   {gt_yaw, est_yaw, raw_yaw});
    plots.push_back({p, "yaw GT vs filtered vs raw"});
    p = path("pitch_timeseries.png");
    renderLinePlot(p, ev.label + ": pitch vs ground truth", xlabel, "pitch [deg]",
                   {gt_pitch, est_pitch, raw_pitch});
    plots.push_back({p, "pitch GT vs filtered vs raw"});
    p = path("yaw_pitch_error_timeseries.png");
    renderLinePlot(p, ev.label + ": angle error vs ground truth", xlabel,
                   "error [deg]",
                   {err_yaw, err_pitch, err_yaw_raw, err_pitch_raw});
    plots.push_back({p, "yaw/pitch error vs time"});
    p = path("distance_timeseries.png");
    renderLinePlot(p, ev.label + ": distance vs ground truth", xlabel,
                   "distance [m]", {gt_dist, est_dist});
    plots.push_back({p, "distance GT vs estimate"});
    p = path("error_hist_yaw.png");
    renderHistogram(p, ev.label + ": yaw error histogram", "yaw error [deg]",
                    yaw_errors, 25, kColorErrYaw);
    plots.push_back({p, "yaw error histogram"});
    p = path("error_hist_pitch.png");
    renderHistogram(p, ev.label + ": pitch error histogram", "pitch error [deg]",
                    pitch_errors, 25, kColorErrPitch);
    plots.push_back({p, "pitch error histogram"});
  } else {
    LineSeries est_yaw, est_pitch, raw_yaw, raw_pitch;
    est_yaw.label = "filtered yaw";
    est_yaw.color = kColorEst;
    est_pitch.label = "filtered pitch";
    est_pitch.color = kColorEst;
    raw_yaw.label = "raw PnP yaw";
    raw_yaw.color = kColorRaw;
    raw_yaw.thickness = 1;
    raw_pitch.label = "raw PnP pitch";
    raw_pitch.color = kColorRaw;
    raw_pitch.thickness = 1;
    for (size_t i = 0; i < track_rows.size(); ++i) {
      const TrackRow& r = track_rows[i];
      const double t = xs[i];
      if (r.valid) {
        est_yaw.x.push_back(t);
        est_yaw.y.push_back(r.yaw);
        est_pitch.x.push_back(t);
        est_pitch.y.push_back(r.pitch);
      }
      if (rawPoseValid(r)) {
        raw_yaw.x.push_back(t);
        raw_yaw.y.push_back(r.raw_yaw);
        raw_pitch.x.push_back(t);
        raw_pitch.y.push_back(r.raw_pitch);
      }
    }
    std::string p = path("yaw_timeseries.png");
    renderLinePlot(p, ev.label + ": yaw", xlabel, "yaw [deg]",
                   {est_yaw, raw_yaw});
    plots.push_back({p, "filtered yaw + raw PnP"});
    p = path("pitch_timeseries.png");
    renderLinePlot(p, ev.label + ": pitch", xlabel, "pitch [deg]",
                   {est_pitch, raw_pitch});
    plots.push_back({p, "filtered pitch + raw PnP"});
  }

  // Jitter bar charts (both modes).
  {
    std::vector<BarItem> second_diff = {
        {"yaw 2nd-diff filtered", ev.jitter_yaw.second_diff_std, kColorEst},
        {"yaw 2nd-diff raw", ev.jitter_raw_yaw.second_diff_std, kColorRaw},
        {"pitch 2nd-diff filtered", ev.jitter_pitch.second_diff_std, kColorEst},
        {"pitch 2nd-diff raw", ev.jitter_raw_pitch.second_diff_std, kColorRaw},
    };
    std::string p = path("jitter_second_diff.png");
    renderBarPlot(p, ev.label + ": jitter (std of 2nd difference)", "deg",
                  second_diff);
    plots.push_back({p, "jitter: 2nd-difference std, filtered vs raw"});
    std::vector<BarItem> first_diff = {
        {"yaw 1st-diff filtered", ev.jitter_yaw.first_diff_mean_abs, kColorEst},
        {"yaw 1st-diff raw", ev.jitter_raw_yaw.first_diff_mean_abs, kColorRaw},
        {"pitch 1st-diff filtered", ev.jitter_pitch.first_diff_mean_abs, kColorEst},
        {"pitch 1st-diff raw", ev.jitter_raw_pitch.first_diff_mean_abs, kColorRaw},
    };
    p = path("jitter_first_diff.png");
    renderBarPlot(p, ev.label + ": jitter (mean |1st difference|)", "deg",
                  first_diff);
    plots.push_back({p, "jitter: mean absolute 1st difference, filtered vs raw"});
  }

  // Reprojection RMS vs frame (both modes).
  {
    LineSeries reproj;
    reproj.label = "reproj RMSE";
    reproj.color = kColorAux;
    for (size_t i = 0; i < track_rows.size(); ++i) {
      if (rawPoseValid(track_rows[i]) && std::isfinite(track_rows[i].reproj)) {
        reproj.x.push_back(xs[i]);
        reproj.y.push_back(track_rows[i].reproj);
      }
    }
    std::string p = path("reproj_rmse_timeseries.png");
    renderLinePlot(p, ev.label + ": PnP reprojection RMS", xlabel,
                   "RMSE [px]", {reproj});
    plots.push_back({p, "reprojection RMS vs frame"});
  }

  // LOS cross-check plots (needs intrinsics).
  if (ev.config_loaded) {
    LineSeries raw_yaw, bearing_yaw, raw_pitch, bearing_pitch;
    raw_yaw.label = "raw PnP yaw";
    raw_yaw.color = kColorRaw;
    raw_yaw.thickness = 1;
    bearing_yaw.label = "ellipse bearing yaw";
    bearing_yaw.color = kColorGt;
    bearing_yaw.thickness = 2;
    raw_pitch.label = "raw PnP pitch";
    raw_pitch.color = kColorRaw;
    raw_pitch.thickness = 1;
    bearing_pitch.label = "ellipse bearing pitch";
    bearing_pitch.color = kColorGt;
    bearing_pitch.thickness = 2;
    for (size_t i = 0; i < track_rows.size(); ++i) {
      const TrackRow& r = track_rows[i];
      if (!r.detected || !std::isfinite(r.raw_yaw) || !std::isfinite(r.raw_pitch)) {
        continue;
      }
      if (!(r.semi_major > 0.0) || !std::isfinite(r.ellipse_cx)) continue;
      if (!rawPoseValid(r)) continue;
      const double nx = (r.ellipse_cx - ev.config.camera.cx) / ev.config.camera.fx;
      const double ny = (r.ellipse_cy - ev.config.camera.cy) / ev.config.camera.fy;
      const double t = xs[i];
      raw_yaw.x.push_back(t);
      raw_yaw.y.push_back(r.raw_yaw);
      bearing_yaw.x.push_back(t);
      bearing_yaw.y.push_back(std::atan(nx) * kRad2Deg);
      raw_pitch.x.push_back(t);
      raw_pitch.y.push_back(r.raw_pitch);
      bearing_pitch.x.push_back(t);
      bearing_pitch.y.push_back(-std::atan(ny) * kRad2Deg);
    }
    std::string p = path("los_crosscheck.png");
    renderLinePlot(p, ev.label + ": PnP LOS vs ellipse-centre bearing", xlabel,
                   "angle [deg]", {bearing_yaw, raw_yaw, bearing_pitch, raw_pitch});
    plots.push_back({p, "raw PnP yaw/pitch vs ellipse-centre bearing"});
  }
  return plots;
}

std::string statePct(const Evaluation& ev, const std::string& state) {
  int count = 0;
  for (const auto& kv : ev.state_counts) {
    if (lower(kv.first) == lower(state)) count = kv.second;
  }
  return fmtFixed(percentage(count, ev.track_rows), 1) + " %";
}

struct Verdict {
  std::string name;
  double value = kNaN;
  double target = kNaN;
  bool le = true;  // true: value <= target, false: value >= target
  std::string unit;
};

std::vector<Verdict> buildVerdicts(const Evaluation& ev) {
  std::vector<Verdict> v;
  if (!ev.has_gt) return v;
  v.push_back({"yaw RMSE (measured)", ev.yaw.rmse, 1.0, true, "deg"});
  v.push_back({"pitch RMSE (measured)", ev.pitch.rmse, 1.0, true, "deg"});
  v.push_back({"raw PnP yaw RMSE (no filter)", ev.yaw_raw.rmse, 0.5, true,
               "deg"});
  v.push_back({"raw PnP pitch RMSE (no filter)", ev.pitch_raw.rmse, 0.5, true,
               "deg"});
  v.push_back({"distance relative RMSE", ev.distance_rel.rmse, 8.0, true, "%"});
  v.push_back({"valid rate on GT-visible frames", ev.valid_rate_visible, 90.0,
               false, "%"});
  // On noise-free synthetic data the raw PnP jitter is essentially zero, so the
  // filtered/raw jitter ratio is not a meaningful criterion there; report n/a.
  const bool jitter_meaningful =
      std::isfinite(ev.jitter_raw_yaw.second_diff_std) &&
      ev.jitter_raw_yaw.second_diff_std > 0.15 &&
      std::isfinite(ev.jitter_raw_pitch.second_diff_std) &&
      ev.jitter_raw_pitch.second_diff_std > 0.15;
  v.push_back({"jitter ratio yaw (filtered/raw)",
               jitter_meaningful ? ev.jitter_ratio_yaw : kNaN, 0.85, true,
               "-"});
  v.push_back({"jitter ratio pitch (filtered/raw)",
               jitter_meaningful ? ev.jitter_ratio_pitch : kNaN, 0.85, true,
               "-"});
  return v;
}

std::string verdictMark(const Verdict& v) {
  if (!std::isfinite(v.value)) return "n/a";
  const bool pass = v.le ? (v.value <= v.target) : (v.value >= v.target);
  return pass ? "PASS" : "FAIL";
}

void writeMetricsMarkdown(const Evaluation& ev,
                          const std::vector<MetricRow>& metrics,
                          const std::vector<PlotEntry>& plots,
                          const std::vector<Verdict>& verdicts,
                          const std::string& path) {
  std::ofstream out(path);
  out << "# Validation metrics: " << ev.label << "\n\n";
  out << "- Tracker CSV: `" << ev.track_path << "`\n";
  if (ev.has_gt) {
    out << "- Ground truth CSV: `" << ev.gt_path << "`\n";
  } else {
    out << "- Ground truth CSV: none (real-video / no-GT mode)\n";
  }
  if (!ev.config_path.empty()) {
    out << "- Config: `" << ev.config_path << "`";
    if (ev.config_loaded) {
      out << " (camera: " << ev.config.camera.width << "x"
          << ev.config.camera.height << ", fx=" << fmtFixed(ev.config.camera.fx, 1)
          << ", fy=" << fmtFixed(ev.config.camera.fy, 1)
          << ", cx=" << fmtFixed(ev.config.camera.cx, 1)
          << ", cy=" << fmtFixed(ev.config.camera.cy, 1) << ")";
    }
    out << "\n";
  }
  out << "- Tracker rows: " << ev.track_rows;
  if (ev.has_gt) {
    out << ", GT rows: " << ev.gt_rows << ", matched by frame: "
        << ev.matched_rows << " (GT unmatched: " << ev.gt_unmatched << ")";
  }
  out << "\n\n";

  auto table = [&](const std::string& title,
                   const std::vector<std::string>& names) {
    out << "## " << title << "\n\n";
    out << "| metric | value | unit | n |\n| --- | ---: | --- | ---: |\n";
    for (const auto& name : names) {
      for (const auto& row : metrics) {
        if (row.name != name) continue;
        out << "| `" << row.name << "` | " << valueOrNa(row.value, 5) << " | "
            << row.unit << " | " << row.count << " |\n";
        break;
      }
    }
    out << "\n";
  };

  if (ev.has_gt) {
    table("Accuracy vs synthetic ground truth (GT-visible, valid output)",
          {"yaw_rmse_measured_deg", "yaw_bias_measured_deg",
           "yaw_mae_measured_deg", "yaw_p95_abs_err_measured_deg",
           "yaw_max_abs_err_measured_deg", "pitch_rmse_measured_deg",
           "pitch_bias_measured_deg", "pitch_mae_measured_deg",
           "pitch_p95_abs_err_measured_deg", "pitch_max_abs_err_measured_deg",
           "yaw_rmse_all_deg", "pitch_rmse_all_deg",
           "yaw_rmse_predicted_deg", "pitch_rmse_predicted_deg",
           "yaw_rmse_raw_deg", "yaw_bias_raw_deg",
           "yaw_p95_abs_err_raw_deg", "pitch_rmse_raw_deg",
           "pitch_bias_raw_deg", "pitch_p95_abs_err_raw_deg",
           "accuracy_frames_raw"});
    table("Distance and disc-centre accuracy",
          {"distance_rmse_m", "distance_bias_m", "distance_mae_m",
           "distance_p95_abs_err_m", "distance_max_abs_err_m",
           "distance_rel_rmse_pct", "distance_rel_max_abs_pct",
           "disc_center_err_rmse_px", "disc_center_err_p95_px",
           "disc_center_err_max_px", "disc_radius_rel_rmse_pct"});
    table("Detection / valid coverage",
          {"gt_visible_frames", "gt_invisible_frames", "detection_rate_visible",
           "valid_rate_visible", "measured_rate_visible",
           "predicted_rate_visible", "false_detection_rate_invisible",
           "detection_rate_all", "valid_rate_all", "measured_rate_all"});
    table("Prediction through dropouts",
          {"dropout_visible_predicted_frames", "blind_prediction_frames",
           "dropout_pred_yaw_rmse_deg", "dropout_pred_pitch_rmse_deg",
           "blind_pred_yaw_rmse_deg", "blind_pred_pitch_rmse_deg",
           "blind_hold_yaw_rmse_deg", "blind_hold_pitch_rmse_deg"});
  } else {
    table("Detection / valid coverage",
          {"detection_rate_all", "valid_rate_all", "measured_rate_all"});
  }
  table("Filter smoothness (jitter) and rate statistics",
        {"jitter_filtered_yaw_2nd_std_deg", "jitter_raw_yaw_2nd_std_deg",
         "jitter_ratio_yaw", "jitter_filtered_pitch_2nd_std_deg",
         "jitter_raw_pitch_2nd_std_deg", "jitter_ratio_pitch",
         "jitter_filtered_yaw_1st_mad_deg", "jitter_raw_yaw_1st_mad_deg",
         "jitter_filtered_pitch_1st_mad_deg", "jitter_raw_pitch_1st_mad_deg",
         "max_abs_yaw_rate_dps", "max_abs_pitch_rate_dps",
         "p95_abs_yaw_rate_dps", "p95_abs_pitch_rate_dps"});
  table("Filter vs raw PnP deviation (tracked frames)",
        {"filter_raw_frames", "filter_raw_yaw_diff_mean_deg",
         "filter_raw_yaw_diff_rmse_deg", "filter_raw_yaw_diff_max_abs_deg",
         "filter_raw_pitch_diff_mean_deg", "filter_raw_pitch_diff_rmse_deg",
         "filter_raw_pitch_diff_max_abs_deg"});
  table("PnP quality",
        {"reproj_rmse_mean_px", "reproj_rmse_median_px", "reproj_rmse_p95_px",
         "reproj_rmse_max_px", "reproj_pass_rate_pct", "quality_mean",
         "quality_min"});
  if (ev.reproj.n > 0 && ev.reproj.max_abs == 0.0) {
    out << "> Note: `reproj_rmse_px` is 0.0 on every frame, i.e. the pose "
           "estimator does not populate `PoseEstimate::reproj_rmse_px` yet; "
           "treat the reprojection-RMS rows above as not available.\n\n";
  }
  table("Tracking continuity",
        {"detection_gap_count", "detection_gap_mean_frames",
         "detection_gap_max_frames", "prediction_run_count",
         "prediction_run_mean_frames", "prediction_run_max_frames"});
  if (ev.config_loaded) {
    table("PnP LOS vs ellipse-centre bearing cross-check",
          {"los_crosscheck_frames", "los_yaw_dev_bias_deg",
           "los_yaw_dev_std_deg", "los_yaw_dev_rmse_deg",
           "los_pitch_dev_bias_deg", "los_pitch_dev_std_deg",
           "los_pitch_dev_rmse_deg"});
  }

  if (!verdicts.empty()) {
    out << "## Verdict vs reference acceptance targets\n\n";
    out << "These thresholds are reference values for the synthetic closed-loop "
           "validation (they are informational, not fused into the pipeline).\n\n";
    out << "| criterion | value | target | result |\n| --- | ---: | ---: | --- |\n";
    for (const auto& v : verdicts) {
      out << "| " << v.name << " | " << fmtFixed(v.value, 4) << " " << v.unit
          << " | " << (v.le ? "<=" : ">=") << " " << fmtFixed(v.target, 2)
          << " " << v.unit << " | " << verdictMark(v) << " |\n";
    }
    out << "\n";
  }

  out << "## State distribution\n\n";
  out << "| state | frames | share |\n| --- | ---: | ---: |\n";
  for (const auto& kv : ev.state_counts) {
    out << "| " << kv.first << " | " << kv.second << " | "
        << fmtFixed(percentage(kv.second, ev.track_rows), 1) << " % |\n";
  }
  out << "\n";
  out << "Key states: TRACK " << statePct(ev, "TRACK") << ", PREDICT "
      << statePct(ev, "PREDICT") << ", SEARCH " << statePct(ev, "SEARCH")
      << ", ACQUIRE " << statePct(ev, "ACQUIRE") << ", LOST "
      << statePct(ev, "LOST") << ".\n\n";

  if (!plots.empty()) {
    out << "## Plots\n\n";
    for (const auto& plot : plots) {
      out << "- `" << std::filesystem::path(plot.path).filename().string()
          << "` - " << plot.note << "\n";
    }
    out << "\n";
  }
  out << "## Metric glossary\n\n";
  out << "- `yaw_*_deg` / `pitch_*_deg`: signed error = estimate - ground truth; "
         "measured_* uses only frames where the detector fed the filter, "
         "predicted_* only coasting frames, all_* every valid output. "
         "`*_raw_*` uses the unfiltered PnP pose (raw_yaw_deg/raw_pitch_deg), "
         "i.e. pure detector + pose-estimator accuracy.\n";
  out << "- `jitter_*_2nd_std_deg`: standard deviation of the second discrete "
         "difference (lower = smoother); `jitter_ratio` = filtered / raw, "
         "values < 1 mean the Kalman filter reduces jitter.\n";
  out << "- `dropout_pred_*`: error while the synthetic target was not rendered "
         "(GT still known) and the tracker predicted; `blind_hold_*` is the "
         "constant-hold baseline (last measured value), so the prediction gain "
         "is visible.\n";
  out << "- `reproj_rmse_*`: PnP reprojection RMS in pixels; "
         "`reproj_pass_rate_pct` is the share of frames below the configured "
         "`pose.max_reproj_rmse_px`.\n";
  out << "- `los_*_dev_*`: raw PnP yaw/pitch minus the bearing derived from the "
         "fitted disc-ellipse centre (`atan((cx-cx0)/fx)`), degrees; the two are "
         "expected to agree up to the perspective ellipse-centre bias.\n";
  out << "- `filter_raw_*_diff_*`: filtered output minus raw PnP measurement on "
         "frames where both exist, i.e. the filter's smoothing lag / prediction "
         "bias vs its own measurement (degrees).\n";
}

void printSummary(const Evaluation& ev, const std::vector<Verdict>& verdicts,
                  const std::string& out_dir) {
  std::printf("\n===== glowtrack_eval: %s =====\n", ev.label.c_str());
  std::printf("tracker rows      : %d", ev.track_rows);
  if (ev.has_gt) {
    std::printf("  (gt rows %d, matched %d)", ev.gt_rows, ev.matched_rows);
  }
  std::printf("\n");
  std::printf("detection rate    : %.1f %% (all)\n", ev.detection_rate_all);
  std::printf("valid rate        : %.1f %% (all)\n", ev.valid_rate_all);
  std::printf("measured rate     : %.1f %% (all)\n", ev.measured_rate_all);
  if (ev.has_gt) {
    std::printf("coverage (visible): det %.1f %% | valid %.1f %% | measured %.1f %%\n",
                ev.detection_rate_visible, ev.valid_rate_visible,
                ev.measured_rate_visible);
    std::printf("yaw   error       : RMSE %.3f deg | bias %+.3f | max %.3f | p95 "
                "%.3f  (n=%d)\n",
                ev.yaw.rmse, ev.yaw.mean, ev.yaw.max_abs, ev.yaw.p95_abs,
                ev.yaw.n);
    std::printf("pitch error       : RMSE %.3f deg | bias %+.3f | max %.3f | p95 "
                "%.3f  (n=%d)\n",
                ev.pitch.rmse, ev.pitch.mean, ev.pitch.max_abs, ev.pitch.p95_abs,
                ev.pitch.n);
    std::printf("raw PnP error     : yaw RMSE %.3f deg | pitch RMSE %.3f deg  "
                "(detector+pose only, n=%d)\n",
                ev.yaw_raw.rmse, ev.pitch_raw.rmse, ev.yaw_raw.n);
    std::printf("distance error    : RMSE %.4f m (%.2f %%) | bias %+.4f m  "
                "(n=%d)\n",
                ev.distance.rmse, ev.distance_rel.rmse, ev.distance.mean,
                ev.distance.n);
    std::printf("disc centre error : RMSE %.2f px (n=%d)\n",
                ev.center_err_px.rmse, ev.center_err_px.n);
    std::printf("dropout prediction: yaw RMSE %.3f deg | pitch RMSE %.3f deg  "
                "(n=%d); hold baseline %.3f / %.3f deg\n",
                ev.dropout_yaw.rmse, ev.dropout_pitch.rmse, ev.dropout_yaw.n,
                ev.blind_hold_yaw.rmse, ev.blind_hold_pitch.rmse);
    if (ev.blind_prediction_frames > 0) {
      std::printf("blind prediction  : yaw RMSE %.3f deg | pitch RMSE %.3f deg  "
                  "(n=%d)\n",
                  ev.blind_yaw.rmse, ev.blind_pitch.rmse,
                  ev.blind_prediction_frames);
    }
  }
  std::printf("jitter 2nd-diff   : yaw %.4f deg (raw %.4f, ratio %.2f) | pitch "
              "%.4f (raw %.4f, ratio %.2f)\n",
              ev.jitter_yaw.second_diff_std, ev.jitter_raw_yaw.second_diff_std,
              ev.jitter_ratio_yaw, ev.jitter_pitch.second_diff_std,
              ev.jitter_raw_pitch.second_diff_std, ev.jitter_ratio_pitch);
  if (ev.reproj.n > 0 && ev.reproj.max_abs == 0.0) {
    std::printf("reproj RMS        : not available (reproj_rmse_px column is "
                "all zero, pose_estimator has not populated it yet)\n");
  } else {
    std::printf(
        "reproj RMS        : mean %.2f px | p95 %.2f | pass %.1f %% (n=%d)\n",
        ev.reproj.mean, ev.reproj.p95_abs, ev.reproj_pass_rate, ev.reproj.n);
  }
  if (ev.filter_raw_frames > 0) {
    std::printf(
        "filter vs raw PnP : yaw diff %.3f +/- %.3f deg (max %.2f) | pitch "
        "diff %.3f +/- %.3f deg (n=%d)\n",
        ev.filter_raw_yaw.mean, ev.filter_raw_yaw.stddev,
        ev.filter_raw_yaw.max_abs, ev.filter_raw_pitch.mean,
        ev.filter_raw_pitch.stddev, ev.filter_raw_frames);
  }
  if (ev.config_loaded) {
    std::printf("LOS cross-check   : yaw dev %.3f +/- %.3f deg | pitch dev "
                "%.3f +/- %.3f deg (n=%d)\n",
                ev.los_yaw_dev.mean, ev.los_yaw_dev.stddev,
                ev.los_pitch_dev.mean, ev.los_pitch_dev.stddev, ev.los_frames);
  }
  if (!verdicts.empty()) {
    std::printf("reference targets : ");
    for (size_t i = 0; i < verdicts.size(); ++i) {
      if (i > 0) std::printf(", ");
      std::printf("%s=%s", verdicts[i].name.c_str(), verdictMark(verdicts[i]).c_str());
    }
    std::printf("\n");
  }
  std::printf("outputs           : %s\n", out_dir.c_str());
}

}  // namespace

int main(int argc, char** argv) {
  Args args;
  if (!parseArgs(argc, argv, &args)) {
    return 1;
  }
  try {
    const Evaluation ev = evaluate(args);
    const std::vector<MetricRow> metrics = buildMetrics(ev);
    const std::vector<Verdict> verdicts = buildVerdicts(ev);

    std::error_code ec;
    std::filesystem::create_directories(args.out, ec);
    if (ec) {
      std::fprintf(stderr, "error: cannot create output directory %s: %s\n",
                   args.out.c_str(), ec.message().c_str());
      return 1;
    }
    const std::string metrics_csv =
        (std::filesystem::path(args.out) / "metrics.csv").string();
    const std::string metrics_md =
        (std::filesystem::path(args.out) / "metrics.md").string();
    writeMetricsCsv(ev, metrics, metrics_csv);

    std::vector<PlotEntry> plots;
    std::vector<TrackRow> track_rows_copy;
    std::vector<GtRow> gt_rows_copy;
    std::unordered_map<int, size_t> gt_by_frame;
    bool matched_by_frame = false;
    if (!args.no_plots) {
      // Re-read the inputs for the plotting pass (keeps evaluate() one-pass
      // simple and the memory cost is trivial for validation runs).
      std::string warning;
      Table track_table;
      Table gt_table;
      if (!loadTable(args.track, &track_table, &warning)) return 1;
      if (!parseTrackTable(track_table, &track_rows_copy, &warning)) return 1;
      if (!args.gt.empty()) {
        if (!loadTable(args.gt, &gt_table, &warning)) return 1;
        if (!parseGtTable(gt_table, &gt_rows_copy, &warning)) return 1;
        for (size_t i = 0; i < gt_rows_copy.size(); ++i) {
          gt_by_frame[gt_rows_copy[i].frame] = i;
        }
        for (const auto& r : track_rows_copy) {
          if (gt_by_frame.count(r.frame)) {
            matched_by_frame = true;
            break;
          }
        }
      }
      plots = renderPlots(ev, track_rows_copy, gt_rows_copy, gt_by_frame,
                          matched_by_frame, args.out);
    }

    if (ev.has_gt) {
      // matched_series.csv needs the same inputs.
      if (track_rows_copy.empty()) {
        std::string warning;
        Table track_table;
        Table gt_table;
        loadTable(args.track, &track_table, &warning);
        parseTrackTable(track_table, &track_rows_copy, &warning);
        if (!args.gt.empty()) {
          loadTable(args.gt, &gt_table, &warning);
          parseGtTable(gt_table, &gt_rows_copy, &warning);
          for (size_t i = 0; i < gt_rows_copy.size(); ++i) {
            gt_by_frame[gt_rows_copy[i].frame] = i;
          }
        }
      }
      const std::string series_csv =
          (std::filesystem::path(args.out) / "matched_series.csv").string();
      writeMatchedSeries(ev, track_rows_copy, gt_rows_copy, gt_by_frame,
                         matched_by_frame || !gt_by_frame.empty(), series_csv);
    }

    writeMetricsMarkdown(ev, metrics, plots, verdicts, metrics_md);
    printSummary(ev, verdicts, args.out);
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
