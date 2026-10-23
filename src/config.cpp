// TEMPORARY STUB -- owned by the pipeline/IO workstream.
// Replace with the real cv::FileStorage based YAML loader.
#include "glowtrack/config.hpp"

#include <opencv2/core.hpp>

#include <stdexcept>

namespace glowtrack {

RuntimeConfig loadRuntimeConfig(const std::string& path) {
  // TODO(pipeline): read every section of the YAML file into RuntimeConfig.
  // Stub: returns hard-coded defaults so the rest of the project builds.
  RuntimeConfig config;
  (void)path;
  return config;
}

}  // namespace glowtrack
