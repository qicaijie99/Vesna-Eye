// TEMPORARY STUB -- owned by the pipeline/IO workstream.
#include "glowtrack/frame_source.hpp"

namespace glowtrack {

std::unique_ptr<FrameSource> FrameSource::create(const FrameSourceConfig& config,
                                                 std::string* error) {
  (void)config;
  if (error != nullptr) {
    *error = "FrameSource::create not implemented yet";
  }
  return nullptr;
}

}  // namespace glowtrack
