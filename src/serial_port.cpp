// TEMPORARY STUB -- owned by the pipeline/IO workstream.
#include "glowtrack/output_sink.hpp"

namespace glowtrack {

std::unique_ptr<SerialPort> SerialPort::open(const std::string& port, int baud,
                                             std::string* error) {
  (void)port;
  (void)baud;
  if (error != nullptr) {
    *error = "serial port support not implemented yet";
  }
  return nullptr;
}

bool sendTrackPacket(SerialPort& port, const TrackOutput& output, uint8_t seq,
                     std::string* error) {
  (void)port;
  (void)output;
  (void)seq;
  if (error != nullptr) {
    *error = "serial packet support not implemented yet";
  }
  return false;
}

}  // namespace glowtrack
