#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "messages.h"

// The text format shared with the board firmware and with the Python classifier.
namespace protocol {

uint8_t checksum(const std::string& body);
// Adds the leading $ and the trailing *CS and newline.
std::string frameLine(const std::string& body);
// Returns the body of a valid "$body*CS" line. Damaged lines return nothing.
std::optional<std::string> unframeLine(const std::string& line);

std::string formatCommand(uint32_t seq, const ControlCommand& command);
std::string formatDetections(uint32_t seq, uint32_t classMask, int nearestClass, int nearestDecimetres);

struct BoardMessage {
    enum class Kind { Sensors, Ack } kind = Kind::Sensors;
    SensorReadings sensors{};
    uint32_t ackSeq = 0;
};
std::optional<BoardMessage> parseBoardLine(const std::string& line);

// Parses one classifier reply: id|detectorMs|segmenterMs|class,score,x0,y0,x1,y1,source;...
std::optional<DetectionFrame> parseDetectionLine(const std::string& line);

}  // namespace protocol

// RFC2217 runs serial data inside Telnet, so 0xFF starts a command and a literal 0xFF is doubled.
// This strips commands out of the received bytes and builds the replies Telnet requires.
class TelnetFilter {
public:
    struct Output {
        std::string data;
        std::string replies;
    };
    Output feed(const uint8_t* bytes, size_t count);
    static std::string escape(const std::string& data);

private:
    enum class State { Data, Command, Option, Sub, SubCommand };
    State state_ = State::Data;
    uint8_t verb_ = 0;
};
