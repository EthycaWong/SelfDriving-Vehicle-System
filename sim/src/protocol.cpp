#include "protocol.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sstream>

namespace protocol {

namespace {

std::vector<std::string> split(const std::string& text, char separator) {
    std::vector<std::string> parts;
    std::stringstream stream(text);
    std::string part;
    while (std::getline(stream, part, separator)) parts.push_back(part);
    return parts;
}

bool parseInt(const std::string& text, int& value) {
    char* end = nullptr;
    const long parsed = std::strtol(text.c_str(), &end, 10);
    if (text.empty() || *end != '\0') return false;
    value = static_cast<int>(parsed);
    return true;
}

bool parseFloat(const std::string& text, float& value) {
    char* end = nullptr;
    value = std::strtof(text.c_str(), &end);
    return !text.empty() && *end == '\0';
}

}  // namespace

uint8_t checksum(const std::string& body) {
    uint8_t sum = 0;
    for (const char c : body) sum ^= static_cast<uint8_t>(c);
    return sum;
}

std::string frameLine(const std::string& body) {
    char tail[8];
    std::snprintf(tail, sizeof(tail), "*%02X\n", checksum(body));
    return "$" + body + tail;
}

std::optional<std::string> unframeLine(const std::string& line) {
    std::string text = line;
    while (!text.empty() && (text.back() == '\r' || text.back() == '\n')) text.pop_back();
    if (text.size() < 4 || text[0] != '$') return std::nullopt;
    const size_t star = text.rfind('*');
    if (star == std::string::npos || star + 3 != text.size()) return std::nullopt;
    const std::string body = text.substr(1, star - 1);
    char* end = nullptr;
    const long expected = std::strtol(text.c_str() + star + 1, &end, 16);
    if (*end != '\0' || expected != checksum(body)) return std::nullopt;
    return body;
}

std::string formatCommand(uint32_t seq, const ControlCommand& command) {
    char body[64];
    const int steerDegrees = static_cast<int>(std::lround(command.steer * 180.0f / 3.14159265f));
    std::snprintf(body, sizeof(body), "CMD,%u,%d,%d,%d,%d,%d", seq, steerDegrees,
                  static_cast<int>(std::lround(command.throttle * 100)), static_cast<int>(std::lround(command.brake * 100)),
                  static_cast<int>(command.state), static_cast<int>(command.maneuver));
    return frameLine(body);
}

std::string formatDetections(uint32_t seq, uint32_t classMask, int nearestClass, int nearestDecimetres) {
    char body[48];
    std::snprintf(body, sizeof(body), "DET,%u,%u,%d,%d", seq, classMask, nearestClass, nearestDecimetres);
    return frameLine(body);
}

std::optional<BoardMessage> parseBoardLine(const std::string& line) {
    const auto body = unframeLine(line);
    if (!body) return std::nullopt;
    const std::vector<std::string> fields = split(*body, ',');
    if (fields.empty()) return std::nullopt;

    BoardMessage message;
    if (fields[0] == "SEN" && fields.size() == 6) {
        int flags = 0;
        if (!parseInt(fields[1], message.sensors.frontCm) || !parseInt(fields[2], message.sensors.rearCm) ||
            !parseInt(fields[3], message.sensors.leftCm) || !parseInt(fields[4], message.sensors.rightCm) ||
            !parseInt(fields[5], flags)) {
            return std::nullopt;
        }
        message.kind = BoardMessage::Kind::Sensors;
        message.sensors.watchdogTripped = (flags & 1) != 0;
        return message;
    }
    if (fields[0] == "ACK" && fields.size() == 2) {
        int seq = 0;
        if (!parseInt(fields[1], seq)) return std::nullopt;
        message.kind = BoardMessage::Kind::Ack;
        message.ackSeq = static_cast<uint32_t>(seq);
        return message;
    }
    return std::nullopt;
}

std::optional<DetectionFrame> parseDetectionLine(const std::string& line) {
    std::string text = line;
    while (!text.empty() && (text.back() == '\r' || text.back() == '\n')) text.pop_back();
    const std::vector<std::string> sections = split(text, '|');
    if (sections.size() < 3) return std::nullopt;

    DetectionFrame frame;
    int id = 0;
    if (!parseInt(sections[0], id) || !parseFloat(sections[1], frame.detectorMs) ||
        !parseFloat(sections[2], frame.segmenterMs)) {
        return std::nullopt;
    }
    frame.frameId = static_cast<uint32_t>(id);
    if (sections.size() < 4) return frame;
    for (const std::string& item : split(sections[3], ';')) {
        const std::vector<std::string> fields = split(item, ',');
        Detection detection;
        if (fields.size() != 7 || !parseFloat(fields[1], detection.score) || !parseFloat(fields[2], detection.x0) ||
            !parseFloat(fields[3], detection.y0) || !parseFloat(fields[4], detection.x1) ||
            !parseFloat(fields[5], detection.y1)) {
            return std::nullopt;
        }
        detection.cls = classFromName(fields[0]);
        detection.source = fields[6];
        frame.detections.push_back(std::move(detection));
    }
    return frame;
}

}  // namespace protocol

namespace {
constexpr uint8_t kIac = 255, kDont = 254, kDo = 253, kWont = 252, kWill = 251, kSb = 250, kSe = 240;
}

TelnetFilter::Output TelnetFilter::feed(const uint8_t* bytes, size_t count) {
    Output output;
    for (size_t i = 0; i < count; ++i) {
        const uint8_t byte = bytes[i];
        switch (state_) {
            case State::Data:
                if (byte == kIac) {
                    state_ = State::Command;
                } else {
                    output.data.push_back(static_cast<char>(byte));
                }
                break;
            case State::Command:
                if (byte == kIac) {
                    output.data.push_back(static_cast<char>(byte));
                    state_ = State::Data;
                } else if (byte == kSb) {
                    state_ = State::Sub;
                } else if (byte == kWill || byte == kWont || byte == kDo || byte == kDont) {
                    verb_ = byte;
                    state_ = State::Option;
                } else {
                    state_ = State::Data;
                }
                break;
            case State::Option: {
                // Agree to whatever the server offers, so it starts passing serial data.
                const uint8_t reply = verb_ == kWill ? kDo : verb_ == kDo ? kWill : verb_ == kWont ? kDont : kWont;
                output.replies.push_back(static_cast<char>(kIac));
                output.replies.push_back(static_cast<char>(reply));
                output.replies.push_back(static_cast<char>(byte));
                state_ = State::Data;
                break;
            }
            case State::Sub:
                if (byte == kIac) state_ = State::SubCommand;
                break;
            case State::SubCommand:
                state_ = byte == kSe ? State::Data : State::Sub;
                break;
        }
    }
    return output;
}

std::string TelnetFilter::escape(const std::string& data) {
    std::string escaped;
    escaped.reserve(data.size());
    for (const char c : data) {
        escaped.push_back(c);
        if (static_cast<uint8_t>(c) == kIac) escaped.push_back(c);
    }
    return escaped;
}
