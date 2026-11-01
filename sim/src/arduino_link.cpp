#include "arduino_link.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>

namespace {
constexpr auto kCommandPeriod = std::chrono::milliseconds(50);
constexpr size_t kMaxLine = 256;
}

ArduinoLink::ArduinoLink(MessageBus& bus, std::string host, int port) : bus_(bus), host_(std::move(host)), port_(port) {
    bus_.subscribe<ControlCommand>(topic::kControl, [this](const ControlCommand& command) {
        std::lock_guard lock(mutex_);
        command_ = command;
        hasCommand_ = true;
    });
    bus_.subscribe<VehicleState>(topic::kVehicle, [this](const VehicleState& state) {
        std::lock_guard lock(mutex_);
        vehicle_ = state;
    });
    bus_.subscribe<ObstacleList>(topic::kObstacles, [this](const ObstacleList& list) {
        std::lock_guard lock(mutex_);
        classMask_ = list.classMask;
        nearestClass_ = -1;
        float nearest = 1e9f;
        for (const Obstacle& obstacle : list.obstacles) {
            const float distance = length(obstacle.position - vehicle_.position);
            if (distance < nearest) {
                nearest = distance;
                nearestClass_ = static_cast<int>(obstacle.cls);
            }
        }
        nearestDm_ = nearestClass_ >= 0 ? static_cast<int>(nearest * 10) : 0;
    });
    thread_ = std::thread([this] { run(); });
}

ArduinoLink::~ArduinoLink() {
    stop_ = true;
    thread_.join();
}

LinkStatus ArduinoLink::status() {
    std::lock_guard lock(mutex_);
    return status_;
}

void ArduinoLink::handleLine(const std::string& line) {
    if (line.empty()) return;
    // Lines starting with # are the firmware's notes for the Wokwi serial monitor.
    if (line[0] == '#') {
        std::lock_guard lock(mutex_);
        status_.lastBoardLine = line;
        return;
    }
    const std::optional<protocol::BoardMessage> message = protocol::parseBoardLine(line);
    std::lock_guard lock(mutex_);
    if (!message) {
        if (line[0] == '$') ++status_.badLines;
        return;
    }
    status_.lastReceived = bus_.time();
    if (message->kind == protocol::BoardMessage::Kind::Sensors) {
        SensorReadings readings = message->sensors;
        readings.time = bus_.time();
        bus_.publish(topic::kSensors, readings);
    } else {
        const auto found = pendingAcks_.find(message->ackSeq);
        if (found != pendingAcks_.end()) {
            status_.ackRoundTripMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - found->second).count();
            pendingAcks_.erase(pendingAcks_.begin(), std::next(found));
        }
    }
}

void ArduinoLink::sendPending(int socket) {
    std::string out;
    {
        std::lock_guard lock(mutex_);
        if (hasCommand_) {
            const uint32_t seq = ++seq_;
            out += protocol::formatCommand(seq, command_);
            pendingAcks_[seq] = std::chrono::steady_clock::now();
            if (pendingAcks_.size() > 64) pendingAcks_.erase(pendingAcks_.begin());
        }
        if (classMask_ != sentMask_ || nearestClass_ != sentNearestClass_) {
            out += protocol::formatDetections(++seq_, classMask_, nearestClass_, nearestDm_);
            sentMask_ = classMask_;
            sentNearestClass_ = nearestClass_;
        }
    }
    if (!out.empty()) {
        const std::string escaped = TelnetFilter::escape(out);
        send(socket, escaped.data(), escaped.size(), 0);
    }
}

void ArduinoLink::run() {
    while (!stop_) {
        const int fd = socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(static_cast<uint16_t>(port_));
        inet_pton(AF_INET, host_.c_str(), &address.sin_addr);
        if (connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
            close(fd);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }
        {
            std::lock_guard lock(mutex_);
            status_.connected = true;
            sentMask_ = 0xFFFFFFFF;
        }
        bus_.publish(topic::kLink, true, "Board connected");

        TelnetFilter telnet;
        std::string line;
        auto nextSend = std::chrono::steady_clock::now();
        while (!stop_) {
            const auto now = std::chrono::steady_clock::now();
            if (now >= nextSend) {
                sendPending(fd);
                nextSend = now + kCommandPeriod;
            }
            fd_set readable;
            FD_ZERO(&readable);
            FD_SET(fd, &readable);
            timeval timeout{0, 10000};
            if (select(fd + 1, &readable, nullptr, nullptr, &timeout) <= 0) continue;

            uint8_t buffer[512];
            const ssize_t received = recv(fd, buffer, sizeof(buffer), 0);
            if (received <= 0) break;
            const TelnetFilter::Output output = telnet.feed(buffer, static_cast<size_t>(received));
            if (!output.replies.empty()) send(fd, output.replies.data(), output.replies.size(), 0);
            for (const char c : output.data) {
                if (c == '\n') {
                    while (!line.empty() && line.back() == '\r') line.pop_back();
                    handleLine(line);
                    line.clear();
                } else if (line.size() < kMaxLine) {
                    line.push_back(c);
                }
            }
        }
        close(fd);
        {
            std::lock_guard lock(mutex_);
            status_.connected = false;
        }
        bus_.publish(topic::kLink, false, "Board disconnected");
    }
}
