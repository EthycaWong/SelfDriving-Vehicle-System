#include "vision_link.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstring>

#include "protocol.h"

namespace {

bool sendAll(int socket, const void* data, size_t size) {
    const char* cursor = static_cast<const char*>(data);
    while (size > 0) {
        const ssize_t sent = send(socket, cursor, size, 0);
        if (sent <= 0) return false;
        cursor += sent;
        size -= static_cast<size_t>(sent);
    }
    return true;
}

}  // namespace

VisionLink::VisionLink(MessageBus& bus, std::string host, int port) : bus_(bus), host_(std::move(host)), port_(port) {
    bus_.subscribe<CameraFrame>(topic::kCameraFrame, [this](const CameraFrame& frame) {
        {
            std::lock_guard lock(mutex_);
            latest_ = frame;
        }
        wake_.notify_one();
    });
    thread_ = std::thread([this] { run(); });
}

VisionLink::~VisionLink() {
    stop_ = true;
    wake_.notify_all();
    thread_.join();
}

bool VisionLink::exchange(int socket, const CameraFrame& frame) {
    const auto start = std::chrono::steady_clock::now();
    const std::array<uint32_t, 3> header{frame.id, static_cast<uint32_t>(frame.width), static_cast<uint32_t>(frame.height)};
    if (!sendAll(socket, "FRAM", 4) || !sendAll(socket, header.data(), sizeof(header)) ||
        !sendAll(socket, frame.rgb->data(), frame.rgb->size())) {
        return false;
    }

    std::string line;
    char buffer[4096];
    while (line.find('\n') == std::string::npos) {
        const ssize_t received = recv(socket, buffer, sizeof(buffer), 0);
        if (received <= 0) return false;
        line.append(buffer, static_cast<size_t>(received));
    }
    std::optional<DetectionFrame> reply = protocol::parseDetectionLine(line.substr(0, line.find('\n')));
    if (!reply) return false;

    reply->roundTripMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    reply->time = bus_.time();
    roundTripMs_ = reply->roundTripMs;
    const std::string summary = std::to_string(reply->detections.size()) + " detections in " +
                                std::to_string(static_cast<int>(reply->roundTripMs)) + " ms";
    bus_.publish(topic::kDetections, std::move(*reply), summary);
    return true;
}

void VisionLink::run() {
    while (!stop_) {
        const int fd = socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(static_cast<uint16_t>(port_));
        inet_pton(AF_INET, host_.c_str(), &address.sin_addr);
        if (connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
            close(fd);
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            continue;
        }
        connected_ = true;
        while (!stop_) {
            std::optional<CameraFrame> frame;
            {
                std::unique_lock lock(mutex_);
                wake_.wait_for(lock, std::chrono::milliseconds(200), [&] { return latest_.has_value() || stop_; });
                frame.swap(latest_);
            }
            if (frame && !exchange(fd, *frame)) break;
        }
        connected_ = false;
        close(fd);
    }
}
