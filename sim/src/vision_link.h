#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "bus.h"
#include "messages.h"

// Sends camera frames to the Python classifier and publishes its replies. If the classifier is
// busy, older frames are dropped so it always works on the newest one.
class VisionLink {
public:
    VisionLink(MessageBus& bus, std::string host, int port);
    ~VisionLink();

    bool connected() const { return connected_; }
    double lastRoundTripMs() const { return roundTripMs_; }

private:
    void run();
    bool exchange(int socket, const CameraFrame& frame);

    MessageBus& bus_;
    std::string host_;
    int port_;
    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> connected_{false};
    std::atomic<double> roundTripMs_{0};
    std::mutex mutex_;
    std::condition_variable wake_;
    std::optional<CameraFrame> latest_;
};
