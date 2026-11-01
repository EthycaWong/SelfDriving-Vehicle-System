#pragma once

#include <atomic>
#include <map>
#include <mutex>
#include <string>
#include <thread>

#include "bus.h"
#include "messages.h"
#include "protocol.h"

// Talks to the Wokwi board through the simulator's RFC2217 serial port, and reconnects if it drops.
class ArduinoLink {
public:
    ArduinoLink(MessageBus& bus, std::string host, int port);
    ~ArduinoLink();

    LinkStatus status();

private:
    void run();
    void handleLine(const std::string& line);
    void sendPending(int socket);

    MessageBus& bus_;
    std::string host_;
    int port_;
    std::thread thread_;
    std::atomic<bool> stop_{false};

    std::mutex mutex_;
    ControlCommand command_{};
    bool hasCommand_ = false;
    uint32_t classMask_ = 0;
    int nearestClass_ = -1;
    int nearestDm_ = 0;
    uint32_t sentMask_ = 0xFFFFFFFF;
    int sentNearestClass_ = -2;
    LinkStatus status_{};
    uint32_t seq_ = 0;
    std::map<uint32_t, std::chrono::steady_clock::time_point> pendingAcks_;
    VehicleState vehicle_{};
};
