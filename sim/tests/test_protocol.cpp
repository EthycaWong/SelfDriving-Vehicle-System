#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include "bus.h"
#include "check.h"
#include "protocol.h"

namespace {

void testFraming() {
    const std::string line = protocol::frameLine("SEN,10,20,30,40,0");
    CHECK(line.back() == '\n');
    CHECK(protocol::unframeLine(line).value_or("") == "SEN,10,20,30,40,0");

    std::string damaged = line;
    damaged[5] = '9';
    CHECK(!protocol::unframeLine(damaged));
    CHECK(!protocol::unframeLine("SEN,1,2,3,4,0"));
    CHECK(!protocol::unframeLine("$SEN,1,2,3,4,0"));
    CHECK(!protocol::unframeLine("$SEN,1*ZZ"));
    CHECK(!protocol::unframeLine(""));
}

void testBoardLines() {
    auto sensors = protocol::parseBoardLine(protocol::frameLine("SEN,12,34,56,78,1"));
    CHECK(sensors && sensors->kind == protocol::BoardMessage::Kind::Sensors);
    CHECK(sensors && sensors->sensors.frontCm == 12 && sensors->sensors.rearCm == 34);
    CHECK(sensors && sensors->sensors.leftCm == 56 && sensors->sensors.rightCm == 78);
    CHECK(sensors && sensors->sensors.watchdogTripped);

    auto ack = protocol::parseBoardLine(protocol::frameLine("ACK,77"));
    CHECK(ack && ack->kind == protocol::BoardMessage::Kind::Ack && ack->ackSeq == 77);

    CHECK(!protocol::parseBoardLine(protocol::frameLine("SEN,1,2,3")));
    CHECK(!protocol::parseBoardLine(protocol::frameLine("SEN,a,2,3,4,0")));
    CHECK(!protocol::parseBoardLine(protocol::frameLine("WAT,1")));
}

void testCommandFormat() {
    ControlCommand command;
    command.steer = 0.1745f;
    command.throttle = 0.5f;
    command.state = DriveState::Stop;
    command.maneuver = ManeuverType::Overtake;
    CHECK(protocol::unframeLine(protocol::formatCommand(9, command)).value_or("") == "CMD,9,10,50,0,2,2");
}

void testDetectionReply() {
    auto frame = protocol::parseDetectionLine("12|30.5|150.0|person,0.90,1.0,2.0,3.0,4.0,detector;tree,1.00,5,6,7,8,segmenter\n");
    CHECK(frame && frame->frameId == 12 && frame->detections.size() == 2);
    CHECK(frame && frame->detections[0].cls == ObjectClass::Person && frame->detections[0].x1 == 3.0f);
    CHECK(frame && frame->detections[1].cls == ObjectClass::Tree && frame->detections[1].source == "segmenter");
    auto empty = protocol::parseDetectionLine("13|1|2|\n");
    CHECK(empty && empty->detections.empty());
    CHECK(!protocol::parseDetectionLine("garbage"));
    CHECK(!protocol::parseDetectionLine("1|2|3|person,0.9,1,2\n"));
}

void testTelnet() {
    TelnetFilter filter;
    // Negotiation, a subcommand, data with an escaped 0xFF, and a command split across two reads.
    const std::vector<uint8_t> first = {255, 251, 44, 'H', 'i', 255, 250, 44, 1, 2, 255, 240, 255, 255, '!', 255};
    const auto a = filter.feed(first.data(), first.size());
    CHECK(a.data == std::string("Hi\xFF!"));
    CHECK(a.replies == std::string("\xFF\xFD\x2C"));
    const std::vector<uint8_t> second = {253, 3, 'Z'};
    const auto b = filter.feed(second.data(), second.size());
    CHECK(b.data == "Z");
    CHECK(b.replies == std::string("\xFF\xFB\x03"));
    CHECK(TelnetFilter::escape(std::string("a\xFF")) == std::string("a\xFF\xFF"));
}

void testBusAcrossThreads() {
    MessageBus bus;
    int received = 0;
    long total = 0;
    bus.subscribe<int>("numbers", [&](const int& value) {
        ++received;
        total += value;
    });
    std::vector<std::thread> publishers;
    for (int t = 0; t < 4; ++t) {
        publishers.emplace_back([&bus] {
            for (int i = 0; i < 1000; ++i) bus.publish("numbers", 1);
        });
    }
    std::atomic<bool> done{false};
    std::thread dispatcher([&] {
        while (!done) bus.dispatch();
        bus.dispatch();
    });
    for (auto& publisher : publishers) publisher.join();
    done = true;
    dispatcher.join();
    CHECK(received == 4000);
    CHECK(total == 4000);
    CHECK(bus.stats()["numbers"].count == 4000);
}

}  // namespace

int main() {
    testFraming();
    testBoardLines();
    testCommandFormat();
    testDetectionReply();
    testTelnet();
    testBusAcrossThreads();
    std::printf("protocol tests: %d failures\n", failureCount());
    return failureCount() == 0 ? 0 : 1;
}
