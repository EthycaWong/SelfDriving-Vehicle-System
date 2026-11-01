#pragma once

#include <any>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <vector>

struct TopicStats {
    long count = 0;
    double lastTime = -1;
    double rate = 0;
    std::string summary;
};

struct BusLogEntry {
    double time = 0;
    std::string topic;
    std::string summary;
};

// Typed publish/subscribe used by every module. Any thread may publish. Handlers only run inside
// dispatch(), on the thread that calls it, so modules need no locks of their own.
class MessageBus {
public:
    void setTime(double now) {
        std::lock_guard lock(mutex_);
        now_ = now;
    }

    double time() {
        std::lock_guard lock(mutex_);
        return now_;
    }

    template <typename T>
    void subscribe(const std::string& topic, std::function<void(const T&)> handler) {
        std::lock_guard lock(mutex_);
        handlers_[topic].push_back([handler = std::move(handler)](const std::any& value) {
            handler(std::any_cast<const T&>(value));
        });
    }

    // A non-empty summary also goes into the log shown on the dashboard.
    template <typename T>
    void publish(const std::string& topic, T value, std::string summary = {}) {
        std::lock_guard lock(mutex_);
        queue_.push_back({topic, std::any(std::move(value))});
        TopicStats& stats = stats_[topic];
        ++stats.count;
        ++windowCount_[topic];
        stats.lastTime = now_;
        if (!summary.empty()) {
            stats.summary = summary;
            log_.push_back({now_, topic, std::move(summary)});
            if (log_.size() > kLogSize) log_.pop_front();
        }
    }

    void dispatch() {
        // Handlers can publish. Those messages run in a later round, with a limit so a feedback
        // loop can't stall the frame.
        for (int round = 0; round < 8; ++round) {
            std::deque<Queued> batch;
            {
                std::lock_guard lock(mutex_);
                batch.swap(queue_);
                updateRates();
            }
            if (batch.empty()) return;
            for (const Queued& item : batch) {
                std::vector<std::function<void(const std::any&)>> handlers;
                {
                    std::lock_guard lock(mutex_);
                    const auto found = handlers_.find(item.topic);
                    if (found != handlers_.end()) handlers = found->second;
                }
                for (const auto& handler : handlers) handler(item.value);
            }
        }
    }

    std::map<std::string, TopicStats> stats() {
        std::lock_guard lock(mutex_);
        return stats_;
    }

    std::vector<BusLogEntry> recentLog() {
        std::lock_guard lock(mutex_);
        return {log_.begin(), log_.end()};
    }

private:
    static constexpr size_t kLogSize = 200;

    struct Queued {
        std::string topic;
        std::any value;
    };

    void updateRates() {
        const double elapsed = now_ - windowStart_;
        if (elapsed < 1.0) return;
        for (auto& [topic, stats] : stats_) {
            stats.rate = static_cast<double>(windowCount_[topic]) / elapsed;
            windowCount_[topic] = 0;
        }
        windowStart_ = now_;
    }

    std::mutex mutex_;
    std::deque<Queued> queue_;
    std::map<std::string, std::vector<std::function<void(const std::any&)>>> handlers_;
    std::map<std::string, TopicStats> stats_;
    std::map<std::string, long> windowCount_;
    std::deque<BusLogEntry> log_;
    double now_ = 0;
    double windowStart_ = 0;
};
