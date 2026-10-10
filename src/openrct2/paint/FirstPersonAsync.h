/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <utility>

namespace OpenRCT2::Paint
{
    // Shared by capture, publication and uploads. Budgets stop BETWEEN bounded
    // units of work; they never wait for a worker or force a cold cache build.
    class FirstPersonFrameBudget
    {
        using Clock = std::chrono::steady_clock;
        Clock::time_point _end;
        size_t _remaining;
    public:
        explicit FirstPersonFrameBudget(size_t units, std::chrono::microseconds time)
            : _end(Clock::now() + time), _remaining(units) {}
        bool available() const { return _remaining != 0 && Clock::now() < _end; }
        void beginPhase(std::chrono::microseconds time) { _end = Clock::now() + time; }
        bool take(size_t units = 1)
        {
            if (units > _remaining || !available()) return false;
            _remaining -= units;
            return true;
        }
    };

    // Publishing one shared asset can wake thousands of instances. Moving a
    // batch is constant-time; repaint each tile under its own frame budget
    // rather than hiding an unbounded invalidation loop inside one commit.
    class FirstPersonTileRepaintQueue
    {
        std::deque<std::unordered_set<uint64_t>> _batches;
        size_t _pending = 0;
    public:
        void enqueue(std::unordered_set<uint64_t> tiles)
        {
            if (tiles.empty()) return;
            _pending += tiles.size();
            _batches.push_back(std::move(tiles));
        }
        size_t pending() const { return _pending; }
        void clear() { _batches.clear(); _pending = 0; }
        template<typename Repaint>
        void drain(FirstPersonFrameBudget& budget, Repaint&& repaint)
        {
            while (!_batches.empty() && budget.take())
            {
                auto& batch = _batches.front();
                const auto tile = *batch.begin();
                batch.erase(batch.begin());
                --_pending;
                if (batch.empty()) _batches.pop_front();
                repaint(tile);
            }
        }
    };

    class FirstPersonWorkQueue
    {
    public:
        using Commit = std::function<void()>;
        using Work = std::function<Commit()>;
    private:
        struct Job { uint64_t epoch; Work work; Commit failed; };
        struct Result { uint64_t epoch; Commit commit; };
        std::mutex _mutex;
        std::condition_variable _wake;
        std::deque<Job> _pending;
        std::deque<Result> _completed;
        std::atomic<uint64_t> _epoch{1};
        bool _stop = false;
        bool _running = false;
        std::thread _worker;
        static constexpr size_t kCapacity = 8;

        void run()
        {
            for (;;)
            {
                Job job;
                {
                    std::unique_lock lock(_mutex);
                    _wake.wait(lock, [&] { return _stop || (!_pending.empty() && _completed.size() < 2); });
                    if (_stop) return;
                    job = std::move(_pending.front());
                    _pending.pop_front();
                    _running = true;
                }
                Commit commit;
                if (job.epoch == _epoch.load())
                {
                    try { commit = job.work(); }
                    catch (...) { commit = std::move(job.failed); }
                }
                {
                    std::lock_guard lock(_mutex);
                    _running = false;
                    if (commit && job.epoch == _epoch.load())
                        _completed.push_back({ job.epoch, std::move(commit) });
                }
            }
        }
    public:
        FirstPersonWorkQueue() : _worker([this] { run(); }) {}
        ~FirstPersonWorkQueue()
        {
            { std::lock_guard lock(_mutex); _stop = true; }
            _wake.notify_one();
            _worker.join(); // application shutdown only, never POV/cache reset
        }
        FirstPersonWorkQueue(const FirstPersonWorkQueue&) = delete;
        FirstPersonWorkQueue& operator=(const FirstPersonWorkQueue&) = delete;
        uint64_t epoch() const { return _epoch.load(); }
        bool hasCapacity()
        {
            std::unique_lock lock(_mutex, std::try_to_lock);
            return lock.owns_lock() && _pending.size() + _completed.size() + size_t(_running) < kCapacity;
        }
        bool submit(Work work, Commit failed = {})
        {
            std::unique_lock lock(_mutex, std::try_to_lock);
            if (!lock.owns_lock() || _pending.size() + _completed.size() + size_t(_running) >= kCapacity)
                return false;
            _pending.push_back({ epoch(), std::move(work), std::move(failed) });
            _wake.notify_one();
            return true;
        }
        void publish(FirstPersonFrameBudget& budget)
        {
            while (budget.available())
            {
                Result result;
                {
                    std::unique_lock lock(_mutex, std::try_to_lock);
                    if (!lock.owns_lock() || _completed.empty()) return;
                    result = std::move(_completed.front());
                    _completed.pop_front();
                }
                _wake.notify_one();
                budget.take();
                if (result.epoch == epoch()) result.commit();
            }
        }
        void cancel()
        {
            // In-flight jobs own every byte they read. Resetting a park does
            // not join them or let their eventual results mutate the new park.
            ++_epoch;
            std::deque<Job> pending;
            std::deque<Result> completed;
            {
                std::lock_guard lock(_mutex);
                pending.swap(_pending);
                completed.swap(_completed);
            }
            _wake.notify_one();
        }
    };
}
