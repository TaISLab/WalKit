#ifndef WALKER_STAMP_UTILS_STAMP_SEQUENCER_H_
#define WALKER_STAMP_UTILS_STAMP_SEQUENCER_H_

#include <algorithm>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <tuple>
#include <utility>
#include <vector>

// Hands the messages of several streams to a handler in data-time order
// (header stamp), not in the order they happened to be delivered.
//
// ROS delivers each topic in order, but gives no ordering between topics:
// which of two messages of different topics a node sees first depends on
// DDS, thread scheduling and, replaying a bag, on --rate. A node that reacts
// to messages as they arrive (or to a timer) therefore computes something
// different on every run. This class makes the result depend only on the
// messages' stamps: every pushed message is buffered and released, sorted by
// (stamp, stream index, arrival order within the stream), once no stream can
// still deliver anything older -- i.e. when the "watermark", the smallest of
// the streams' last stamps, has passed it (the usual event-time notion of
// stream processors). The price is latency: a message waits for the slowest
// stream, one of its periods at most.
//
// A stream whose stamps can repeat (several messages with the same stamp,
// e.g. a state republished as other inputs change) is declared in `repeating`:
// more messages with its last stamp may still come, so that stamp only counts
// as complete once the stream has moved on to a later one. Otherwise the order
// in which the repeated messages interleave with other streams' messages of
// the same stamp would depend on arrival order.
//
// A stream that stops delivering would hold everything back forever, so the
// watermark is never allowed to lag the newest stamp seen by more than
// max_wait_ns: after that the stalled stream is given up on, and whatever it
// delivers later, if older than what was already released, is dropped (and
// counted). With max_wait_ns <= 0 the wait is unbounded.
//
// Same semantics as walker_loads/stamp_sequencer.py (used by the python
// nodes).
template <typename Payload>
class StampSequencer {
  public:
    using Handler = std::function<void(int stream, int64_t stamp_ns, Payload &payload)>;

    StampSequencer(int n_streams, int64_t max_wait_ns, std::size_t max_buffer, Handler handler,
                   std::vector<bool> repeating = {})
        : last_(n_streams, none()), repeating_(std::move(repeating)), max_wait_ns_(max_wait_ns),
          max_buffer_(max_buffer), handler_(std::move(handler)) {
        repeating_.resize(n_streams, false);
    }

    // Buffers a message. Returns false if it was dropped as late (older than
    // what was already released). Nothing is handed to the handler until
    // release() is called.
    bool push(int stream, int64_t stamp_ns, Payload payload) {
        if (have_released_ && stamp_ns < released_ns_) {
            ++late_;
            return false;
        }
        last_[stream] = std::max(last_[stream], stamp_ns);
        max_seen_ = std::max(max_seen_, stamp_ns);
        buffer_.emplace(std::make_tuple(stamp_ns, stream, seq_++), std::move(payload));
        if (max_buffer_ > 0 && buffer_.size() > max_buffer_) {
            buffer_.erase(buffer_.begin());
            ++overflow_;
        }
        return true;
    }

    // Hands the handler everything the watermark has passed, oldest first.
    void release() {
        if (buffer_.empty()) {
            return;
        }
        int64_t wm = std::numeric_limits<int64_t>::max();
        for (std::size_t s = 0; s < last_.size(); ++s) {
            // none() - 1 would wrap around: a stream that has not spoken yet blocks, repeating or not
            int64_t complete = (repeating_[s] && last_[s] != none()) ? last_[s] - 1 : last_[s];
            wm = std::min(wm, complete);
        }
        if (max_wait_ns_ > 0) {
            wm = std::max(wm, max_seen_ - max_wait_ns_);
        }
        while (!buffer_.empty()) {
            auto it = buffer_.begin();
            const int64_t stamp_ns = std::get<0>(it->first);
            if (stamp_ns > wm) {
                break;
            }
            const int stream = std::get<1>(it->first);
            Payload payload = std::move(it->second);
            buffer_.erase(it);
            released_ns_ = stamp_ns;
            have_released_ = true;
            handler_(stream, stamp_ns, payload);
        }
    }

    std::size_t pending() const { return buffer_.size(); }
    uint64_t late() const { return late_; }
    uint64_t overflow() const { return overflow_; }

  private:
    static int64_t none() { return std::numeric_limits<int64_t>::min(); }

    std::map<std::tuple<int64_t, int, uint64_t>, Payload> buffer_;
    std::vector<int64_t> last_;
    std::vector<bool> repeating_;
    int64_t max_seen_ = std::numeric_limits<int64_t>::min();
    int64_t released_ns_ = std::numeric_limits<int64_t>::min();
    bool have_released_ = false;
    int64_t max_wait_ns_;
    std::size_t max_buffer_;
    uint64_t seq_ = 0;
    uint64_t late_ = 0;
    uint64_t overflow_ = 0;
    Handler handler_;
};

#endif  // WALKER_STAMP_UTILS_STAMP_SEQUENCER_H_
