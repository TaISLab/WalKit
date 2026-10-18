#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "walker_stamp_utils/stamp_sequencer.h"

namespace {

struct Out {
    int stream;
    int64_t stamp;
    std::string tag;
};

struct Fixture {
    std::vector<Out> out;
    StampSequencer<std::string> seq;
    Fixture(int streams, int64_t max_wait, std::size_t max_buffer = 1000)
        : seq(streams, max_wait, max_buffer,
              [this](int s, int64_t t, std::string &p) { out.push_back({s, t, p}); }) {}
};

}  // namespace

TEST(StampSequencer, ReleasesInStampOrderWhateverTheArrivalOrder) {
    // same messages, two different arrival orders -> same output
    std::vector<Out> first;
    for (int variant = 0; variant < 2; ++variant) {
        Fixture f(2, 0);
        if (variant == 0) {
            f.seq.push(0, 10, "a10"); f.seq.push(0, 30, "a30"); f.seq.push(1, 20, "b20"); f.seq.push(1, 40, "b40");
        } else {
            f.seq.push(1, 20, "b20"); f.seq.push(1, 40, "b40"); f.seq.push(0, 10, "a10"); f.seq.push(0, 30, "a30");
        }
        f.seq.release();
        // 40 and 30 stay: the watermark is min(last) = 30 -> a30 is released, b40 is not
        ASSERT_EQ(f.out.size(), 3u);
        EXPECT_EQ(f.out[0].tag, "a10");
        EXPECT_EQ(f.out[1].tag, "b20");
        EXPECT_EQ(f.out[2].tag, "a30");
        EXPECT_EQ(f.seq.pending(), 1u);
        if (variant == 0) first = f.out;
        else for (std::size_t i = 0; i < 3; ++i) EXPECT_EQ(first[i].tag, f.out[i].tag);
    }
}

TEST(StampSequencer, TiesGoByStreamThenArrival) {
    Fixture f(3, 0);
    f.seq.push(2, 5, "c");
    f.seq.push(1, 5, "b1");
    f.seq.push(1, 5, "b2");
    f.seq.push(0, 5, "a");
    f.seq.release();
    ASSERT_EQ(f.out.size(), 4u);
    EXPECT_EQ(f.out[0].tag, "a");
    EXPECT_EQ(f.out[1].tag, "b1");
    EXPECT_EQ(f.out[2].tag, "b2");
    EXPECT_EQ(f.out[3].tag, "c");
}

TEST(StampSequencer, NothingIsReleasedUntilEveryStreamHasSpoken) {
    Fixture f(2, 0);
    f.seq.push(0, 10, "a");
    f.seq.release();
    EXPECT_TRUE(f.out.empty());
    f.seq.push(1, 10, "b");
    f.seq.release();
    EXPECT_EQ(f.out.size(), 2u);
}

TEST(StampSequencer, SilentStreamIsGivenUpOnAfterMaxWait) {
    Fixture f(2, 100);
    f.seq.push(1, 0, "b0");
    f.seq.push(0, 50, "a50");
    f.seq.push(0, 90, "a90");
    f.seq.release();
    // watermark = min(last stamps) = 0: only b0
    ASSERT_EQ(f.out.size(), 1u);
    EXPECT_EQ(f.out[0].tag, "b0");
    f.seq.push(0, 150, "a150");
    f.seq.release();
    // stream 1 is silent: the watermark is forced to 150 - 100 = 50
    ASSERT_EQ(f.out.size(), 2u);
    EXPECT_EQ(f.out[1].tag, "a50");
    // the silent stream comes back with something older than what was released: dropped
    EXPECT_FALSE(f.seq.push(1, 20, "late"));
    EXPECT_EQ(f.seq.late(), 1u);
    // ... newer than that is fine
    EXPECT_TRUE(f.seq.push(1, 60, "ok"));
}

TEST(StampSequencer, OverflowDropsTheOldest) {
    Fixture f(2, 0, 3);
    f.seq.push(0, 1, "a1"); f.seq.push(0, 2, "a2"); f.seq.push(0, 3, "a3"); f.seq.push(0, 4, "a4");
    EXPECT_EQ(f.seq.pending(), 3u);
    EXPECT_EQ(f.seq.overflow(), 1u);
}

TEST(StampSequencer, RepeatingStreamStampIsCompleteOnlyOnceTheStreamMovesOn) {
    for (int variant = 0; variant < 2; ++variant) {
        std::vector<Out> out;
        StampSequencer<std::string> seq(2, 0, 1000,
            [&out](int s, int64_t t, std::string &p) { out.push_back({s, t, p}); }, {true, false});
        if (variant == 0) {
            seq.push(0, 10, "a1"); seq.release();
            seq.push(1, 10, "b"); seq.release();
            seq.push(0, 10, "a2"); seq.release();
        } else {
            seq.push(0, 10, "a1"); seq.push(0, 10, "a2"); seq.release();
            seq.push(1, 10, "b"); seq.release();
        }
        EXPECT_TRUE(out.empty());          // stamp 10 of the repeating stream is not complete yet
        seq.push(0, 20, "a3"); seq.release();
        ASSERT_EQ(out.size(), 3u);
        EXPECT_EQ(out[0].tag, "a1");
        EXPECT_EQ(out[1].tag, "a2");
        EXPECT_EQ(out[2].tag, "b");
    }
}
