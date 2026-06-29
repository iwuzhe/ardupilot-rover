#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>
#include <AR_Trajectory/AR_Trajectory.h>
#include <cmath>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AR_TrajectoryPoint make_point(float time_s, float x_m, float yaw_rad)
{
    return {
        time_s,
        x_m,
        2.0f * x_m,
        yaw_rad,
        1.0f + x_m,
        0.1f * x_m,
    };
}

TEST(AR_Trajectory, rejects_invalid_upload)
{
    AR_Trajectory trajectory;
    AR_TrajectoryPoint point = make_point(0.0f, 0.0f, 0.0f);

    EXPECT_EQ(trajectory.set_point(1, 0, 1, point), AR_Trajectory::Result::INVALID_COUNT);
    EXPECT_EQ(trajectory.set_point(1, 2, 2, point), AR_Trajectory::Result::INVALID_INDEX);

    point.x_m = NAN;
    EXPECT_EQ(trajectory.set_point(1, 0, 2, point), AR_Trajectory::Result::INVALID_POINT);
}

TEST(AR_Trajectory, validates_complete_time_axis)
{
    AR_Trajectory trajectory;
    EXPECT_EQ(trajectory.set_point(7, 0, 2, make_point(1.0f, 0.0f, 0.0f)), AR_Trajectory::Result::OK);
    EXPECT_EQ(trajectory.finalise(7, 2), AR_Trajectory::Result::MISSING_POINT);
    EXPECT_EQ(trajectory.set_point(7, 1, 2, make_point(1.0f, 1.0f, 0.0f)), AR_Trajectory::Result::OK);
    EXPECT_EQ(trajectory.finalise(7, 2), AR_Trajectory::Result::INVALID_TIME_AXIS);
}

TEST(AR_Trajectory, accepts_idempotent_duplicate)
{
    AR_Trajectory trajectory;
    const AR_TrajectoryPoint point = make_point(0.0f, 0.0f, 0.0f);
    EXPECT_EQ(trajectory.set_point(2, 0, 2, point), AR_Trajectory::Result::OK);
    EXPECT_EQ(trajectory.set_point(2, 0, 2, point), AR_Trajectory::Result::OK);
    EXPECT_EQ(trajectory.received_count(), 1);
    EXPECT_EQ(trajectory.set_point(2, 0, 2, make_point(0.0f, 1.0f, 0.0f)),
              AR_Trajectory::Result::DUPLICATE_CONFLICT);
}

TEST(AR_Trajectory, interpolates_values_and_wraps_yaw)
{
    AR_Trajectory trajectory;
    EXPECT_EQ(trajectory.set_point(3, 0, 2, make_point(0.0f, 0.0f, radians(170.0f))), AR_Trajectory::Result::OK);
    EXPECT_EQ(trajectory.set_point(3, 1, 2, make_point(2.0f, 2.0f, radians(-170.0f))), AR_Trajectory::Result::OK);
    EXPECT_EQ(trajectory.finalise(3, 2), AR_Trajectory::Result::OK);
    EXPECT_EQ(trajectory.start(1000000ULL), AR_Trajectory::Result::OK);

    AR_TrajectoryReference reference {};
    EXPECT_EQ(trajectory.sample(2000000ULL, reference), AR_Trajectory::Result::OK);
    EXPECT_NEAR(reference.x_m, 1.0f, 1.0e-5f);
    EXPECT_NEAR(reference.y_m, 2.0f, 1.0e-5f);
    EXPECT_NEAR(fabsf(reference.yaw_rad), M_PI, 1.0e-5f);
    EXPECT_EQ(reference.segment_index, 0);
}

TEST(AR_Trajectory, finishes_at_last_point)
{
    AR_Trajectory trajectory;
    EXPECT_EQ(trajectory.set_point(4, 0, 2, make_point(0.0f, 0.0f, 0.0f)), AR_Trajectory::Result::OK);
    EXPECT_EQ(trajectory.set_point(4, 1, 2, make_point(1.0f, 1.0f, 0.0f)), AR_Trajectory::Result::OK);
    EXPECT_EQ(trajectory.finalise(4, 2), AR_Trajectory::Result::OK);
    EXPECT_EQ(trajectory.start(0), AR_Trajectory::Result::OK);

    AR_TrajectoryReference reference {};
    EXPECT_EQ(trajectory.sample(1000000ULL, reference), AR_Trajectory::Result::OK);
    EXPECT_TRUE(trajectory.finished());
    EXPECT_FLOAT_EQ(reference.x_m, 1.0f);
}

TEST(AR_Trajectory, finds_monotonic_spatial_progress)
{
    AR_Trajectory trajectory;
    EXPECT_EQ(trajectory.set_point(5, 0, 3, make_point(0.0f, 0.0f, 0.0f)), AR_Trajectory::Result::OK);
    EXPECT_EQ(trajectory.set_point(5, 1, 3, make_point(1.0f, 1.0f, 0.0f)), AR_Trajectory::Result::OK);
    EXPECT_EQ(trajectory.set_point(5, 2, 3, make_point(2.0f, 2.0f, 0.0f)), AR_Trajectory::Result::OK);
    EXPECT_EQ(trajectory.finalise(5, 3), AR_Trajectory::Result::OK);

    uint16_t closest_index = 0;
    float progress = 0.0f;
    ASSERT_TRUE(trajectory.closest_progress(1.1f, 2.2f, 0, closest_index, progress));
    EXPECT_EQ(closest_index, 1);
    EXPECT_FLOAT_EQ(progress, 0.5f);

    ASSERT_TRUE(trajectory.closest_progress(0.0f, 0.0f, closest_index, closest_index, progress));
    EXPECT_EQ(closest_index, 1);
    EXPECT_FLOAT_EQ(progress, 0.5f);
}

AP_GTEST_MAIN()
