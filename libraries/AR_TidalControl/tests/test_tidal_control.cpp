#include <AP_gtest.h>

#include <AR_TidalControl/AR_BacksteppingPPC.h>
#include <AR_TidalControl/AR_ResidualSMO.h>
#include <AR_TidalControl/AR_TidalControl.h>
#include <AR_TidalControl/AR_WheelCompensation.h>
#include <cmath>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AR_TidalState make_state()
{
    AR_TidalState state {};
    state.dt_s = 0.01f;
    state.position_valid = true;
    state.velocity_valid = true;
    return state;
}

static AR_TrajectoryReference make_reference()
{
    AR_TrajectoryReference reference {};
    return reference;
}

static AR_BacksteppingPPC make_unlimited_controller()
{
    AR_BacksteppingPPC::Config config;
    config.speed_accel_max_mps2 = 1000.0f;
    config.yaw_accel_max_radps2 = 1000.0f;
    return AR_BacksteppingPPC(config);
}

TEST(AR_BacksteppingPPC, rejects_invalid_state)
{
    AR_BacksteppingPPC controller;
    AR_TidalState state = make_state();
    AR_TrajectoryReference reference = make_reference();
    AR_TidalControlOutput output {};

    state.x_m = NAN;
    EXPECT_FALSE(controller.update(state, reference, 0.0f, false, output));
    EXPECT_FALSE(output.valid);
    EXPECT_TRUE(output.stop_required);

    state = make_state();
    state.dt_s = 0.2f;
    EXPECT_FALSE(controller.update(state, reference, 0.0f, false, output));
}

TEST(AR_BacksteppingPPC, matches_backstepping_equations)
{
    AR_BacksteppingPPC controller = make_unlimited_controller();
    AR_TidalState state = make_state();
    AR_TrajectoryReference reference = make_reference();
    AR_TidalControlOutput output {};

    state.speed_mps = 0.5f;
    reference.x_m = 0.1f;
    reference.y_m = 0.2f;
    reference.yaw_rad = 0.1f;
    reference.speed_mps = 1.0f;
    reference.yaw_rate_radps = 0.2f;

    ASSERT_TRUE(controller.update(state, reference, 0.0f, false, output));
    EXPECT_NEAR(output.speed_cmd_mps, 1.5155042f, 1.0e-5f);
    EXPECT_NEAR(output.yaw_rate_cmd_radps, 0.5501157f, 1.0e-5f);

    const AR_BacksteppingPPCDebug &debug = controller.debug();
    EXPECT_NEAR(debug.error_x_m, 0.1f, 1.0e-6f);
    EXPECT_NEAR(debug.error_y_m, 0.2f, 1.0e-6f);
    EXPECT_NEAR(debug.error_y_control_m, 0.1890175f, 1.0e-5f);
    EXPECT_NEAR(debug.error_yaw_rad, 0.1f, 1.0e-6f);
    EXPECT_FALSE(debug.ppc_active);
}

TEST(AR_BacksteppingPPC, applies_fixed_boundary_ppc)
{
    AR_BacksteppingPPC controller = make_unlimited_controller();
    AR_TidalState state = make_state();
    AR_TrajectoryReference reference = make_reference();
    AR_TidalControlOutput output {};

    state.y_m = -0.3f;
    state.speed_mps = 1.0f;
    reference.speed_mps = 1.0f;

    ASSERT_TRUE(controller.update(state, reference, 0.0f, true, output));
    EXPECT_NEAR(controller.debug().ppc_rho_m, 0.6f, 1.0e-6f);
    EXPECT_NEAR(controller.debug().ppc_xi, 0.5f, 1.0e-6f);
    EXPECT_NEAR(controller.debug().error_y_transformed, 0.5493062f, 1.0e-5f);
    EXPECT_NEAR(output.yaw_rate_cmd_radps, 0.4943756f, 1.0e-5f);
    EXPECT_TRUE(controller.debug().ppc_active);
    EXPECT_FALSE(output.ppc_violation);
}

TEST(AR_BacksteppingPPC, shrinks_ppc_boundary_with_progress)
{
    AR_BacksteppingPPC controller = make_unlimited_controller();
    AR_TidalState state = make_state();
    AR_TrajectoryReference reference = make_reference();
    AR_TidalControlOutput output {};

    ASSERT_TRUE(controller.update(state, reference, 1.0f, true, output));
    EXPECT_NEAR(controller.debug().ppc_rho_m, 0.0895238f, 1.0e-5f);
    EXPECT_NEAR(controller.debug().trajectory_progress, 1.0f, 1.0e-6f);
}

TEST(AR_BacksteppingPPC, relaxes_ppc_boundary_with_slip_difference)
{
    AR_BacksteppingPPC controller = make_unlimited_controller();
    AR_TidalState state = make_state();
    AR_TrajectoryReference reference = make_reference();
    AR_SlipEstimate slip {};
    AR_TidalControlOutput output {};

    slip.slip_left = 0.0f;
    slip.slip_right = 0.2f;
    slip.confidence = 1.0f;
    slip.valid = true;

    ASSERT_TRUE(controller.update(state, reference, 0.0f, slip, true, true, output));
    EXPECT_NEAR(controller.debug().ppc_rho_base_m, 0.6f, 1.0e-6f);
    EXPECT_NEAR(controller.debug().ppc_slip_factor, 0.8807971f, 1.0e-5f);
    EXPECT_NEAR(controller.debug().ppc_rho_m, 0.7056956f, 1.0e-5f);
    EXPECT_TRUE(controller.debug().ppc_slip_scheduled);
}

TEST(AR_BacksteppingPPC, wraps_heading_error)
{
    AR_BacksteppingPPC controller = make_unlimited_controller();
    AR_TidalState state = make_state();
    AR_TrajectoryReference reference = make_reference();
    AR_TidalControlOutput output {};

    state.yaw_rad = 3.13f;
    reference.yaw_rad = -3.13f;

    ASSERT_TRUE(controller.update(state, reference, 0.0f, false, output));
    EXPECT_NEAR(controller.debug().error_yaw_rad, 0.0231853f, 1.0e-5f);
}

TEST(AR_BacksteppingPPC, falls_back_outside_ppc_boundary)
{
    AR_BacksteppingPPC controller = make_unlimited_controller();
    AR_TidalState state = make_state();
    AR_TrajectoryReference reference = make_reference();
    AR_TidalControlOutput output {};

    state.y_m = -0.7f;
    state.speed_mps = 1.0f;
    reference.speed_mps = 1.0f;

    ASSERT_TRUE(controller.update(state, reference, 0.0f, true, output));
    EXPECT_TRUE(output.ppc_violation);
    EXPECT_TRUE(controller.debug().ppc_fallback);
    EXPECT_FALSE(controller.debug().ppc_active);
    EXPECT_NEAR(output.yaw_rate_cmd_radps, 0.63f, 1.0e-5f);
}

TEST(AR_BacksteppingPPC, rate_limits_first_output)
{
    AR_BacksteppingPPC controller;
    AR_TidalState state = make_state();
    AR_TrajectoryReference reference = make_reference();
    AR_TidalControlOutput output {};

    reference.speed_mps = 2.0f;
    reference.yaw_rate_radps = 1.5f;

    ASSERT_TRUE(controller.update(state, reference, 0.0f, false, output));
    EXPECT_NEAR(output.speed_cmd_mps, 0.0381f, 1.0e-6f);
    EXPECT_NEAR(output.yaw_rate_cmd_radps, 0.0635f, 1.0e-6f);
}

TEST(AR_BacksteppingPPC, reset_clears_controller_history)
{
    AR_BacksteppingPPC controller;
    AR_TidalState state = make_state();
    AR_TrajectoryReference reference = make_reference();
    AR_TidalControlOutput output {};

    reference.speed_mps = 2.0f;
    ASSERT_TRUE(controller.update(state, reference, 0.0f, false, output));
    ASSERT_TRUE(controller.update(state, reference, 0.0f, false, output));
    EXPECT_NEAR(output.speed_cmd_mps, 0.0762f, 1.0e-6f);

    controller.reset();
    ASSERT_TRUE(controller.update(state, reference, 0.0f, false, output));
    EXPECT_NEAR(output.speed_cmd_mps, 0.0381f, 1.0e-6f);
}

TEST(AR_TidalControl, optional_placeholders_fail_safe)
{
    AR_TidalControl controller;
    AR_TidalState state = make_state();
    AR_TrajectoryReference reference = make_reference();
    AR_TidalControlOutput output {};

    controller.set_use_ppc(false);
    ASSERT_TRUE(controller.update(state, reference, 0.0f, output));
    EXPECT_TRUE(output.valid);
    EXPECT_FALSE(output.stop_required);

    controller.set_use_residual_smo(true);
    EXPECT_TRUE(controller.update(state, reference, 0.0f, output));
    EXPECT_TRUE(output.valid);
    EXPECT_FALSE(controller.slip_estimate().valid);
}

TEST(AR_TidalControl, integrates_observer_and_wheel_output)
{
    AR_TidalControl controller;
    AR_TidalState state = make_state();
    AR_TrajectoryReference reference = make_reference();
    AR_TidalControlOutput output {};

    controller.set_use_residual_smo(true);
    controller.set_use_wheel_compensation(true);
    controller.set_wheel_rate_limit(12.0f);
    state.speed_mps = 1.0f;
    state.wheel_rate_left_radps = 10.0f;
    state.wheel_rate_right_radps = 10.0f;
    state.wheel_left_time_ms = 100;
    state.wheel_right_time_ms = 100;
    state.wheel_left_valid = true;
    state.wheel_right_valid = true;
    reference.speed_mps = 1.0f;

    ASSERT_TRUE(controller.update(state, reference, 0.0f, output));
    EXPECT_TRUE(output.wheel_rate_output);
    EXPECT_FALSE(output.slip_compensation_applied);

    state.wheel_left_time_ms = 110;
    state.wheel_right_time_ms = 110;
    ASSERT_TRUE(controller.update(state, reference, 0.0f, output));
    EXPECT_TRUE(controller.slip_estimate().valid);
    EXPECT_TRUE(output.wheel_rate_output);
    EXPECT_TRUE(output.slip_compensation_applied);
}

TEST(AR_ResidualSMO, updates_only_on_new_encoder_data)
{
    AR_ResidualSMO observer;
    AR_TidalState state = make_state();
    AR_SlipEstimate estimate {};

    state.speed_mps = 1.0f;
    state.wheel_rate_left_radps = 10.0f;
    state.wheel_rate_right_radps = 10.0f;
    state.wheel_left_time_ms = 100;
    state.wheel_right_time_ms = 100;
    state.wheel_left_valid = true;
    state.wheel_right_valid = true;

    ASSERT_TRUE(observer.update(state, estimate));
    EXPECT_FALSE(estimate.new_data);
    EXPECT_FALSE(estimate.valid);

    state.wheel_left_time_ms = 110;
    state.wheel_right_time_ms = 110;
    ASSERT_TRUE(observer.update(state, estimate));
    EXPECT_TRUE(estimate.new_data);
    EXPECT_TRUE(estimate.valid);
    EXPECT_NEAR(estimate.residual_left, 0.27f, 1.0e-5f);
    EXPECT_NEAR(estimate.residual_right, 0.27f, 1.0e-5f);
    EXPECT_NEAR(estimate.slip_left, 0.00889f, 1.0e-5f);
    EXPECT_NEAR(estimate.slip_right, 0.00889f, 1.0e-5f);

    ASSERT_TRUE(observer.update(state, estimate));
    EXPECT_FALSE(estimate.new_data);
    EXPECT_NEAR(estimate.slip_left, 0.00889f, 1.0e-5f);
}

TEST(AR_ResidualSMO, marks_low_speed_estimate_invalid)
{
    AR_ResidualSMO observer;
    AR_TidalState state = make_state();
    AR_SlipEstimate estimate {};

    state.wheel_left_time_ms = 100;
    state.wheel_right_time_ms = 100;
    state.wheel_left_valid = true;
    state.wheel_right_valid = true;
    ASSERT_TRUE(observer.update(state, estimate));

    state.wheel_left_time_ms = 110;
    state.wheel_right_time_ms = 110;
    ASSERT_TRUE(observer.update(state, estimate));
    EXPECT_TRUE(estimate.new_data);
    EXPECT_FLOAT_EQ(estimate.confidence, 0.0f);
    EXPECT_FALSE(estimate.valid);
}

TEST(AR_ResidualSMO, uses_ned_yaw_sign_for_track_speeds)
{
    AR_ResidualSMO observer;
    AR_TidalState state = make_state();
    AR_SlipEstimate estimate {};

    state.speed_mps = 1.0f;
    state.yaw_rate_radps = 0.5f;
    state.wheel_rate_left_radps = 1.3f / 0.127f;
    state.wheel_rate_right_radps = 0.7f / 0.127f;
    state.wheel_left_time_ms = 100;
    state.wheel_right_time_ms = 100;
    state.wheel_left_valid = true;
    state.wheel_right_valid = true;
    ASSERT_TRUE(observer.update(state, estimate));

    state.wheel_left_time_ms = 110;
    state.wheel_right_time_ms = 110;
    ASSERT_TRUE(observer.update(state, estimate));
    EXPECT_NEAR(estimate.residual_left, 0.0f, 1.0e-5f);
    EXPECT_NEAR(estimate.residual_right, 0.0f, 1.0e-5f);
}

static AR_WheelCompensation make_unlimited_wheel_compensation()
{
    AR_WheelCompensation::Config config;
    config.wheel_accel_max_radps2 = 10000.0f;
    config.wheel_rate_max_radps = 100.0f;
    return AR_WheelCompensation(config);
}

TEST(AR_WheelCompensation, maps_ned_yaw_and_applies_slip)
{
    AR_WheelCompensation compensation = make_unlimited_wheel_compensation();
    AR_SlipEstimate slip {};
    AR_TidalControlOutput output {};

    slip.slip_left = 0.2f;
    slip.slip_right = 0.5f;
    slip.confidence = 1.0f;
    slip.valid = true;

    ASSERT_TRUE(compensation.update(1.0f, 0.5f, 0.01f, slip, output));
    EXPECT_NEAR(output.wheel_rate_left_nominal_radps, 10.2362205f, 1.0e-5f);
    EXPECT_NEAR(output.wheel_rate_right_nominal_radps, 5.5118110f, 1.0e-5f);
    EXPECT_NEAR(output.wheel_rate_left_cmd_radps, 12.7952756f, 1.0e-5f);
    EXPECT_NEAR(output.wheel_rate_right_cmd_radps, 11.0236220f, 1.0e-5f);
    EXPECT_TRUE(output.wheel_rate_output);
    EXPECT_TRUE(output.slip_compensation_applied);
}

TEST(AR_WheelCompensation, disables_compensation_for_invalid_slip)
{
    AR_WheelCompensation compensation = make_unlimited_wheel_compensation();
    AR_SlipEstimate slip {};
    AR_TidalControlOutput output {};

    ASSERT_TRUE(compensation.update(1.0f, 0.0f, 0.01f, slip, output));
    EXPECT_NEAR(output.wheel_rate_left_cmd_radps, 7.8740157f, 1.0e-5f);
    EXPECT_NEAR(output.wheel_rate_right_cmd_radps, 7.8740157f, 1.0e-5f);
    EXPECT_FALSE(output.slip_compensation_applied);
}

TEST(AR_WheelCompensation, limits_wheel_rate_and_ramp)
{
    AR_WheelCompensation compensation;
    AR_SlipEstimate slip {};
    AR_TidalControlOutput output {};

    ASSERT_TRUE(compensation.update(2.0f, 0.0f, 0.01f, slip, output));
    EXPECT_NEAR(output.wheel_rate_left_cmd_radps, 0.3f, 1.0e-6f);
    EXPECT_NEAR(output.wheel_rate_right_cmd_radps, 0.3f, 1.0e-6f);
}

AP_GTEST_MAIN()
