#include <AP_gtest.h>

#include <AR_TidalControl/AR_BacksteppingPPC.h>
#include <AR_TidalControl/AR_TidalControl.h>
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
    EXPECT_FALSE(controller.update(state, reference, 0.0f, output));
    EXPECT_FALSE(output.valid);
    EXPECT_TRUE(output.stop_required);
}

AP_GTEST_MAIN()
