#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "artemis_controller.h"
#include "artemis_protocol.h"
#include "line_indicator.h"

static void assert_near(float actual, float expected, float tolerance)
{
    assert(fabsf(actual - expected) <= tolerance);
}

static artemis_observation_t observation(
    uint32_t sequence_id,
    float sim_time_s,
    float yaw_deg,
    const char *digital)
{
    artemis_observation_t value;
    size_t index;
    const size_t count = strlen(digital);

    memset(&value, 0, sizeof(value));
    value.sequence_id = sequence_id;
    value.sim_time_s = sim_time_s;
    value.yaw_deg = yaw_deg;
    value.digital_count = count < 8U ? count : 8U;
    for (index = 0U; index < value.digital_count; index++) {
        value.digital_values[index] = (uint8_t) (digital[index] == '1');
    }
    return value;
}

static void test_protocol(void)
{
    artemis_response_t response;
    artemis_control_command_t command;
    char output[128];

    assert(artemis_protocol_parse_response(
        "STARTED seq=0 t=0 yaw=1.5 dig=0011100 enc_tl=106.2 enc_tr=106.2",
        &response));
    assert(response.type == ARTEMIS_RESPONSE_STARTED);
    assert(response.observation.digital_count == 7U);
    assert(response.observation.digital_values[2] == 1U);
    assert_near(response.observation.forward_distance_cm, 10.0f, 0.0001f);

    assert(artemis_protocol_parse_response(
        "OBS enc_tr=23 dig=00110001 yaw=-3.25 seq=9 enc_tl=21 t=0.18 extra=ok",
        &response));
    assert(response.type == ARTEMIS_RESPONSE_OBSERVATION);
    assert(response.observation.sequence_id == 9U);
    assert(response.observation.digital_count == 8U);
    assert(response.observation.digital_values[7] == 1U);

    assert(artemis_protocol_parse_response(
        "FINISHED elapsed_time_s=6.3 reached_goal=1 reason=goal", &response));
    assert(response.type == ARTEMIS_RESPONSE_FINISHED);
    assert(response.finished.reached_goal);
    assert(strcmp(response.finished.reason, "goal") == 0);

    assert(artemis_protocol_parse_response(
        "ERR message=Mudri_request_timed_out", &response));
    assert(response.type == ARTEMIS_RESPONSE_ERROR);
    assert(strcmp(response.error_message, "Mudri_request_timed_out") == 0);

    assert(!artemis_protocol_parse_response("OBS seq=1 t=0 yaw=0 dig=0011", &response));
    assert(!artemis_protocol_parse_response(
        "OBS seq=1 t=nan yaw=0 dig=0011 enc_tl=0 enc_tr=0", &response));
    assert(!artemis_protocol_parse_response(
        "OBS seq=1 t=0 yaw=0 dig=00x1 enc_tl=0 enc_tr=0", &response));

    memset(&command, 0, sizeof(command));
    command.sequence_id = 12U;
    command.rear_left_target_speed = 7.25f;
    command.rear_right_target_speed = -1.5f;
    assert(artemis_protocol_format_step(output, sizeof(output), &command) > 0);
    assert(strcmp(output, "STEP 12 7.250000 -1.500000\n") == 0);
    assert(artemis_protocol_format_start(output, sizeof(output)) > 0);
    assert(strcmp(
        output,
        "START max_time_s=120 control_period_s=0.02 initial_progress_index=0\n") == 0);
}

static void test_controllers(void)
{
    artemis_line_controller_t original;
    artemis_line_controller_t fuzzy;
    artemis_yaw_controller_t yaw;
    const uint8_t left_line[8] = {1U, 1U, 0U, 0U, 0U, 0U, 0U, 0U};
    const uint8_t right_line[8] = {0U, 0U, 0U, 0U, 0U, 0U, 1U, 1U};
    bool stable = false;
    float first;
    float second;

    artemis_line_controller_init(&original, ARTEMIS_LINE_CONTROLLER_ORIGINAL);
    assert(artemis_line_controller_scan(&original, left_line, 8U));
    assert_near(original.line_error, -3.5f, 0.0001f);
    assert(artemis_line_controller_compute_turn(&original, 7.0f) > 0.0f);

    artemis_line_controller_reset(&original);
    assert(artemis_line_controller_scan(&original, right_line, 8U));
    assert(artemis_line_controller_compute_turn(&original, 7.0f) < 0.0f);
    first = fabsf(artemis_line_controller_compute_turn(&original, 7.0f));
    assert(artemis_line_controller_scan(&original, right_line, 8U));
    second = fabsf(artemis_line_controller_compute_turn(&original, 7.0f));
    assert(second <= first);

    artemis_line_controller_init(&fuzzy, ARTEMIS_LINE_CONTROLLER_FUZZY);
    assert(artemis_line_controller_scan(&fuzzy, right_line, 8U));
    first = fabsf(artemis_line_controller_compute_turn(&fuzzy, 7.0f));
    assert(artemis_line_controller_scan(&fuzzy, right_line, 8U));
    second = fabsf(artemis_line_controller_compute_turn(&fuzzy, 7.0f));
    assert(second <= first);

    artemis_yaw_controller_reset(&yaw);
    assert(artemis_yaw_controller_compute(&yaw, 1.0f, 359.0f, &stable) > 0.0f);
    assert(!stable);
}

static void test_task3(void)
{
    artemis_mission_t mission;
    artemis_observation_t obs;
    artemis_control_command_t command;
    uint32_t sequence_id = 0U;
    float time_s = 0.0f;
    unsigned int index;

    artemis_mission_reset(&mission);
    obs = observation(sequence_id++, time_s, 0.0f, "00000000");
    command = artemis_mission_step(&mission, &obs);
    assert_near(command.velocity, 7.0f, 0.0001f);
    assert_near(command.turn, 1.894331217f, 0.00001f);
    assert_near(command.rear_left_target_speed, 8.894330978f, 0.00001f);
    assert_near(command.rear_right_target_speed, 5.105669022f, 0.00001f);
    assert(mission.action_index == 0U);

    time_s += 0.02f;
    obs = observation(sequence_id++, time_s, -10.0f, "00110000");
    command = artemis_mission_step(&mission, &obs);
    assert_near(command.turn, 1.882424474f, 0.00001f);
    assert_near(command.rear_left_target_speed, 8.882424355f, 0.00001f);
    assert_near(command.rear_right_target_speed, 5.117575645f, 0.00001f);
    assert(mission.action_index == 0U);
    time_s += 0.02f;
    obs = observation(sequence_id++, time_s, -15.0f, "00110000");
    command = artemis_mission_step(&mission, &obs);
    assert(mission.action_index == 1U);
    assert_near(command.velocity, 7.0f, 0.0001f);
    assert_near(command.turn, 2.279999971f, 0.00001f);
    assert_near(command.rear_left_target_speed, 4.720000267f, 0.00001f);
    assert_near(command.rear_right_target_speed, 9.279999733f, 0.00001f);
    assert(mission.line_seen);

    for (index = 0U; index < 49U; index++) {
        time_s += 0.02f;
        obs = observation(sequence_id++, time_s, -20.0f, "00000000");
        command = artemis_mission_step(&mission, &obs);
        assert(mission.action_index == 1U);
    }
    time_s += 0.02f;
    obs = observation(sequence_id++, time_s, -20.0f, "00000000");
    command = artemis_mission_step(&mission, &obs);
    assert(mission.action_index == 2U);
    assert_near(command.velocity, 0.0f, 0.0001f);

    time_s += 1.21f;
    obs = observation(sequence_id++, time_s, -100.0f, "00000000");
    command = artemis_mission_step(&mission, &obs);
    assert(mission.action_index == 3U);
    assert_near(command.velocity, 7.0f, 0.0001f);

    time_s += 0.02f;
    obs = observation(sequence_id++, time_s, -120.0f, "00011000");
    command = artemis_mission_step(&mission, &obs);
    assert(mission.action_index == 3U);
    time_s += 0.02f;
    obs = observation(sequence_id++, time_s, -125.0f, "00011000");
    command = artemis_mission_step(&mission, &obs);
    assert(mission.action_index == 4U);
    assert(mission.line_seen);

    for (index = 0U; index < 50U; index++) {
        time_s += 0.02f;
        obs = observation(sequence_id++, time_s, -130.0f, "00000000");
        command = artemis_mission_step(&mission, &obs);
    }
    assert(command.completed);
    assert(mission.action_index == 5U);
}

static void test_line_indicator(void)
{
    line_indicator_t indicator;
    const uint8_t clear[8] = {0U};
    const uint8_t line[8] = {0U, 0U, 1U, 1U, 0U, 0U, 0U, 0U};

    line_indicator_reset(&indicator);
    line_indicator_update(&indicator, line, 8U, 0U);
    assert(!line_indicator_is_on(&indicator));
    line_indicator_update(&indicator, clear, 8U, 1U);
    assert(!line_indicator_is_on(&indicator));

    line_indicator_update(&indicator, line, 8U, 10U);
    line_indicator_update(&indicator, line, 8U, 11U);
    assert(line_indicator_is_on(&indicator));
    line_indicator_tick(&indicator, 510U);
    assert(line_indicator_is_on(&indicator));

    line_indicator_update(&indicator, clear, 8U, 20U);
    line_indicator_update(&indicator, clear, 8U, 21U);
    assert(indicator.pending_pulses == 1U);

    line_indicator_tick(&indicator, 511U);
    assert(!line_indicator_is_on(&indicator));
    assert(indicator.phase == LINE_INDICATOR_GAP);
    line_indicator_tick(&indicator, 610U);
    assert(!line_indicator_is_on(&indicator));
    line_indicator_tick(&indicator, 611U);
    assert(line_indicator_is_on(&indicator));
    line_indicator_tick(&indicator, 1111U);
    assert(!line_indicator_is_on(&indicator));
    assert(indicator.phase == LINE_INDICATOR_IDLE);
}

int main(void)
{
    test_protocol();
    test_controllers();
    test_task3();
    test_line_indicator();
    puts("All Artemis firmware host tests passed.");
    return 0;
}
