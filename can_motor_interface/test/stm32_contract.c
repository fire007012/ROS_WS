/* Reuse the firmware's fake HAL and regressions, without modifying firmware. */
#define main firmware_regression_main
#include CAN_FIRMWARE_REGRESSION
#undef main

void ros_encode_command(uint8_t command, uint8_t index, int32_t value, uint8_t accel,
                        uint32_t *id, uint8_t *dlc, uint8_t *data);
void ros_encode_wheel(uint8_t wheel, int32_t rpm, uint32_t *id, uint8_t *dlc, uint8_t *data);
void ros_encode_arm(double radians, uint32_t *id, uint8_t *dlc, uint8_t *data);

static void receive_ros_command(uint8_t command, uint8_t index, int32_t value, uint8_t accel)
{
    uint32_t id;
    uint8_t dlc, data[8];
    ros_encode_command(command, index, value, accel, &id, &dlc, data);
    CAN_RxHeaderTypeDef header = {.StdId = id, .IDE = CAN_ID_STD,
                                .RTR = CAN_RTR_DATA, .DLC = dlc};
    assert(id == 0x100U && dlc == 8U);
    CAN1_RxCallback(header, data);
}

int main(void)
{
    assert(firmware_regression_main() == 0);
    Motor_Command_t command;
    /* Reproduce the original ROS DLC=7 rejection in the actual CAN1 parser. */
    reset();
    CAN_RxHeaderTypeDef old_header = {.StdId = 0x100, .IDE = CAN_ID_STD,
                                    .RTR = CAN_RTR_DATA, .DLC = 7};
    uint8_t old_data[8] = {1, 0, 100, 0, 0, 0, 101, 0};
    CAN1_RxCallback(old_header, old_data);
    assert(motor_fetch_command(&command, 0) == 0U);
    puts("PASS reproduced original ROS seven-byte command rejection in STM32 parser");

    const int32_t targets[] = {100, -100, 0, 30, -30};
    for (unsigned wheel = 0; wheel < 4; wheel++) {
        for (unsigned t = 0; t < sizeof(targets) / sizeof(targets[0]); t++) {
            reset();
            zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
            receive_ros_command(1, (uint8_t)wheel, targets[t], 50);
            assert(motor_fetch_command(&command, 0) == 1U);
            assert(command.cmd == 1 && command.motor_idx == wheel && command.value == targets[t]);
            assert(command.param0 == 50 && command.param1 == 0);
            assert(motor_apply_command(&command) == HAL_OK);
            drain();
            const uint16_t speed = (uint16_t)(abs(targets[t]) * 10);
            const uint8_t expected[8] = {0xF6, (targets[t] < 0) ? 1 : 0, 0, 50,
                                        (uint8_t)(speed >> 8), (uint8_t)speed, 0, 0x6B};
            expect_frame(1, 0, (wheel + 1U) << 8, expected, 8);
        }
    }
    puts("PASS actual ROS encoder -> STM32 parser -> motor control -> exact CAN2 F6 bytes (20 cases)");

    for (unsigned wheel = 0; wheel < 4; ++wheel) {
        uint32_t id; uint8_t dlc, data[8];
        const uint8_t addresses[] = {2, 1, 4, 3};
        reset();
        zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
        ros_encode_wheel((uint8_t)wheel, 30, &id, &dlc, data);
        CAN_RxHeaderTypeDef header = {.StdId = id, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = dlc};
        CAN1_RxCallback(header, data);
        assert(motor_fetch_command(&command, 0) == 1U);
        assert(motor_apply_command(&command) == HAL_OK);
        drain();
        expect_frame(1, 0, addresses[wheel] << 8, (uint8_t[]){0xF6, 0, 0, 50, 1, 0x2C, 0, 0x6B}, 8);
    }
    puts("PASS logical wheel order maps to physical CAN2 addresses 2/1/4/3");

    for (int sign = -1; sign <= 1; sign += 2) {
        uint32_t id; uint8_t dlc, data[8];
        reset();
        zdt_set_response_policy(ZDT_RESPONSE_FIRE_AND_FORGET);
        receive_ros_command(4, 4, 100, 0);
        assert(motor_fetch_command(&command, 0) == 1U);
        assert(motor_apply_command(&command) == HAL_OK);
        ros_encode_arm(sign * 3.141592653589793, &id, &dlc, data);
        CAN_RxHeaderTypeDef header = {.StdId = id, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = dlc};
        CAN1_RxCallback(header, data);
        assert(motor_fetch_command(&command, 0) == 1U && command.motor_idx == 4);
        assert(motor_apply_command(&command) == HAL_OK);
        drain();
        expect_frame(1, 0, 0x500, (uint8_t[]){0xFB, sign < 0 ? 1 : 0, 3, 0xE8, 0, 0, 7, 8}, 8);
        expect_frame(1, 1, 0x501, (uint8_t[]){0xFB, 1, 0, 0x6B}, 4);
    }
    puts("PASS arm +/-pi radians -> STM32 index 4 -> CAN2 address 5 absolute +/-180 degrees");

    reset();
    motor_set_report_mask(3); /* firmware boot default has no velocity bit */
    receive_ros_command(7, 0xFF, 7, 0);
    assert(motor_fetch_command(&command, 0) == 1U);
    assert(motor_apply_command(&command) == HAL_OK);
    assert((motor_get_report_mask() & MOTOR_REPORT_VELOCITY) != 0);
    tick = 501;
    assert(motor_is_ros_timeout(500) == 1U);
    receive_ros_command(9, 0xFF, 0, 0);
    assert(motor_is_ros_timeout(500) == 0U);
    assert(motor_fetch_command(&command, 0) == 1U && command.cmd == MOTOR_CMD_HEARTBEAT);
    puts("PASS ROS report-mask request enables velocity feedback; heartbeat refreshes STM32 watchdog");
    return 0;
}
