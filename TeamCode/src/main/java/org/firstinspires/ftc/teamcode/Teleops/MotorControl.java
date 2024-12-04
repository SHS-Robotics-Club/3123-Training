package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Components.Motor;


/**
 * This is the TeleOp class for controlling the Motor.
 * It handles button presses and joystick inputs for controlling motor operation modes
 * and motor movement, while displaying telemetry data.
 */
@TeleOp(name = "L01 - Motor Drive Control", group = "Training")
public class MotorControl extends OpMode {

    private static final double BUTTON_A_POWER_FACTOR = 0.65;  // Low power factor
    private static final double BUTTON_B_POWER_FACTOR = 0.85; // High power factor
    private static final double ZERO_MOTOR_POWER = 0.0;  // Low power factor

    private Motor motor;
    private double power = ZERO_MOTOR_POWER;

    /**
     * Initialize the motor.
     */
    @Override
    public void init() {

        motor = new Motor(hardwareMap, telemetry); // Pass telemetry object to Motor class

        motor.init();  // Initialize the motor in the Motor class
    }

    /**
     * The init_loop method is used to monitor and report status during initialization.
     * It runs repeatedly while the robot is in the initialization phase.
     */
    @Override
    public void init_loop() {
        telemetry.addData("Motor Initialization", motor != null ? "Success" : "Failed");
        telemetry.addData("Joystick Power", gamepad1.left_stick_y); // Report joystick value
        telemetry.addData("Motor Power", "Power = %.2f   |   Power Factor = %.2f",
                motor.getPowerFactor(), motor.getPowerFactor() * gamepad1.left_stick_y); // Actual power applied
        telemetry.update();
    }

    /**
     * The main loop for the TeleOp control.
     * Handles joystick and button input to control the motor and other devices.
     */
    @Override
    public void loop() {

        // Get joystick input for manual mode
        power = -gamepad1.left_stick_y;

        // If joystick Y is near zero, switch to target position mode
        if (Math.abs(power) < 0.1) {
            // Check for button presses to set the target position
            if (gamepad1.a) {
                motor.setPowerFactor(BUTTON_A_POWER_FACTOR); // Set to Position A
            } else if (gamepad1.b) {
                motor.setPowerFactor(BUTTON_B_POWER_FACTOR); // Set to Position B
            } else {
                motor.operate(ZERO_MOTOR_POWER);  // Stop motor if no button is pressed in target position mode
            }
        } else {
            // Apply power based on joystick input
            motor.operate(power); // Apply power based on joystick input
        }

        // Report telemetry for debugging
        telemetry.addData("Joystick Power", -gamepad1.left_stick_y); // Report joystick value
        motor.getTelemetry(); // Report motor status
        telemetry.update();
    }
}
