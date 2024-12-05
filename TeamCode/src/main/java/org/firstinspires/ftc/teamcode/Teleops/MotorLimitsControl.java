package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.MotorLimits;

/**
 * This is the TeleOp class for controlling the Motor.
 * It handles button presses and joystick inputs for controlling motor operation modes
 * and motor movement, while displaying telemetry data.
 */
@TeleOp(name = "L02 - Motor Limits Control", group = "Training")
public class MotorLimitsControl extends OpMode {

    private static final double ZERO_MOTOR_POWER = 0.0;

    private MotorLimits motor;
    private double power = 0.0;
    private boolean isManualMode; // Flag for manual mode

    /**
     * Initialize the motor.
     */
    @Override
    public void init() {
        motor = new MotorLimits(hardwareMap, telemetry); // Pass telemetry object to Motor class
        motor.init();  // Initialize the motor in the Motor class
        isManualMode = true; // Start in manual mode
    }

    /**
     * The init_loop method is used to monitor and report status during initialization.
     * It runs repeatedly while the robot is in the initialization phase.
     */
    @Override
    public void init_loop() {
        telemetry.addData("Motor Initialization", motor != null ? "Success" : "Failed");
        telemetry.addData("Motor Mode", isManualMode ? "Manual" : "Target Position");
        telemetry.addData("Joystick Power", gamepad1.left_stick_y); // Report joystick value
        telemetry.addData("Motor", "Power = %.2f   |   Power Factor = %.2f",
                motor.getPowerFactor() * -gamepad1.left_stick_y, motor.getPowerFactor()); // Actual power applied
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
                isManualMode = false; // Switch to manual mode if joystick is being used
                motor.setTargetPosition(MotorLimits.LOW_POSITION_LIMIT); // Set to Position A
//                motor.operate(1.0); // Apply power to move towards Position A
            } else if (gamepad1.b) {
                isManualMode = false; // Switch to manual mode if joystick is being used
                motor.setTargetPosition(MotorLimits.HIGH_POSITION_LIMIT); // Set to Position B
//                motor.operate(1.0); // Apply power to move towards Position B
            }
            else {
                motor.operate(ZERO_MOTOR_POWER);  // Stop motor if no button is pressed in target position mode
            }
        } else {
            isManualMode = true; // Switch to manual mode if joystick is being used
            // Apply power based on joystick input
            motor.operate(power); // Apply power based on joystick input
        }

        // Report telemetry for debugging
        telemetry.addData("Motor Mode", isManualMode ? "Manual" : "Target Position");
        motor.getTelemetry();
        telemetry.update(); // Report motor status
    }
}
