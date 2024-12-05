package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Components.SimpleServo;

//@Disabled
// Possible Groups: Competition, Development, Test, Training
@TeleOp(name = "L03 - Simple Servo Control", group = "Training")

/**
 * SimpleServoControl
 * Controls servo using left joystick Y for position.
 * Uses left bumper for fixed position called "open" and right bumper for fixed position called "close".
 *
 * VERSION   DATE     WHO  DETAIL
 * 00.01.00  24Nov24  SEB  Initial release
 *
 */
public class SimpleServoControl extends OpMode {

    // Left servo constants
    public static final double SERVO_POSITION_0 = 0;  // Claw position before START
    public static final double SERVO_POSITION_1 = 0.3;  // Claw open position
    public static final double SERVO_POSITION_2 = 0.6;  // Power to motors before START
    public static final double SERVO_POSITION_3 = 1.0;  // Power to motors before START

    // Define as instance of a Robot class as null
    private SimpleServo servo;
    // Define local parameters
    private double position;

    /**
     * Instantiates and initializes the subsystems and utilities
     * Runs once
     */
    @Override
    public void init(){

        // Instantiate a robot using the hardwareMap constructor
        servo = new SimpleServo(hardwareMap, telemetry);

        // Initialize robot subsystems
        servo.init();

        // Set default telemetry
        telemetry.addData("Servo Status", "Initialized");
        servo.getTelemetry();
        telemetry.update(); // Send "Initialized" and powerFactor to the Driver Station
    }

    /**
     * Initialization complete. Wait here for PLAY.
     */
    @Override
    public void init_loop() {

        // Report telemetry while waiting
        telemetry.addData("Servo Status", "Initialized");
        servo.getTelemetry();
        telemetry.update(); // Send "Initialized" and powerFactor to
    }

    /**
     * Main TeleOp operation loop
     * Wait here until STOP
     */
    @Override
    public void loop() {

        if (gamepad1.dpad_down) {
            servo.setCurrentPosition(SERVO_POSITION_0);
        } else if (gamepad1.dpad_left) {
            servo.setCurrentPosition(SERVO_POSITION_1);
        } else if (gamepad1.dpad_up) {
            servo.setCurrentPosition(SERVO_POSITION_2);
        } else if (gamepad1.dpad_right) {
            servo.setCurrentPosition(SERVO_POSITION_3);
        }

        // Report telemetry from subsystems
        servo.getTelemetry();
        telemetry.update();
    }
}
