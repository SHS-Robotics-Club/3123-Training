package org.firstinspires.ftc.teamcode.Components;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * The Claw class defines all the claw components for a robot.
 *
 * Control Hub
 *     s_tb - Servo port 0
 *
 * PUBLIC METHODS:
 *     SimpleServo(hardwareMap) - constructor for instantiating a claw
 *     void init() - initializes the components of the claw
 *     void open() - opens claw (default)
 *     void close() - closes claw
 *     void getTelemetry() - reports claw telemetry information
 * PRIVATE METHODS:
 *     None
 *
 * VERSION   DATE     WHO  DETAIL
 * 00.01.00  05Dec24  SEB  Initial release
 *
 */
public class SimpleServo {

    public static final double SERVO_INIT_POSITION = 0.5;  // Claw position before START

    // Declare claw servo instances
    private Servo s_tb;
    // Shared timer and motion tracking
    private Telemetry telemetry;

    /**
     * - Claw Constructor -
     * Instantiates all the claw components
     * @param hardwareMap the central store for hardware configuration
     * @param telemetry a project shared resource
     */
    public SimpleServo(HardwareMap hardwareMap, Telemetry telemetry) {

        // Instantiate shared resource
        this.telemetry = telemetry;

        try {
            // Instantiate two servos for the claw
            this.s_tb = hardwareMap.get(Servo.class, "s_tb");
        } catch (Exception e) {
            telemetry.addData("Error", "Servo initialization failed: " + e.getMessage());
            telemetry.update();
        }
    }

    /**
     * Initializes all claw components
     * Servo directions are set as opposing and in the middle position
     *
     */
    public void init() {

        if (s_tb != null) {
            // Initialize servoFORWARD);
            s_tb.setPosition(SERVO_INIT_POSITION);
        } else {
            telemetry.addData("Error", "Claw servos are not initialized.");
        }
    }

    /**
     * Starts a smooth move of the servos to the target positions.
     * @param targetPosition The target position for the servo
     */
    public void setCurrentPosition(double targetPosition) {
        s_tb.setPosition(targetPosition);
    }

    /**
     * Starts a smooth move of the servos to the target positions.
     * @return The current position of the servo
     */
    public double getCurrentPosition() {
        return s_tb.getPosition();
    }

    /**
     * Reports the current encoder position and power level for each motor.
     *
     */
    public void getTelemetry() {

        // Send motor data as telemetry data
        telemetry.addData("SERVO ", "Position: %.2f",
                s_tb.getPosition());
    }
}
