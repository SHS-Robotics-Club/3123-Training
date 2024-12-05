package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * The Motor class handles motor control, including power application and position tracking.
 */
public class MotorLimits {

    // goBILDA 5203 Series motor with a 19.2:1 gear ratio
    private static final int ENCODER_TICKS_PER_REVOLUTION = 537;
    private static final double GEAR_RATIO = 1.0; // Ratio between motor and arm shaft
    private static final double TICKS_PER_DEGREE = ENCODER_TICKS_PER_REVOLUTION * GEAR_RATIO / 360.0;
    // Constants for the two fixed positions (position = angle * TICKS_PER_DEGREE)
    public static final int LOW_POSITION_LIMIT = (int) (-10.0 * TICKS_PER_DEGREE);  // Position A
    public static final int HIGH_POSITION_LIMIT = (int) (90.0 * TICKS_PER_DEGREE);  // Position B
    public static final int INIT_POSITION = 0;  // Position B
    public static final double LOW_POWER_FACTOR_LIMIT = 0.25;  // Low power factor
    public static final double HIGH_POWER_FACTOR_LIMIT = 1.0; // High power factor
    public static final double MOTOR_POWER_ZERO = 0.0;  // Power to motors before START
    public static final long SAFETY_TIMEOUT_MS = 5000;  // Loop safety timeout

    private DcMotor m_tb; // The motor object for m_tb
    private double powerFactor = (LOW_POWER_FACTOR_LIMIT + HIGH_POWER_FACTOR_LIMIT) / 2; // Current power factor (1.0 or 0.6)
    private double adjustedPower;
    private Telemetry telemetry; // Telemetry object for reporting status

    /**
     * Constructor for the Motor class.
     * Initializes the motor and sets up telemetry reporting.
     *
     * @param hardwareMap The hardware map to get the motor.
     * @param telemetry   The telemetry object for reporting.
     */
    public MotorLimits(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        try {
            m_tb = hardwareMap.get(DcMotor.class, "m_tb");
        } catch (Exception e) {
            telemetry.addData("Error", "Motor initialization failed: " + e.getMessage());
            telemetry.update();
        }

        // Default values
        this.powerFactor = HIGH_POWER_FACTOR_LIMIT; // Default power factor
    }

    /**
     * Initializes the motor and sets its mode and behavior.
     */
    public void init() {
        if (m_tb != null) {
            m_tb.setDirection(DcMotorSimple.Direction.FORWARD); // Set motor direction
//            m_tb.setPower(MOTOR_POWER_ZERO); // Set motor to zero power for safety
            m_tb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Motor should brake when power is zero
            m_tb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder
            m_tb.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Reset encoder
        } else {
            telemetry.addData("Error", "Motor is not initialized.");
            telemetry.update();
        }
    }

    /**
     * Sets the power factor for the motor (limits the motor power).
     *
     * @param factor The power factor to be applied (either HIGH_POWER or LOW_POWER).
     */
    public void setPowerFactor(double factor) {
        if (factor >= LOW_POWER_FACTOR_LIMIT && factor <= HIGH_POWER_FACTOR_LIMIT) {
            this.powerFactor = factor;
        }
    }

    /**
     * Returns the current power factor.
     *
     * @return The current power factor.
     */
    public double getPowerFactor() {
        return this.powerFactor;
    }

    /**
     * Sets the motor's target position for target position mode.
     *
     * @param position The target position (within valid limits).
     */
    public void setTargetPosition(int position) {

        // Clamp position to operating limits
        if (position < LOW_POSITION_LIMIT) {
            position = LOW_POSITION_LIMIT;
        } else if (position > HIGH_POSITION_LIMIT) {
            position = HIGH_POSITION_LIMIT;
        }

        // un motor to requested position
        m_tb.setTargetPosition(position); // Set target position
        m_tb.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Move the motor to the target position
        m_tb.setPower(HIGH_POWER_FACTOR_LIMIT * powerFactor); // Apply full power with power factor


        ElapsedTime runtime = new ElapsedTime();
        runtime.reset(); // Reset the timer before starting
        long timeout = SAFETY_TIMEOUT_MS; // Timeout value in seconds (for example, 5 seconds)

        // Wait until the motor reaches the target position
        while (m_tb.isBusy() && runtime.milliseconds() < timeout) {
            // Update telemetry periodically to avoid excessive updates
            if (runtime.milliseconds() % 100 < 50) { // Update every 100ms
                telemetry.addData("Running to Position", "Target: %4d, Current: %4d",
                        position, m_tb.getCurrentPosition());
                telemetry.update();
            }
        }

        // If the motor hasn't reached the target position within the timeout
        if (runtime.milliseconds() >= timeout) {
            telemetry.addData("Error", "Timeout reached while moving to position");
            telemetry.update();
        }

        // Ensure the motor is back in the default mode
        m_tb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Returns the current motor position from the encoder.
     *
     * @return The current motor position.
     */
    public int getCurrentPosition() {
        return m_tb.getCurrentPosition();
    }

    /**
     * Reports motor telemetry, including position and power.
     */
    public void getTelemetry() {
        telemetry.addData("Motor Limits", "Power: %.2f  |  Pos: %4d  | Factor: %.2f",
                m_tb.getPower(), m_tb.getCurrentPosition(), powerFactor);
    }

    /**
     * Operates the motor based on the power input and power factor.
     *
     * @param power The power to apply to the motor.
     */
    public void operate(double power) {
        adjustedPower = power * powerFactor;
        m_tb.setPower(adjustedPower);
    }
}
