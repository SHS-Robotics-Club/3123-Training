package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * The Motor class handles motor control, including power application and position tracking.
 */
public class Motor {

    // Constants for the two fixed positions
    private static final double LOW_POWER_FACTOR_LIMIT = 0.6;  // Low power factor
    private static final double HIGH_POWER_FACTOR_LIMIT = 1.0; // High power factor
    private static final double MOTOR_POWER_ZERO = 0.0;  // Power to motors before START
    private static final double ABSOLUTE_MAX_POWER = 1.0;  // Maximum power to motors

    private DcMotor m_tb; // The motor object for m_tb
    private double powerFactor; // Current power factor (1.0 or 0.6)
    private double adjustedPower;
    private Telemetry telemetry; // Telemetry object for reporting status

    /**
     * Constructor for the Motor class.
     * Initializes the motor and sets up telemetry reporting.
     *
     * @param hardwareMap The hardware map to get the motor.
     * @param telemetry   The telemetry object for reporting.
     */
    public Motor(HardwareMap hardwareMap, Telemetry telemetry) {

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
            m_tb.setPower(MOTOR_POWER_ZERO); // Set motor to zero power for safety
            m_tb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Motor should brake when power is zero
            m_tb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder
            m_tb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // No PID control
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
     * Operates the motor based on the power input and power factor.
     *
     * @param power The power to apply to the motor.
     */
    public void operate(double power) {
        adjustedPower = power * powerFactor;
        m_tb.setPower(adjustedPower);
    }

    /**
     * Reports motor telemetry, including position and power.
     */
    public void getTelemetry() {
        telemetry.addData("Motor", "Power: %.2f   |   Power Factor: %.2f",
                m_tb.getPower(), powerFactor);
    }

}
