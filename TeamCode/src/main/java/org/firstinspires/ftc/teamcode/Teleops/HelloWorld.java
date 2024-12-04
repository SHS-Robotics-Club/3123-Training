package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "L00 - Hello World Teleop", group = "Training")
public class HelloWorld extends OpMode {

    /**
     * Called when the driver presses INIT.
     */
    @Override
    public void init() {

        telemetry.addData("Hello", "World");
        telemetry.update();
    }


    /**
     * Called repeatedly while OpMode is playing.
     */
    @Override
    public void loop() {

        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}
