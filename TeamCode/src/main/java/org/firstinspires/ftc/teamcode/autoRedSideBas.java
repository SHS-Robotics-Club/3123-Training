package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;



@Autonomous(name="RedSide bas", group="AutoRedSide")
public class autoRedSideBas extends LinearOpMode {
    private DcMotorEx motorA;
    private DcMotorEx motorB;
    private DcMotorEx bras1;
    private DcMotorEx bras2;

    private Servo coudeG;
    private Servo coudeD;

    private Servo mainG;
    private Servo mainD;
    private Servo lanceur;

    private double zeroDuBras;
    private double zeroDuHaut;

    public void driveForwardPID(double distance, double power) {
        //normalSpeed();
        double ROTATIONS = distance / 0.2827;
        double COUNTS = ROTATIONS * 515.46;
        int leftTarget = (int) COUNTS - motorA.getCurrentPosition();
        int rightTarget = (int) COUNTS + motorB.getCurrentPosition();
        motorA.setTargetPosition(-leftTarget);
        motorB.setTargetPosition(rightTarget);
        motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorA.setPower(power);
        motorB.setPower(power);
        while (motorA.isBusy() || motorB.isBusy()) {
            telemetry.addData("MA", motorA.getTargetPosition());
            telemetry.addData("MAC", motorA.getCurrentPosition());
            telemetry.update();
            idle();
        }
        motorA.setPower(0);
        motorB.setPower(0);
        motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //resetMotors();
    }

    void goLeft(double power) {

        double ROTATIONS = 0.29 / 0.2827;
        double COUNTS = ROTATIONS * 515.46;
        int leftTarget = (int)  COUNTS + motorA.getCurrentPosition();
        int rightTarget = (int) COUNTS + motorB.getCurrentPosition();
        motorA.setTargetPosition(leftTarget);
        motorB.setTargetPosition(rightTarget);
        motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorA.setPower(power);
        motorB.setPower(power);
        while (motorA.isBusy() || motorB.isBusy()) {
            telemetry.addData("MA", motorA.getTargetPosition());
            telemetry.addData("MAC", motorA.getCurrentPosition());
            telemetry.update();
            idle();
        }
        motorA.setPower(0);
        motorB.setPower(0);
        motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void waitTime(double tps) {
        double t=getRuntime();
        while (getRuntime()-t < tps) {idle();}
    }

    void coudeTo(double pos, double speed) {
        double initg = coudeG.getPosition();
        double debugG = initg;
        telemetry.addData("coudeTo", debugG);
        telemetry.update();
        if (pos>initg) {
            while (pos>initg) {

                initg = coudeG.getPosition();
                coudeG.setPosition(initg+speed);
                coudeD.setPosition(1-(initg+speed));
                waitTime(1);
                telemetry.addData("coudeTo", debugG);
                telemetry.addData("goingUP", initg);
                telemetry.update();
            }
        } else {
            while (pos < initg) {
                initg = coudeG.getPosition();
                coudeG.setPosition(initg - speed);
                coudeD.setPosition(1 - (initg - speed));
                waitTime(1);
                telemetry.addData("coudeTo", debugG);
                telemetry.addData("goingDown", initg);
                telemetry.update();
            }
        }
    }

    public void BrasGoTo(double pos) {
        if (bras2.getCurrentPosition() < zeroDuHaut+pos) {
            while (bras2.getCurrentPosition() < zeroDuHaut+pos) {
                bras1.setPower(0.3);
                bras2.setPower(0.3);
            }
            bras1.setPower(0);
            bras2.setPower(0);
        }
        else {
            while (bras2.getCurrentPosition() > zeroDuHaut+pos) {
                bras1.setPower(-0.3);
                bras2.setPower(-0.3);
            }
            bras1.setPower(0);
            bras2.setPower(0);
        }
    }


    @Override
    public void runOpMode() {
        motorA = hardwareMap.get(DcMotorEx.class, "moteur1");
        motorB = hardwareMap.get(DcMotorEx.class, "moteur2");
        bras1 = hardwareMap.get(DcMotorEx.class, "bras1");
        bras2 = hardwareMap.get(DcMotorEx.class, "bras2");
        coudeG = hardwareMap.get(Servo.class, "coudeG");
        coudeD = hardwareMap.get(Servo.class, "coudeD");
        mainG = hardwareMap.get(Servo.class, "mainG");
        mainD = hardwareMap.get(Servo.class, "mainD");
        lanceur = hardwareMap.get(Servo.class, "lanceur");

        double t;
        zeroDuBras = bras2.getCurrentPosition();
        zeroDuHaut = zeroDuBras - 462;



            // Display the light level while we are waiting to start
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();
        waitForStart();



        bras1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bras2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "Started");
        telemetry.update();

        //coudeTo(0.2,0.0001);
        coudeG.setPosition(0.2);
        coudeD.setPosition(1-0.2);
        mainG.setPosition(0);
        mainD.setPosition(1);

        waitTime(3);

        driveForwardPID(0.1,1);

        waitTime(1);

        mainG.setPosition(1);
        mainD.setPosition(0);


        waitTime(1.5);
        coudeG.setPosition(0.91);
        coudeD.setPosition(1-0.91);

        bras1.setPower(-0.3);
        bras2.setPower(-0.3);
        waitTime(0.5);
        bras1.setPower(0);
        bras2.setPower(0);

        driveForwardPID(0.6,1);
        goLeft(1);
        driveForwardPID(1.8,1);

        BrasGoTo(300);
        coudeG.setPosition(0.2);
        coudeD.setPosition(1-0.2);



        waitTime(3);

        while (opModeIsActive()) {
            coudeG.setPosition(0.91);
            coudeD.setPosition(1-0.91);
            mainG.setPosition(0);
            mainD.setPosition(1);
            telemetry.addData("Status", "Wating for end");
            telemetry.update();
        }
    }
}
