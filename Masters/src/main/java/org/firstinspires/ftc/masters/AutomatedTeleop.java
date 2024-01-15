package org.firstinspires.ftc.masters;


import static org.firstinspires.ftc.masters.CSCons.backSlidesPos;
import static org.firstinspires.ftc.masters.CSCons.claw;
import static org.firstinspires.ftc.masters.CSCons.clawOpen;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.CSCons.*;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "Center Stage automated teleop", group = "competition")
public class AutomatedTeleop extends LinearOpMode {

    static int target = 0;
    OuttakePosition backSlidePos = OuttakePosition.BOTTOM;

    private final double ticks_in_degrees = 384.5 / 180;

    PIDController controller;

    public static double p = 0.01, i = 0, d = 0.0001;
    public static double f = 0.05;

//    double y = 0;
//    double x = 0;
//    double rx = 0;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private final ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftRearMotor = null;
    DcMotor rightRearMotor = null;

    DcMotor gpSlideRight = null;
    DcMotor gpSlideLeft = null;
    DcMotor backSlides = null;
    DcMotor hangingMotor = null;

    Servo planeLaunch;
    Servo planeRaise;
    Servo clawServo;
    Servo clawArm;
    Servo clawAngle;
    Servo cameraTurning;
    Servo outtakeHook;
    Servo outtakeRotation;
    Servo outtakeMovementRight;
    Servo outtakeMovementLeft;

    TouchSensor touchSensor;

    boolean clawClosed = true;
    boolean hookEngaged = false;
    boolean hookPressed = false;
    boolean presetPushed = false;
    double claw2transfer_time = -1;
    double claw2transfer_delay = .5;
    double x_last_pressed = -1;

    int backSlidesTargetPos = 0;
    int presetBackSlidesTargetPos = 0;
    boolean outtakeGoingToTransfer;
    double outtakeRotationTarget;

    int v4bPresetTarget = 0;


    private enum Retract {
        back,
        flip_bar,
        cartridge
    }

    private boolean isRetracting = false;

    private DriveMode driveMode = DriveMode.NORMAL;
    private OuttakeState outtakeState = OuttakeState.MoveToTransfer;
    private IntakeState intakeState = IntakeState.Transition;

    private Retract retract = Retract.back;

    private ClawPosition clawPosition = ClawPosition.OPEN;
    private HookPosition hookPosition = HookPosition.OPEN;
    RevColorSensorV3 colorSensor;
    double angleRotationAdjustment = 0;

    @Override
    public void runOpMode() {
        /*
            Controls
            --------
            Gamepad2

            Gamepad1

            --------
Drive with sticks intake is the front
Four bar position 1-5 auto stack pixels
Slides position 1-11 placement on backboard
Normal mode - pixel grabbing mode - entered with left stick down
D-pad controls extendo
D-pad up - extendo fully extended with four bar in position 4
D-pad right - extendo out 2/3? with four bar in position 1
D-pad down - extendo out 1/3? with four bar in position 1
D-pad left - extendo fully in and four bar in position 1
Buttons
A - claw opens and closes
B - transfers and moves to slide pos 1
X - auto aligns and switches to pixel scoring mode
Y - press once drop one pixel, hold for drop both pixels, once both are placed outtake goes back into transfer
LB - four bar down (presets)
RB - four bar up (presets)
LT - slides down (presets)
RT - slides up (presets)

Pixel scoring mode - entered with x while in normal
Only moves right and left no forward/ backwards
LT - slides down (presets)
RT - slides up (presets)
Y - press once drop one pixel, hold for drop both pixels

Endgame mode - entered with right stick down
Normal driving
X - Auto aligns on April tag for shooter, raise shooter, shoot
LB - Hang down
RB - Hang up
        */
        leftFrontMotor = hardwareMap.dcMotor.get("frontLeft");
        rightFrontMotor = hardwareMap.dcMotor.get("frontRight");
        leftRearMotor = hardwareMap.dcMotor.get("backLeft");
        rightRearMotor = hardwareMap.dcMotor.get("backRight");

        hangingMotor = hardwareMap.dcMotor.get("hangingMotor");
        gpSlideLeft = hardwareMap.dcMotor.get("gpSlideLeft");
        gpSlideRight = hardwareMap.dcMotor.get("gpSlideRight");
        backSlides = hardwareMap.dcMotor.get("backSlides");

        planeLaunch = hardwareMap.servo.get("planeLaunch");
        planeRaise = hardwareMap.servo.get("planeRaise");
        clawServo = hardwareMap.servo.get("clawServo");
        clawArm = hardwareMap.servo.get("clawArm");
        clawAngle = hardwareMap.servo.get("clawAngle");
        cameraTurning = hardwareMap.servo.get("cameraTurning");
        outtakeHook = hardwareMap.servo.get("outtakeHook");
        outtakeRotation = hardwareMap.servo.get("outtakeRotation");
        outtakeMovementRight = hardwareMap.servo.get("outtakeMovementRight");
        outtakeMovementLeft = hardwareMap.servo.get("outtakeMovementLeft");
        touchSensor = hardwareMap.touchSensor.get("touch");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");


        // Set the drive motor direction:
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //backSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        clawArm.setPosition(CSCons.clawArmTransition);
        clawAngle.setPosition(CSCons.clawAngleTransition);
        clawServo.setPosition(CSCons.clawOpen);

        outtakeMovementLeft.setPosition(CSCons.outtakeMovementBackTransfer);
        outtakeMovementRight.setPosition(CSCons.outtakeMovementBackTransfer);
        outtakeRotation.setPosition(CSCons.outtakeAngleTransfer);
        outtakeHook.setPosition(CSCons.openHook);
        hookPosition = HookPosition.OPEN;

        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);

        target = backSlidePos.getTarget();
        angleRotationAdjustment = 0;

        ElapsedTime intakeElapsedTime = null, outtakeElapsedTime = null;


        waitForStart();

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//            telemetry.addData("linear slide encoder",  + linearSlideMotor.getCurrentPosition());

            switch (driveMode) {

                case NORMAL:
                case END_GAME:

                    drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
                    break;
                case PIXEL_SCORE:
                    drive(gamepad1.left_stick_x, 0, 0);
                    break;
            }

            backSlidesMove(target);

            switch (intakeState) {
                case Intake:
                    if (gamepad2.a) {
                        if (clawPosition == ClawPosition.OPEN || clawPosition == ClawPosition.TRANSFER) {
                            clawPosition = ClawPosition.CLOSED;
                            clawServo.setPosition(CSCons.clawClosed);
                        } else if (clawPosition == ClawPosition.CLOSED) {
                            clawPosition = ClawPosition.OPEN;
                            clawServo.setPosition(clawOpen);
                        }
                    }

                    if (clawPosition == ClawPosition.OPEN && colorSensor.getDistance(DistanceUnit.CM) < CSCons.closeClawDistance) {
                        clawPosition = ClawPosition.CLOSED;
                        clawServo.setPosition(CSCons.clawClosed);

                    }

                    if (gamepad2.dpad_up) {
                        intakeState = IntakeState.Transfer;
                        clawAngle.setPosition(CSCons.clawAngleTransfer);
                        clawArm.setPosition(CSCons.clawArmTransfer + CSCons.skewampus);
                    }

                    if (gamepad2.dpad_left) {
                        intakeState = IntakeState.Transition;
                        clawAngle.setPosition(CSCons.clawAngleTransition);
                        clawArm.setPosition(CSCons.clawArmTransition);
                    }


                    break;
                case MoveToIntake:
                    if (intakeElapsedTime != null && intakeElapsedTime.time(TimeUnit.MILLISECONDS) > CSCons.transferToBottomIntake) {
                        intakeState = IntakeState.Intake;
                        clawPosition = ClawPosition.OPEN;
                        clawServo.setPosition(clawOpen);
                    }


                    break;
                case Transfer:
                    //is there a reason we would ever want to close the claw here?
                    if (gamepad2.a && outtakeState == OuttakeState.ReadyToTransfer) {
                        clawPosition = ClawPosition.TRANSFER;
                        clawServo.setPosition(CSCons.clawTransfer);
                    }


                    if (gamepad2.dpad_down) {
                        intakeState = IntakeState.MoveToIntake;
                        intakeElapsedTime = new ElapsedTime();
                        clawAngle.setPosition(CSCons.clawAngleGroundToThree);
                        clawArm.setPosition(CSCons.clawArmGround);
                    }

                    if (gamepad2.dpad_left) {
                        intakeState = IntakeState.Transition;
                        clawAngle.setPosition(CSCons.clawAngleTransition);
                        clawArm.setPosition(CSCons.clawArmTransition);
                    }


                    break;
                case Transition:

                    if (gamepad2.dpad_down) {
                        intakeState = IntakeState.MoveToIntake;
                        intakeElapsedTime = new ElapsedTime();
                        clawAngle.setPosition(CSCons.clawAngleGroundToThree);
                        clawArm.setPosition(CSCons.clawArmGround);
                    }


                    if (gamepad2.dpad_up) {
                        intakeState = IntakeState.Transfer;
                        clawAngle.setPosition(CSCons.clawAngleTransfer);
                        clawArm.setPosition(CSCons.clawArmTransfer + CSCons.skewampus);
                    }

                    break;
            }

            switch (outtakeState){
                case ReadyToTransfer:
                    if (gamepad2.x && hookPosition==HookPosition.CLOSED){
                        if (outtakeElapsedTime==null || outtakeElapsedTime.time(TimeUnit.MILLISECONDS)>300) {
                            outtakeElapsedTime = new ElapsedTime();
                            hookPosition = HookPosition.OPEN;
                            outtakeHook.setPosition(CSCons.openHook);
                        } else {
                            outtakeHook.setPosition(CSCons.closeHook);
                        }
                    }
                    if (gamepad2.x && hookPosition == HookPosition.OPEN){
                        if(outtakeElapsedTime == null || outtakeElapsedTime.time(TimeUnit.MILLISECONDS)>300){
                            outtakeElapsedTime = closeHook();
                        } else {
                        outtakeHook.setPosition(CSCons.openHook);
                        }
                    }

                    if (gamepad2.left_trigger>0.5){
                        backSlidePos = OuttakePosition.LOW;
                       outtakeElapsedTime = closeHook();
                       outtakeState = OuttakeState.ClosingHook;

                    }
                    if (gamepad2.right_trigger>0.5){
                        backSlidePos = OuttakePosition.MID;
                        outtakeElapsedTime = closeHook();
                        outtakeState = OuttakeState.ClosingHook;
                    }

                    break;
                case MoveToTransfer:


                    //if touch sensor => ready to transfer
                    break;
                case ReadyToDrop:
                    if (gamepad2.x && hookPosition==HookPosition.CLOSED){
                        if (outtakeElapsedTime==null || outtakeElapsedTime.time(TimeUnit.MILLISECONDS)>300) {
                            outtakeElapsedTime = new ElapsedTime();
                            hookPosition = HookPosition.OPEN;
                            outtakeHook.setPosition(CSCons.openHook);
                        } else {
                            outtakeHook.setPosition(CSCons.closeHook);
                        }
                    }
                    if (gamepad2.x && hookPosition == HookPosition.OPEN){
                        if(outtakeElapsedTime == null || outtakeElapsedTime.time(TimeUnit.MILLISECONDS)>300){
                            outtakeElapsedTime = closeHook();
                        } else {
                            outtakeHook.setPosition(CSCons.openHook);
                        }
                    }

                    if (gamepad2.left_trigger>0.5){
                        backSlidePos = OuttakePosition.LOW;
                        target = backSlidePos.getTarget();

                    }
                    if (gamepad2.right_trigger>0.5){
                        backSlidePos = OuttakePosition.MID;
                        target = backSlidePos.getTarget();
                    }


                    //what button to mode back to transfer?
                    // what order to move (slide down or flip first?










                    break;
                case MoveToDrop:
                    if (backSlides.getCurrentPosition()>100){
                        outtakeMovementLeft.setPosition(CSCons.outtakeMovementBackDrop);
                        outtakeMovementRight.setPosition(CSCons.outtakeMovementBackDrop);
                        outtakeRotationTarget = CSCons.outtakeAngleFolder + angleRotationAdjustment;
                    }
                    if (backSlides.getCurrentPosition()>backSlidePos.getTarget()-100){
                        outtakeState = OuttakeState.ReadyToDrop;
                    }
                    break;
                case Align:
                    break;
                case BackUp:
                    break;
                case ClosingHook:
                    if (outtakeElapsedTime!=null && outtakeElapsedTime.time(TimeUnit.MILLISECONDS)>CSCons.transferToScoreOuttake){
                        outtakeState = OuttakeState.MoveToDrop;
                        target = backSlidePos.getTarget();
                    }
                    break;
            }








            if (outtakeState == OuttakeState.ReadyToDrop || outtakeState == OuttakeState.ReadyToTransfer) {

                if (gamepad2.x && outtakeHook.getPosition() <= CSCons.outtakeHook[1] + .1 && outtakeHook.getPosition() >= CSCons.outtakeHook[1] - .1 && runtime.time() > x_last_pressed + .4) {
                    outtakeHook.setPosition(CSCons.openHook);
                    hookPosition = HookPosition.OPEN;

                    x_last_pressed = runtime.time();
                } else if (gamepad2.x && outtakeHook.getPosition() <= CSCons.outtakeHook[0] + .1 && outtakeHook.getPosition() >= CSCons.outtakeHook[0] - .1 && runtime.time() > x_last_pressed + .4) {
                    outtakeHook.setPosition(CSCons.closeHook);
                    hookPosition = HookPosition.CLOSED;
                    x_last_pressed = runtime.time();
                }
            }

            //outtake

            if (gamepad2.left_stick_y > 0.2) { // move to backdrop
                outtakeHook.setPosition(CSCons.closeHook);
                outtakeState = OuttakeState.ClosingHook;
                outtakeElapsedTime = new ElapsedTime();
            }

            //does the slide need to go up before the flip?
            if (outtakeState == OuttakeState.ClosingHook && outtakeElapsedTime != null && outtakeElapsedTime.time(TimeUnit.MILLISECONDS) > CSCons.closingHook) {

                hookPosition = HookPosition.CLOSED;
                backSlidePos = OuttakePosition.LOW;
                if (backSlides.getCurrentPosition() > 150) {
                    outtakeMovementLeft.setPosition(CSCons.outtakeMovementBackDrop);
                    outtakeMovementRight.setPosition(CSCons.outtakeMovementBackDrop);
                    outtakeRotationTarget = CSCons.outtakeAngleFolder + angleRotationAdjustment;
                    outtakeState = OuttakeState.MoveToDrop;
                }
            }

            if (outtakeState == OuttakeState.MoveToDrop && backSlides.getCurrentPosition() > OuttakePosition.LOW.getTarget() - 100) {
                outtakeState = OuttakeState.ReadyToDrop;
            }


            if (gamepad2.left_stick_y < -0.2 && backSlidePos != OuttakePosition.BOTTOM) {
                outtakeMovementLeft.setPosition(CSCons.outtakeMovementBackTransfer);
                outtakeMovementRight.setPosition(CSCons.outtakeMovementBackTransfer);
                outtakeGoingToTransfer = true;
                outtakeHook.setPosition(CSCons.outtakeHook[0]);
            }

            if (gamepad2.left_bumper) {
                angleRotationAdjustment += .005;
            }

            outtakeRotation.setPosition(outtakeRotationTarget);


            if (gamepad2.left_trigger > 0.5) {

                if (outtakeState != OuttakeState.ReadyToDrop) {

                }

                backSlidePos = OuttakePosition.LOW;
                target = backSlidePos.getTarget();
            }
            if (gamepad2.right_trigger > 0.5) {
                backSlidePos = OuttakePosition.HIGH;
                target = backSlidePos.getTarget();
            }

            if (gamepad2.right_bumper) {

                backSlidePos = OuttakePosition.BOTTOM;
                target = backSlidePos.getTarget();
            }

            if (gamepad1.a) {
                clawServo.setPosition(claw[2]);
            }


            telemetry.addData("left y", gamepad1.left_stick_y);
            telemetry.addData("left x", gamepad1.left_stick_x);
            telemetry.addData("right x", gamepad1.right_stick_x);
            telemetry.addData("Arm", clawArm.getPosition());
            telemetry.addData("backSlides", target); // 1725, 2400,
            telemetry.addData("time", runtime.time());

            telemetry.update();
        }


    }

    protected void drive(double x, double y, double rx) {

        if (Math.abs(y) < 0.2) {
            y = 0;
        }
        if (Math.abs(x) < 0.2) {
            x = 0;
        }

        double leftFrontPower = y + x + rx;
        double leftRearPower = y - x + rx;
        double rightFrontPower = y - x - rx;
        double rightRearPower = y + x - rx;

        if (Math.abs(leftFrontPower) > 1 || Math.abs(leftRearPower) > 1 || Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1) {

            double max;
            max = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
            max = Math.max(max, Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(rightRearPower));

            leftFrontPower /= max;
            leftRearPower /= max;
            rightFrontPower /= max;
            rightRearPower /= max;
        }

        leftFrontMotor.setPower(leftFrontPower);
        leftRearMotor.setPower(leftRearPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightRearMotor.setPower(rightRearPower);
    }

    protected void backSlidesMove(int target) {

        int slidePos = backSlides.getCurrentPosition();
        double pid = controller.calculate(slidePos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        double liftPower = pid + ff;

        backSlides.setPower(liftPower);
    }

    protected ElapsedTime closeHook(){
        ElapsedTime time = new ElapsedTime();
        hookPosition = HookPosition.CLOSED;
        outtakeHook.setPosition(CSCons.closeHook);
        return time;
    }

}
