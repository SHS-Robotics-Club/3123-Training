package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FourEyesRobot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "YellowSideAuto")
public class YellowSideAuto extends LinearOpMode {

    //State Machine Variables
    enum States {
        DEPOSIT_SAMPLE, HANG_SPECIMIN,
        COLLECT_SAMPLE, COLLECT_SPECIMIN,
        START_TO_SPECIMIN_HANG,
        SPECIMIN_HANG_TO_SPIKE,
        SPIKE_TO_HUMAN,
        HUMAN_TO_NEXT_SPIKE

    };

    States currentState;

    private FourEyesRobot robot;
    private MecanumDrive roadRunnerDrive;

    //Locations
    private Pose2d startPosition = new Pose2d(-12,-60,Math.toRadians(90));
    private Vector2d bucketPosition = new Vector2d(-55, -65);
    private Vector2d yellowSpikeRight = new Vector2d(-49, -26);
    private Vector2d yellowSpikeMiddle = new Vector2d(-61, -26);
    private Vector2d yellowSpikeLeft = new Vector2d(-70, -26);

    //Subsystem info
    double armExtentionLength = 15;

    @Override
    public void runOpMode() throws InterruptedException {

        //Init robot
        currentState = States.START_TO_SPECIMIN_HANG;
//        Lift lift = new Lift(hardwareMap);
//        Arm arm = new Arm(hardwareMap);



        robot = new FourEyesRobot(hardwareMap);

        roadRunnerDrive = new MecanumDrive(hardwareMap, startPosition);
        waitForStart();
        //Auto Begins


        //This is the primary action loop.
        Actions.runBlocking(new ParallelAction(
                robot.autoPID(), //This PID loop will be constantly running in the background
                new SequentialAction(
                        new InstantAction(robot::initializePowerStates), //Initializes robot's servos specifically

                        //Score Sample Pre-Load
                        new InstantAction(() -> this.addTelemetryMessage("Driving to bucket...")),
                        new InstantAction(robot::depositSamplePosForward), //Set new target position
                        strafeWithSubsystems(startPosition, new Pose2d(bucketPosition,Math.toRadians(180))),
                        //Deposit Via Claw
                        new InstantAction(robot::openClaw),
                        new SleepAction(0.5), //Wait for claw to completely open and deposit



                        //Retrieve Right Spike Mark
                        new InstantAction(() -> this.addTelemetryMessage("Driving to Spike Mark 1...")),
                        new InstantAction(robot::intakeSamplePos),
                        strafeWithSubsystems(roadRunnerDrive.pose, yellowSpikeRight.plus(
                                calculateOffset(180, armExtentionLength))),
                        new InstantAction(robot::toggleIntake),

                        new InstantAction(() -> this.addTelemetryMessage("Attempting to retrieve Sample...")),
                        //Drive forward 5 inches to try to "sweep" and pick up the spike mark sample
                        roadRunnerDrive.actionBuilder(roadRunnerDrive.pose)
                                .strafeTo(yellowSpikeRight.plus(calculateOffset(180, armExtentionLength-5)))
                                .build(),

                        new InstantAction(() -> this.addTelemetryMessage("Depositing Sample...")),
                        new InstantAction(robot::depositSamplePosForward), //Set subsystems to prepare deposit
                        strafeWithSubsystems(roadRunnerDrive.pose, bucketPosition),
                        new InstantAction(robot::intakeBackward), //Deposits sample
                        new SleepAction(0.5), //Wait for intake to completely deposit



                        //Retrieve Middle Spike Mark
                        new InstantAction(() -> this.addTelemetryMessage("Driving to Spike Mark 2...")),
                        new InstantAction(robot::intakeSamplePos),
                        strafeWithSubsystems(roadRunnerDrive.pose, yellowSpikeMiddle.plus(
                                calculateOffset(180, armExtentionLength))),
                        new InstantAction(robot::toggleIntake),

                        new InstantAction(() -> this.addTelemetryMessage("Attempting to retrieve Sample...")),
                        //Drive forward 5 inches to try to "sweep" and pick up the spike mark sample
                        roadRunnerDrive.actionBuilder(roadRunnerDrive.pose)
                                .strafeTo(yellowSpikeMiddle.plus(calculateOffset(180, armExtentionLength-5)))
                                .build(),

                        new InstantAction(() -> this.addTelemetryMessage("Depositing Sample...")),
                        new InstantAction(robot::depositSamplePosForward), //Set subsystems to prepare deposit
                        strafeWithSubsystems(roadRunnerDrive.pose, bucketPosition),
                        new InstantAction(robot::intakeBackward), //Deposits sample
                        new SleepAction(0.5), //Wait for intake to completely deposit



                        //Retrieve Middle Spike Mark
                        new InstantAction(() -> this.addTelemetryMessage("Driving to Spike Mark 3...")),
                        new InstantAction(robot::intakeSamplePos),
                        strafeWithSubsystems(roadRunnerDrive.pose, yellowSpikeLeft.plus(
                                calculateOffset(180, armExtentionLength))),
                        new InstantAction(robot::toggleIntake),

                        new InstantAction(() -> this.addTelemetryMessage("Attempting to retrieve Sample...")),
                        //Drive forward 5 inches to try to "sweep" and pick up the spike mark sample
                        roadRunnerDrive.actionBuilder(roadRunnerDrive.pose)
                                .strafeTo(yellowSpikeLeft.plus(calculateOffset(180, armExtentionLength-5)))
                                .build(),

                        new InstantAction(() -> this.addTelemetryMessage("Depositing Sample...")),
                        new InstantAction(robot::depositSamplePosForward), //Set subsystems to prepare deposit
                        strafeWithSubsystems(roadRunnerDrive.pose, bucketPosition),
                        new InstantAction(robot::intakeBackward), //Deposits sample
                        new SleepAction(0.5), //Wait for intake to completely deposit

                        //Resets robot's subsystems to original position
                        new InstantAction(robot::lowerClimb),

                        //Go park in Ascent Zone
                        roadRunnerDrive.actionBuilder(roadRunnerDrive.pose)
                                .setTangent(Math.toRadians(90))
                                .splineTo(new Vector2d(-24,-12),0)
                                .build()
                )
        ));

    }

    public ParallelAction strafeWithSubsystems(Pose2d startPosition, Vector2d endPosition){
        return strafeWithSubsystems(startPosition, new Pose2d(endPosition,startPosition.heading));
    }

    public ParallelAction strafeWithSubsystems(Pose2d startPosition, Pose2d endPosition){
        return new ParallelAction(
                //Travel to place to deposit
                roadRunnerDrive.actionBuilder(startPosition)
                        .strafeToLinearHeading(endPosition.position.plus(
                                calculateOffset(180, armExtentionLength)
                        ), endPosition.heading)
                        .build(),
                robot.waitForLiftArmPID(3) //Ensure that subsystems are in the right position
        );
    }

    public void addTelemetryMessage(String message){
        telemetry.addLine(message);
        telemetry.update();
    }

    public static Vector2d calculateOffset(double angleDegrees, double distance){
        return new Vector2d(-distance * Math.cos(Math.toRadians(angleDegrees)), -distance * Math.sin(Math.toRadians(angleDegrees)));
    }
}
