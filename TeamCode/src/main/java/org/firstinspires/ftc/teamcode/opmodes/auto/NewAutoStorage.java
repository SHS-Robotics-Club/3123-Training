package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.util.VisionToLiftHeight.getPosition;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.RoadRunnerHelper.inchesToCoordinate;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoCarousel;
import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoLift;
import org.firstinspires.ftc.teamcode.core.robot.vision.robot.TseDetector;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class NewAutoStorage extends LinearOpMode {
    public int multiplier = 1;
    public boolean isRed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry goodTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        EventThread eventThread = new EventThread(() -> !isStopRequested());

        AutoCarousel carousel = new AutoCarousel(hardwareMap, multiplier);
        AutoLift lift = new AutoLift(eventThread, hardwareMap);

        TseDetector detector = new TseDetector(hardwareMap, "webcam", true, isRed);
        int height;

        final Pose2d initial = new Pose2d(-47, multiplier * (70 - inchesToCoordinate(9)),
                Math.toRadians(90 * multiplier));

        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);
        drive.setPoseEstimate(initial);

        final boolean[] liftUpdated = {false};
        Thread liftThread = new Thread(() -> {
            while (!isStopRequested()) {
                lift.update();
                liftUpdated[0] = true;
            }
        });

        // Part 1: go to shipping hub
        final Trajectory part1;
        {
            TrajectoryBuilder builder = drive.trajectoryBuilder(initial);
            builder.lineTo(new Vector2d(-40, multiplier == 1 ? 55 : -53));
            builder.splineToLinearHeading(new Pose2d(multiplier == 1 ? -21 : -20,
                    multiplier == 1 ? 42 : -38, Math.toRadians(multiplier == 1 ? 100 : -95)),
                    Math.toRadians(-110 * multiplier));
            part1 = builder.build();
        }

        // Part 2: carousel
        final Trajectory part2;
        {
            TrajectoryBuilder builder = drive.trajectoryBuilder(part1.end());
            builder.lineTo(new Vector2d(-19, 50 * multiplier));
            builder.lineToLinearHeading(new Pose2d(multiplier == 1 ? -60 : -60.5,
                    multiplier == 1 ? 58 : -58.5, Math.toRadians(multiplier == 1 ? 240 : 330)));
            part2 = builder.build();
        }

        ElapsedTime carouselTimer = new ElapsedTime();

        // Part 3: Park in Alliance Storage Unit
        final Trajectory part3;
        {
            TrajectoryBuilder builder = drive.trajectoryBuilder(part2.end());
            builder.lineToLinearHeading(new Pose2d(-60, 35 * multiplier,
                    Math.toRadians(90 * multiplier)));
            part3 = builder.build();
        }

        waitForStart();
        liftThread.start();
        eventThread.start();

        height = detector.run();
        goodTelemetry.addData("height", height);
        goodTelemetry.update();

        // Part 1
        drive.followTrajectoryAsync(part1);
        updateLoop(drive);
        if (isStopRequested()) return;

        liftUpdated[0] = false;
        lift.setPosition(getPosition(height));
        while (!isStopRequested() && (!liftUpdated[0] || lift.getState() != AutoLift.MovementStates.NONE)) {
            drive.update();
        }
        if (isStopRequested()) return;

        // Part 2
        drive.followTrajectoryAsync(part2);
        updateLoop(drive);
        if (isStopRequested()) return;

        carousel.on();
        carouselTimer.reset();
        while (!isStopRequested() && carouselTimer.seconds() < 4) {
            drive.update();
        }
        if (isStopRequested()) return;
        carousel.off();

        // Part 3
        drive.followTrajectoryAsync(part3);
        updateLoop(drive);
    }

    public void updateLoop(SampleMecanumDrive drive) {
        while (!isStopRequested() && drive.isBusy()) {
            drive.update();
        }
    }
}
