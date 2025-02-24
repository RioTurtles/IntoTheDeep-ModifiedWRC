package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous
public class AutonomousChamber extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Project2Hardware robot = new Project2Hardware(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        State state = State.INITIALISE;
        ElapsedTime timer1 = new ElapsedTime();

        Pose2d currentPose, startPose = new Pose2d(8.85, -62.87, Math.toRadians(270.00));
        Vector2d chamberPose = new Vector2d(0.47, -32.63);

        AtomicBoolean run1Async = new AtomicBoolean(false);

        TrajectorySequence initial = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(chamberPose)
                .build();

        Trajectory transition = null;
        TrajectorySequence push = null;

        robot.clawClose();

        telemetry.addData("Status", "Initialised");
        telemetry.update();

        waitForStart();
        drive.setPoseEstimate(startPose);
        robot.slider.setPower(1);
        timer1.reset();

        while (opModeIsActive()) {
            currentPose = drive.getPoseEstimate();

            if (state == State.INITIALISE) {
                if (!run1Async.get() && timer1.milliseconds() > 500) {
                    drive.followTrajectorySequenceAsync(initial);
                    run1Async.set(true);
                }

                robot.setArm(Project2Hardware.CHAMBER_ANGLE);
                if (robot.getArmError() <= 5 || timer1.milliseconds() > 1200) {
                    robot.setSlider(Project2Hardware.SLIDER_CHAMBER);
                    if ((robot.sliderInPosition(15) && !drive.isBusy())
                            || timer1.milliseconds() > 5350) {
                        state = State.SCORE_PRELOAD;
                        timer1.reset();
                        run1Async.set(false);
                    }
                }
            }

            if (state == State.SCORE_PRELOAD) {
                 robot.setSlider(Project2Hardware.SLIDER_CHAMBERED);
                 robot.setArm(Project2Hardware.CHAMBERED_ANGLE);

                 if (robot.sliderInPosition(15) || timer1.milliseconds() > 2500) {
                     robot.clawOpen();
                     state = State.PATH;
                     timer1.reset();
                 }

                transition = drive.trajectoryBuilder(currentPose)
                        .splineTo(new Vector2d(31.76, -36.42), Math.toRadians(0.00))
                        .splineTo(new Vector2d(32.21, -12.76), Math.toRadians(74.74))
                        .splineToSplineHeading(new Pose2d(46.66, -12.76, Math.toRadians(270.00)), Math.toRadians(270.00))
                        .build();
            }

            if (state == State.PATH) {
                if (timer1.milliseconds() > 300) {
                    if (!run1Async.get()) {drive.followTrajectoryAsync(transition); run1Async.set(true);}

                    robot.setSlider(0);
                    if (robot.sliderInPosition(15)) robot.setArm(0);

                    if (!drive.isBusy()) state = State.PUSH;
                }
                push = drive.trajectorySequenceBuilder(currentPose)
                        .lineToConstantHeading(new Vector2d(46.11, -57.10))
                        .lineToConstantHeading(new Vector2d(48.71, -12.76))
                        .splineTo(new Vector2d(55.42, -12.76), Math.toRadians(-1.44))
                        .lineToConstantHeading(new Vector2d(57.84, -57.10))
                        .lineToConstantHeading(new Vector2d(57.84, -12.76))
                        .splineToConstantHeading(new Vector2d(62.31, -12.76), Math.toRadians(-1.20))
                        .lineToConstantHeading(new Vector2d(62.31, -56.17))
                        .build();

            }

            if (state == State.PUSH) {
                drive.followTrajectorySequence(push);
                state = State.END;
            }


            drive.update();
            telemetry.addData("State", state);
            telemetry.update();
        }
    }

    enum State {
        INITIALISE,
        SCORE_PRELOAD,
        PATH,
        PUSH,
        END
    }
}
