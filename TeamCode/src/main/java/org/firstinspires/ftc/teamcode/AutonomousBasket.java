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
public class AutonomousBasket extends LinearOpMode {
    State state = State.INITIALISE;

    @Override
    public void runOpMode() throws InterruptedException {
        Project2Hardware robot = new Project2Hardware(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ElapsedTime autonomous = new ElapsedTime();
        ElapsedTime timer1 = new ElapsedTime();
        Pose2d startPose = new Pose2d(-38.49, -62.66, Math.toRadians(270.00));
        Pose2d basketPose = new Pose2d(-56.58, -58.58, Math.toRadians(45.00));
        Pose2d currentPose;

        state = State.INITIALISE;

        AtomicBoolean run1 = new AtomicBoolean(false);
        AtomicBoolean run2 = new AtomicBoolean(false);
        AtomicBoolean run3Async = new AtomicBoolean(false);

        Pose2d[] poses = {
                new Pose2d(-48.28, -44.40, Math.toRadians(270.00)),
                new Pose2d(-58.98, -44.40, Math.toRadians(270.00)),
                new Pose2d(-52.36, -26.14, Math.toRadians(0.00)),
                new Pose2d(-35.58, -13.24, Math.toRadians(0.00))  // Unused pose, failsafe
        };

        int[] sliders = {420, 450, 410, 315};  // Last value is unused; failsafe

        int cycles = 0;

        telemetry.addData("Status", "Initialised");
        telemetry.update();

        drive.setPoseEstimate(startPose);

        TrajectorySequence preloadArm = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> robot.setArm(Project2Hardware.BASKET_ANGLE))
                .back(5)
                .addTemporalMarker(() -> run1.set(true))
                .build();

        TrajectorySequence preloadSlider = null, pathPickup = null, pathReturn = null;
        Trajectory parkCont;

        waitForStart();
        autonomous.reset();
        robot.clawClose();
        robot.arm.setPower(1);

        while (opModeIsActive()) {
            currentPose = drive.getPoseEstimate();

            if (state == State.INITIALISE) {
                preloadSlider = drive.trajectorySequenceBuilder(currentPose)
                        .lineToSplineHeading(basketPose)
                        .build();

                if (!run1.get()) drive.followTrajectorySequence(preloadArm);
                if (robot.getArmError() <= 5 || autonomous.milliseconds() > 2000) {
                    robot.setSlider(1400);
                    run1.set(false);
                    state = State.INITIALISE_SLIDER;
                    timer1.reset();
                }
            }

            else if (state == State.INITIALISE_SLIDER) {
                drive.followTrajectorySequence(preloadSlider);
                robot.clawOpen();
                state = State.SCORE_PRELOAD;
                timer1.reset();
            }

            else if (state == State.SCORE_PRELOAD) {
                TrajectorySequence forward = drive.trajectorySequenceBuilder(currentPose)
                        .forward(3.5)
                        .addTemporalMarker(() -> run2.set(true))
                        .build();

                pathPickup = drive.trajectorySequenceBuilder(forward.end())
                        .lineToSplineHeading(poses[0])
                        .addTemporalMarker(() -> {robot.setSlider(sliders[0]); timer1.reset();})
                        .addTemporalMarker(() -> run1.set(true))
                        .addTemporalMarker(1.0, 0.0, () -> run3Async.set(false))
                        .build();

                if (timer1.milliseconds() > 300) {
                    if (!run2.get()) {
                        drive.followTrajectorySequence(forward);
                        robot.setSlider(0);
                        run3Async.set(false);
                    }

                    if (!run3Async.get()) {
                        drive.followTrajectorySequenceAsync(pathPickup);
                        run3Async.set(true);
                    }

                    if (robot.sliderInPosition(5) || timer1.milliseconds() > 5000) {
                        robot.setArm(0);
                        if (robot.getArmError() <= 3 || timer1.milliseconds() > 6500) {
                            state = State.PATH_TO_PICKUP;
                            timer1.reset();
                        }
                    }
                }
            }

            else if (state == State.PATH_TO_PICKUP) {
                robot.arm.setPower(0);
                if (!run1.get()) drive.followTrajectorySequence(pathPickup);
                if (robot.sliderInPosition(5) || timer1.milliseconds() > 1000) {
                    // Intake from ground; adjust if needed
                    if (cycles != 2) robot.clawClose(); else robot.clawCloseRare();
                    run1.set(false);
                    state = State.CYCLE_TRANSITION;
                    timer1.reset();
                }

                pathReturn = drive.trajectorySequenceBuilder(currentPose)
                        .lineToSplineHeading(basketPose)
                        .addTemporalMarker(() -> {robot.setSlider(1400); timer1.reset();})
                        .build();
            }

            else if (state == State.CYCLE_TRANSITION) {
                if (timer1.milliseconds() > 300) {
                    robot.setSlider(0);
                    if (robot.sliderInPosition(5) || timer1.milliseconds() > 2400) {
                        robot.setArm(Project2Hardware.BASKET_ANGLE);
                        robot.setSlider(1400);
                        state = State.PATH_TO_BASKET;
                        timer1.reset();
                    }
                }
            }

            else if (state == State.PATH_TO_BASKET) {
                run1.set(false);
                run2.set(false);
                run3Async.set(false);
                drive.followTrajectorySequence(pathReturn);
                state = State.SCORE;
            }

            else if (state == State.SCORE) {
                if (robot.sliderInPosition(5) || timer1.milliseconds() > 500) {
                    robot.clawOpen();
                    timer1.reset();
                    state = State.RETRACT;
                }
            }

            else if (state == State.RETRACT) {
                TrajectorySequence forward = drive.trajectorySequenceBuilder(currentPose)
                        .forward(5)
                        .build();

                if (timer1.milliseconds() > 300) {
                    drive.followTrajectorySequence(forward);
                    state = State.RETURN;

                    cycles++;
                    int finalCycles = cycles;
                    pathPickup = drive.trajectorySequenceBuilder(forward.end())
                            .lineToSplineHeading(poses[cycles])
                            .addTemporalMarker(() -> robot.setSlider(sliders[finalCycles]))
                            .addTemporalMarker(() -> run1.set(true))
                            .build();
                }
                else robot.clawOpen();
            }

            else if (state == State.RETURN) {
                if (!run3Async.get()) {
                    drive.followTrajectorySequenceAsync(pathPickup);
                    run3Async.set(true);
                }


                if (cycles < 3) {
                    robot.setSlider(0);
                    if (robot.sliderInPosition(5) || timer1.milliseconds() > 4000) {
                        robot.setArm(0);
                        if (robot.getArmError() <= 3 || timer1.milliseconds() > 6000) {
                            state = State.RETURN_SLIDER_RESTORE;
                            timer1.reset();
                            run3Async.set(false);
                        }
                    }
                } else {
                    robot.setSlider(200);
                    state = State.PARK;
                    timer1.reset();
                    run3Async.set(false);
                }
            }

            else if (state == State.RETURN_SLIDER_RESTORE) {
                robot.setSlider(sliders[cycles]);
                if (robot.sliderInPosition(5) || timer1.milliseconds() > 1000) {
                    robot.clawClose();
                    state = State.CYCLE_TRANSITION;
                    timer1.reset();
                }
            }

            else if (state == State.PARK) {
                parkCont = drive.trajectoryBuilder(currentPose)
                        .lineToSplineHeading(new Pose2d(-33.58, -13.24, Math.toRadians(0.00)))
                        .splineToConstantHeading(new Vector2d(-21.94, -12.95), Math.toRadians(0.00))
                        .addTemporalMarker(1.0, 0.0, () -> robot.arm.setPower(0))
                        .build();

                TrajectorySequence park = drive.trajectorySequenceBuilder(currentPose)
                        .addTemporalMarker(() -> {
                            robot.arm.setPower(1);
                            robot.setArm(100);
                            robot.setSlider(200);
                        })
                        .addTrajectory(parkCont)
                        .build();

                drive.followTrajectorySequence(park);
                state = State.END;
            }

            drive.update();
            telemetry.addData("Objective", state);
            telemetry.addLine();
            telemetry.addData("X", currentPose.getX());
            telemetry.addData("Y", currentPose.getY());
            telemetry.addData("H(D)", Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("H(R)", currentPose.getHeading());
            telemetry.update();
        }

        drive.setWeightedDrivePower(new Pose2d());
        PoseStorage.pose = drive.getPoseEstimate();
    }

    enum State {
        INITIALISE,
        INITIALISE_SLIDER,
        PATH_TO_PRELOAD,
        SCORE_PRELOAD,
        // ---
        PATH_TO_PICKUP,
        CYCLE_TRANSITION,
        PATH_TO_BASKET,
        SCORE,
        RETRACT,
        RETURN,
        RETURN_SLIDER_RESTORE,
        // ---
        PARK,
        END
    }
}
