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
    Objective objective = Objective.INITIALISE;

    @Override
    public void runOpMode() throws InterruptedException {
        Project2Hardware robot = new Project2Hardware(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ElapsedTime autonomous = new ElapsedTime();
        ElapsedTime timer1 = new ElapsedTime();
        Pose2d startPose = new Pose2d(-38.49, -62.66, Math.toRadians(270.00));
        Pose2d basketPose = new Pose2d(-56.88, -56.88, Math.toRadians(45.00));
        Pose2d currentPose;
        AtomicBoolean run = new AtomicBoolean(false);

        Pose2d[] poses = {
                new Pose2d(-48.28, -44.40, Math.toRadians(270.00)),
                new Pose2d(-59.98, -44.40, Math.toRadians(270.00)),
                new Pose2d(-58.86, -39.14, Math.toRadians(315.00)),
                new Pose2d(-35.58, -13.24, Math.toRadians(0.00))  // Unused pose, failsafe
        };

        int cycles = 0;

        telemetry.addData("Status", "Initialised");
        telemetry.update();

        drive.setPoseEstimate(startPose);

        TrajectorySequence preload = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> robot.setArm(Project2Hardware.BASKET_ANGLE))
                .back(5)
                .lineToSplineHeading(basketPose)
                .addTemporalMarker(() -> {
                    if (robot.getArmError() <= 3)
                        robot.setSlider(1400);
                    run.set(true);
                })
                .build();

        TrajectorySequence pathPickup = null, pathReturn = null;
        Trajectory park;

        waitForStart();
        autonomous.reset();
        robot.clawClose();
        robot.arm.setPower(1);

        while (opModeIsActive()) {
            currentPose = drive.getPoseEstimate();

            if (objective == Objective.INITIALISE) {
                if (!run.get()) drive.followTrajectorySequence(preload);
                if (autonomous.milliseconds() > 5000) {
                    run.set(false);
                    objective = Objective.SCORE_PRELOAD;
                    timer1.reset();
                } else {
                    robot.drivetrain.remote(0.04, 0, 0, 0);
                }
            }

            else if (objective == Objective.SCORE_PRELOAD) {
                if (timer1.milliseconds() > 300) {
                    robot.setSlider(0);
                    if (robot.sliderInPosition(5)) {
                        robot.setArm(0);
                        if (robot.getArmError() <= 3) {
                            objective = Objective.PATH_TO_PICKUP;
                            timer1.reset();
                        }
                    }
                }
                else robot.clawOpen();

                pathPickup = drive.trajectorySequenceBuilder(currentPose)
                        .lineToSplineHeading(poses[0])
                        .addTemporalMarker(() -> {robot.setSlider(560); timer1.reset();})
                        .addTemporalMarker(() -> run.set(true))
                        .build();
            }

            else if (objective == Objective.PATH_TO_PICKUP) {
                robot.arm.setPower(0);
                if (!run.get()) drive.followTrajectorySequence(pathPickup);
                if (robot.sliderInPosition(5) || timer1.milliseconds() > 1000) {
                    robot.clawClose();
                    objective = Objective.PATH_TO_BASKET;
                    timer1.reset();
                    run.set(false);
                }

                pathReturn = drive.trajectorySequenceBuilder(currentPose)
                        .addTemporalMarker(() -> robot.setArm(Project2Hardware.BASKET_ANGLE))
                        .lineToSplineHeading(basketPose)
                        .addTemporalMarker(() -> {robot.setSlider(1400); timer1.reset();})
                        .build();
            }

            else if (objective == Objective.PATH_TO_BASKET) {
                if (timer1.milliseconds() > 300) {
                    robot.setSlider(0);
                    if (robot.sliderInPosition(5) || timer1.milliseconds() > 8000) {
                        drive.followTrajectorySequence(pathReturn);
                        objective = Objective.SCORE;
                        timer1.reset();
                    }
                }
            }

            else if (objective == Objective.SCORE) {
                if (robot.sliderInPosition(5) || timer1.milliseconds() > 8000) {
                    robot.clawOpen();
                    timer1.reset();
                    objective = Objective.RETURN;
                }
            }

            else if (objective == Objective.RETURN) {
                if (timer1.milliseconds() > 300) {
                    cycles++;
                    robot.setSlider(0);
                    if (robot.sliderInPosition(5) || autonomous.milliseconds() > 6300) {
                        robot.setArm(0);
                        if (robot.getArmError() <= 3 || autonomous.milliseconds() > 8000) {
                            if (cycles < 3) objective = Objective.PATH_TO_PICKUP;
                            else objective = Objective.PARK;
                            timer1.reset();
                        }
                    }

                    pathPickup = drive.trajectorySequenceBuilder(currentPose)
                            .lineToSplineHeading(poses[cycles])
                            .addTemporalMarker(() -> {robot.setSlider(1000); timer1.reset();})
                            .build();
                }
                else robot.clawOpen();
            }

            else if (objective == Objective.PARK) {
                park = drive.trajectoryBuilder(currentPose)
                        .lineToSplineHeading(new Pose2d(-35.58, -13.24, Math.toRadians(0.00)))
                        .addTemporalMarker(0.6,  0, () -> {
                            robot.arm.setPower(1);
                            robot.setArm(90);
                        })
                        .splineToConstantHeading(new Vector2d(-23.94, -12.95), Math.toRadians(0.00))
                        .build();
                drive.followTrajectory(park);
                objective = Objective.END;
            }

            drive.update();
            telemetry.addData("Objective", objective);
            telemetry.addLine();
            telemetry.addData("X", currentPose.getX());
            telemetry.addData("Y", currentPose.getY());
            telemetry.addData("H(D)", Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("H(R)", currentPose.getHeading());
            telemetry.update();
        }
    }

    enum Objective {
        INITIALISE,
        PATH_TO_PRELOAD,
        SCORE_PRELOAD,
        // ---
        PATH_TO_PICKUP,
        PATH_TO_BASKET,
        SCORE,
        RETURN,
        // ---
        PARK,
        END
    }
}
