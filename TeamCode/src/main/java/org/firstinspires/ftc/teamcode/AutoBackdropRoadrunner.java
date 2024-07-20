package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp(name="AutoBackdrop, Roadrunner")
public class AutoBackdropRoadrunner extends LinearOpMode {
    SampleMecanumDriveCancelable drive;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDriveCancelable(hardwareMap);
        Pose2d poseEstimate;
        Position selected = Position.CENTRE;
        Control state = Control.DRIVER;

        waitForStart();
        drive.setPoseEstimate(new Pose2d(-37.098, -63.110, Math.toRadians(90.00)));
        while (opModeIsActive()) {
            poseEstimate = drive.getPoseEstimate();

            if (state == Control.DRIVER) {
                drive.setWeightedDrivePower(new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x * 0.8
                ));

                if (gamepad1.square) selected = Position.LEFT;
                else if (gamepad1.triangle) selected = Position.CENTRE;
                else if (gamepad1.circle) selected = Position.RIGHT;

                if (gamepad1.cross) {
                    telemetry.addData("Selected position", selected);
                    drive.followTrajectorySequenceAsync(getTrajectorySequence(poseEstimate, selected));
                    state = Control.AUTOMATIC;
                }
            }

            if (state == Control.AUTOMATIC) {
                if (gamepad1.right_bumper) {
                    drive.breakFollowing();
                    state = Control.DRIVER;
                }

                if (!drive.isBusy()) state = Control.DRIVER;
            }

            drive.update();
            telemetry.addData("Control", state);
            telemetry.addData("Selected position", selected);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("r", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    private TrajectorySequence getTrajectorySequence(Pose2d start, Position position) {
        switch (position) {
            case LEFT:
                return drive.trajectorySequenceBuilder(start)
                        .splineToSplineHeading(new Pose2d(-36.95, -35.50, Math.toRadians(180.00)), Math.toRadians(0), SampleMecanumDriveCancelable.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(10, -35.5), Math.toRadians(0), SampleMecanumDriveCancelable.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .splineToConstantHeading(new Vector2d(45.30, -29.00), Math.toRadians(0))
                        .build();

            case RIGHT:
                return drive.trajectorySequenceBuilder(start)
                        .splineToSplineHeading(new Pose2d(-36.95, -35.50, Math.toRadians(180.00)), Math.toRadians(0), SampleMecanumDriveCancelable.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(10, -35.5), Math.toRadians(0), SampleMecanumDriveCancelable.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .splineToConstantHeading(new Vector2d(45.30, -46.50), Math.toRadians(0))
                        .build();

            case CENTRE:
            default:
                return drive.trajectorySequenceBuilder(start)
                        .splineToSplineHeading(new Pose2d(-36.95, -35.50, Math.toRadians(180.00)), Math.toRadians(0), SampleMecanumDriveCancelable.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(10, -35.5), Math.toRadians(0), SampleMecanumDriveCancelable.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .splineToConstantHeading(new Vector2d(45.30, -37.50), Math.toRadians(0))
                        .build();
        }
    }

    private enum Position {
        LEFT,
        CENTRE,
        RIGHT,
    }

    private enum Control {
        DRIVER,
        AUTOMATIC
    }
}
