package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

@Disabled
@TeleOp(name="v3 TeleOp A3R")
public class TeleOp_v3a3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware_v3 robot = new Hardware_v3();
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        robot.init(hardwareMap, telemetry);
        robot.reset();

        // Variables
        final double ROTATION_MULTIPLIER = 0.8;
        final double BRAKE_MODE_MULTIPLIER = 0.4;

        int stage = 0;
        ControlMode controlState = ControlMode.DRIVER_CONTROL;

        drive.setPoseEstimate(PoseStorage.getCurrentPose());
        waitForStart();

        while (opModeIsActive()) {
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d fieldcentricInput = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x).rotated(-poseEstimate.getHeading());

            switch (controlState) {
                case DRIVER_CONTROL:
                    if (gamepad1.square) {
                        fieldcentricInput = new Vector2d(0, 0).rotated(-poseEstimate.getHeading());
                        Trajectory autoBackdropLeft = drive.trajectoryBuilder(poseEstimate)
                                .lineToSplineHeading(new Pose2d(51.00, -29.75, Math.toRadians(180.00)))
                                .build();
                        telemetry.addLine("Pathing to backdrop...");
                        drive.followTrajectoryAsync(autoBackdropLeft);
                        controlState = ControlMode.AUTOMATIC_CONTROL;
                    }
                    break;

                case AUTOMATIC_CONTROL:
                    if (gamepad1.cross) {
                        drive.breakFollowing();
                        controlState = ControlMode.DRIVER_CONTROL;
                    }
                    break;
            }

            telemetry.addLine(stateToTelemetry(stage));
            telemetry.addData("ControlState", controlState);
            telemetry.update();

            // Fieldcentric controls
            drive.setWeightedDrivePower(new Pose2d(fieldcentricInput.getX(), fieldcentricInput.getY(), -gamepad1.right_stick_x * ROTATION_MULTIPLIER));
        }
    }

    private String stateToTelemetry(int state) {
        String result;
        switch (state) {
            case 0: result = "Intake"; break;
            case 1: result = ""; break;
            default: result = "INVALID";
        }
        return state + " - " + result;
    }

    private enum ControlMode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }
}