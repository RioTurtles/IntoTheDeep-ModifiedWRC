package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//This uses MY code last time, bugs are expected
@TeleOp
public class RedAutoTruss extends LinearOpMode {
    SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Good evening!");
        telemetry.update();

        waitForStart();
        drivetrain.setPoseEstimate(new Pose2d(-37.85, -58.85, Math.toRadians(90.0)));
        while(opModeIsActive()) {
            Pose2d currentPosition = drivetrain.getPoseEstimate();
            TrajectorySequence untitled0 = drivetrain.trajectorySequenceBuilder(new Pose2d(-47.77, 27.92, Math.toRadians(270.00)))
                    .splineToSplineHeading(new Pose2d(-32.54, -36.23, Math.toRadians(180.00)), Math.toRadians(-10.00))
                    .lineTo(new Vector2d(39.46, -36.00))
                    .build();

            if (gamepad1.triangle){
                drivetrain.followTrajectorySequence(untitled0);
            }

            drivetrain.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x
            ));
            drivetrain.update();
        }
    }
}

//Finally, build success