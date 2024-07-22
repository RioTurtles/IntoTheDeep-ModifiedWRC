package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group="new")
public class RRPoseTunerRF extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d currentPose;
        drive.setPoseEstimate(new Pose2d(-40.33, -62.80, Math.toRadians(90.00)));
        waitForStart();

        while (opModeIsActive()) {
            currentPose = drive.getPoseEstimate();
            telemetry.addData("x", currentPose.getX());
            telemetry.addData("y", currentPose.getY());
            telemetry.addData("h(d)", Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("h(r)", currentPose.getHeading());
            telemetry.update();
            drive.update();
        }
    }
}
