package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="PoseReset")
public class PoseStorage extends LinearOpMode {
    public static Pose2d pose = new Pose2d(0, 0, 0);
    public static final Pose2d park = new Pose2d(-21.94, -12.95, Math.toRadians(0.00));

    @Override
    public void runOpMode() {
        pose = new Pose2d(0, 0, 0);
        telemetry.addLine("Pose reset to (0, 0).");
        telemetry.addLine("Play to set pose to park.");
        telemetry.update();
        waitForStart();
        pose = park;
    }
}
