package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Autonomous(name="v2 Autonomous R L")
public class AutonomousRedLeft_v2 extends LinearOpMode {
    public static int orientation = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware_v2 robot = new Hardware_v2();
        robot.init(hardwareMap, telemetry);
        robot.reset();
        double lx=0;
        double ly=0;
        double rot_x=0;
        double botHeading;
        double left_x;
        double left_y;
        double denominator;

        waitForStart();
        while (opModeIsActive()) {
/*
            botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            lx = left_x * Math.cos(-botHeading) - left_y * Math.sin(-botHeading);
            ly = left_x * Math.sin(-botHeading) + left_y * Math.cos(-botHeading);
            denominator = Math.max(Math.abs(left_x) + Math.abs(left_y) + Math.abs(rot_x), 1);
            robot.motorFL.setPower((lx + ly + rot_x) / denominator);
            robot.motorBL.setPower((-lx + ly + rot_x) / denominator);
            robot.motorFR.setPower((ly - lx - rot_x) / denominator);
            robot.motorBR.setPower((lx + ly - rot_x) / denominator);
        }
            if (gamepad1.triangle) {

                orientation += 1;
            }
            telemetry.addData("orientation", orientation);
            telemetry.update();

 */
        }
    }
}
