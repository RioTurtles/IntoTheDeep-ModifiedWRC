package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="AutoBackdrop, DistanceSensor")
public class AutoBackdropDistance extends LinearOpMode {
    Hardware_v3 robot;
    double Kp = 0.012;
    double target = 10;
    double imuKp = 0.8;

    @Override
    public void runOpMode() throws InterruptedException{ //backup, only uses Kp
        robot = new Hardware_v3();
        robot.init(hardwareMap, telemetry);
        robot.reset();
        waitForStart();
        robot.imu.resetYaw();

        while (opModeIsActive()) {
            double distance = robot.distanceSensor.getDistance(DistanceUnit.CM);
            double heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            telemetry.addData("Distance", distance);
            telemetry.addData("Error", Math.abs(target - distance));

            if (gamepad1.square) {
                double error = Math.abs(target - distance);
                if (error > 3 && distance >= target) {
                    robot.remote(error * Kp, 0, -heading * imuKp, heading);
                }
            } else {
                robot.remote(-gamepad1.left_stick_y, gamepad1.left_stick_x,-gamepad1.right_stick_x * 0.8, heading);
            }

            if (gamepad1.options) {
                robot.imu.resetYaw();
            }


            telemetry.update();
        }
    }
}