package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="v2 TeleOp Simple")
public class TeleOp_v2_Simple extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Hardware_v2 robot = new Hardware_v2();
        robot.init(hardwareMap, telemetry);
        robot.reset();

        // Variables
        double bot_heading;
        double left_x;
        double left_y;
        double rot_x;
        double lx;
        double ly;
        double denominator;

        boolean clawUpperOpen = false;
        boolean clawLowerOpen = false;
        boolean isInScoringPosition = false;

        robot.setIntakePosition();

        waitForStart();

        while (opModeIsActive()) {
            // Fieldcentric controls
            bot_heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            left_x = gamepad1.left_stick_x;
            left_y = -gamepad1.left_stick_y;
            rot_x = gamepad1.right_stick_x * -0.8;  // Make rotation less intense (80%)
            lx = left_x * Math.cos(-bot_heading) - left_y * Math.sin(-bot_heading);
            ly = left_x * Math.sin(-bot_heading) + left_y * Math.cos(-bot_heading);
            denominator = Math.max(Math.abs(left_x) + Math.abs(left_y) + Math.abs(rot_x), 1);
            robot.motorFL.setPower((lx + ly + rot_x) / denominator);
            robot.motorBL.setPower((-lx + ly + rot_x) / denominator);
            robot.motorFR.setPower((ly - lx - rot_x) / denominator);
            robot.motorBR.setPower((lx + ly - rot_x) / denominator);

            if (gamepad1.left_bumper) {robot.servoClawUpper.setPosition(0);}
            if (gamepad1.right_bumper) {robot.servoClawLower.setPosition(1);}
            if (gamepad1.left_trigger > 0) {robot.servoClawUpper.setPosition(0.22);}
            if (gamepad1.right_trigger > 0) {robot.servoClawLower.setPosition(0.74);}

            if (gamepad1.triangle) {
                robot.servoArmRight.setPosition(1);
                robot.servoArmLeft.setPosition(1);
                robot.setSliderPosition(1);
            }
            if (gamepad1.circle) {robot.setScoringPosition();}
            if (gamepad1.cross) {robot.setIntakePosition();}
            if (gamepad1.square) {robot.setSliderPosition(0);}

            // Reset IMU.
            if (gamepad1.share) {robot.imu.resetYaw();}

            telemetry.update();
        }
    }
}