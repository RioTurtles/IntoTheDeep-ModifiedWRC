package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="v1 TeleOp")
public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware robot = new Hardware();
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");  // Robot finishes initialization. Here we output a debug message.
        telemetry.update();
        robot.reset();  // Reset robot?
        waitForStart();

        while (opModeIsActive()) {
            // Fieldcentric
            double bot_heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double left_x = gamepad1.left_stick_x;
            double left_y = -gamepad1.left_stick_y;
            double rot_x = gamepad1.right_stick_x*0.6;  // Make rotation less intense (60%)
            double lx = left_x * Math.cos(-bot_heading) - left_y * Math.sin(-bot_heading);
            double ly = left_x * Math.sin(-bot_heading) + left_y * Math.cos(-bot_heading);
            double denominator = Math.max(Math.abs(left_x) + Math.abs(left_y) + Math.abs(rot_x), 1);

            robot.motorFrontLeft.setPower(-(lx + ly + rot_x) / denominator);
            // The below 2 have been reversed.
            robot.motorBackLeft.setPower(-(-lx + ly + rot_x) / denominator);
            robot.motorFrontRight.setPower(-(ly - lx - rot_x) / denominator);
            // The above 2 have been reversed.
            robot.motorBackRight.setPower(-(lx + ly - rot_x) / denominator);

            if (gamepad1.options) {
                robot.resetIMU();
            }

            if (gamepad1.circle) {
                robot.servoTest.setPower(robot.servoTest.getPower() + 50);
                telemetry.addData("servoPower", robot.servoTest.getPower());
            }
            if (gamepad1.square) {
                robot.servoTest.setPower(robot.servoTest.getPower() - 50);
                telemetry.addData("servoPower", robot.servoTest.getPower());
            }

            telemetry.update();
        }
    }
}
