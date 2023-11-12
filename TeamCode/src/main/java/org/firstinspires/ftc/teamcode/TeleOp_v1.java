package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="v1 TeleOp")
public class TeleOp_v1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware_v1 robot = new Hardware_v1();
        robot.init(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");  // Robot finishes initialization. Here we output a debug message.
        telemetry.update();
        robot.reset();  // Reset robot?
        waitForStart();
        telemetry.addData("Status", "Running");
        telemetry.update();

        while (opModeIsActive()) {
            // Fieldcentric
            double bot_heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double left_x = gamepad1.left_stick_x;
            double left_y = -gamepad1.left_stick_y;
            double rot_x = gamepad1.right_stick_x * 0.8;  // Make rotation less intense (80%)
            double lx = left_x * Math.cos(-bot_heading) - left_y * Math.sin(-bot_heading);
            double ly = left_x * Math.sin(-bot_heading) + left_y * Math.cos(-bot_heading);
            double denominator = Math.max(Math.abs(left_x) + Math.abs(left_y) + Math.abs(rot_x), 1);

            robot.motorFrontLeft.setPower(-(lx + ly + rot_x) / denominator);
            robot.motorBackLeft.setPower((-lx + ly + rot_x) / denominator);
            robot.motorFrontRight.setPower((ly - lx - rot_x) / denominator);
            robot.motorBackRight.setPower(-(lx + ly - rot_x) / denominator);

            // Reset IMU
            if (gamepad1.share) {
                robot.resetIMU();
            }

            // Slider height
            if (gamepad1.square) {
                robot.setSliderPosition(true);
            } else if (gamepad1.circle) {
                robot.setSliderPosition(false);
            }

            // Intake & Scoring
            if (gamepad1.left_bumper && gamepad1.right_bumper) {robot.startOuttake();}  // Both bumpers to score
            else if (gamepad1.left_bumper) {robot.startIntake();}  // Single bumper (left) for intake
            else {robot.stopIntake();}  // Stop both servos

            telemetry.addData("heading", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("sliderLeft", robot.motorSliderLeft.getCurrentPosition());
            telemetry.addData("sliderRight", robot.motorSliderRight.getCurrentPosition());
            telemetry.addData("position(p)", robot.sliderPosition);
            telemetry.addData("L-POWER", robot.motorSliderLeft.getPower());
            telemetry.addData("R-POWER", robot.motorSliderRight.getPower());
            telemetry.update();
        }
    }
}
