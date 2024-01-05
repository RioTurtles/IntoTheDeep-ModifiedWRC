package org.firstinspires.ftc.teamcode.archive.v1;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="v1 TeleOp")
public class TeleOp_v1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware_v1 robot = new Hardware_v1();
        robot.init(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");  // debug message
        telemetry.update();
        robot.reset();  // Reset robot
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

            robot.motorFrontL.setPower(-(lx + ly + rot_x) / denominator);
            robot.motorBackL.setPower((-lx + ly + rot_x) / denominator);
            robot.motorFrontR.setPower((ly - lx - rot_x) / denominator);
            robot.motorBackR.setPower(-(lx + ly - rot_x) / denominator);

            // Reset IMU
            if (gamepad1.share) {
                robot.resetIMU();
            }

            // Slider height
            if (gamepad1.dpad_up) { // Scoring position
                robot.setSliderPosition(true);
            } else if (gamepad1.dpad_down) { // Intake position
                robot.setSliderPosition(false);
            }

            // Intake & Scoring servos
            if (gamepad1.right_bumper) {robot.startOuttake();}  // Right bumper to score
            else if (gamepad1.left_bumper) {robot.startIntake();}  // Left bumper for intake
            else {robot.stopIntake();}  // Stop both servos if none pressed

            // Intake Pitch
            if (gamepad1.dpad_left && robot.getSliderHeight() == 1) {robot.setIntakePitch(0);} // Scoring position
            else if (gamepad1.dpad_right && robot.getSliderHeight() == 0) {robot.setIntakePitch(1);} // Intake position

            telemetry.addData("Heading", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("SliderLeft", robot.motorSliderL.getCurrentPosition());
            telemetry.addData("SliderRight", robot.motorSliderR.getCurrentPosition());
            telemetry.addData("SliderHeight", robot.getSliderHeight());
            telemetry.update();
        }
    }
}
