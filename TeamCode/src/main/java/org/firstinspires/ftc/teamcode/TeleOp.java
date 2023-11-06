package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="v1 TeleOp")
public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware robot = new Hardware();
        robot.init(hardwareMap);

        // Variables
        int sliderHeight = 0;

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

            if (gamepad1.options) {  // Reset IMU
                robot.resetIMU();
            }

            if (gamepad1.right_bumper) {
                // Control arm with right stick and intake pitch (vertical) with left stick when R1
                robot.servoArmRight.setPosition(robot.servoArmRight.getPosition() + right_y / 10);
                robot.servoArmLeft.setPosition(robot.servoArmLeft.getPosition() + right_y / 10);
                robot.servoIntakePitchLeft.setPosition(robot.servoIntakePitchLeft.getPosition() + left_y / 10);
                robot.servoIntakePitchRight.setPosition(robot.servoIntakePitchRight.getPosition() + left_y / 10);
            } else {
                robot.motorFrontLeft.setPower((lx + ly + rot_x) / denominator);
                robot.motorBackLeft.setPower((-lx + ly + rot_x) / denominator);
                robot.motorFrontRight.setPower((ly - lx - rot_x) / denominator);
                robot.motorBackRight.setPower((lx + ly - rot_x) / denominator);
            }

            if (gamepad1.right_trigger >= Constants.rightTriggerFullyDown) {
                // R2, open intake
                robot.servoIntakeLeft.setPower(-gamepad1.right_trigger);
                robot.servoIntakeRight.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger >= Constants.leftTriggerFullyDown) {
                // L2, close intake
                robot.servoIntakeLeft.setPower(gamepad1.left_trigger);
                robot.servoIntakeRight.setPower(-gamepad1.left_trigger);
            }

            if (gamepad1.dpad_left) {
                // dpad left, extend intake
                robot.servoArmLeft.setPosition(Constants.armLeftExtended);
                robot.servoArmRight.setPosition(Constants.armRightExtended);
            } else if (gamepad1.dpad_right) {
                // dpad right, retract intake
                robot.servoArmLeft.setPosition(Constants.armLeftRetracted);
                robot.servoArmRight.setPosition(Constants.armRightRetracted);
            }

            if (gamepad1.dpad_down) {
                // dpad down, intake position
                if (sliderHeight == 2) {
                    robot.motorHeightLeft.setTargetPosition(Constants.heightLeftHigh);
                    robot.motorHeightRight.setTargetPosition(Constants.heightRightHigh);
                    robot.servoIntakePitchLeft.setPosition(Constants.intakePitchLeftGrab);
                    robot.servoIntakePitchRight.setPosition(Constants.intakePitchRightGrab);
                    sliderHeight = 1; }
            } else if (gamepad1.dpad_up) {
                // dpad up, scoring position
                if (sliderHeight == 1) {
                    robot.motorHeightLeft.setTargetPosition(Constants.heightLeftDown);
                    robot.motorHeightRight.setTargetPosition(Constants.heightRightDown);
                    robot.servoIntakePitchLeft.setPosition(Constants.intakePitchLeftScore);
                    robot.servoIntakePitchRight.setPosition(Constants.intakePitchRightScore);
                    sliderHeight = 2; }
            } else if (gamepad1.square) {
                // square, max height
                robot.motorHeightLeft.setTargetPosition(Constants.maxLeftHeight);
                robot.motorHeightRight.setTargetPosition(Constants.maxRightHeight);
            } else if (gamepad1.circle) {
                // circle, min height
                robot.motorHeightLeft.setTargetPosition(Constants.minLeftHeight);
                robot.motorHeightRight.setTargetPosition(Constants.minRightHeight);
            }

            if (gamepad1.triangle) {
                // launch drone
                robot.servoDroneLauncher.setPosition(Constants.droneRelease);
            }

            telemetry.update();
        }
    }
}
