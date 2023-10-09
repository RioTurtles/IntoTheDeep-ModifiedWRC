package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleOp_v1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware_v1 robot = new Hardware_v1(hardwareMap);

        // Variables.
        boolean leftArmExtended = false;
        boolean rightArmExtended = false;
        boolean rigged = false;

        telemetry.addData("Status", "Initialized");
        robot.reset();
        waitForStart();

        while (opModeIsActive()) {
            double bot_heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double left_x = gamepad1.left_stick_x;
            double left_y = -gamepad1.left_stick_y;
            double rot_x = gamepad1.right_stick_x;
            double lx = left_x * Math.cos(-bot_heading) - left_y * Math.sin(-bot_heading);
            double ly = left_x * Math.sin(-bot_heading) + left_y * Math.cos(-bot_heading);
            double denominator = Math.max(Math.abs(left_x) + Math.abs(left_y) + Math.abs(rot_x), 1);

            robot.motorFrontLeft.setPower((lx + ly + rot_x) / denominator);
            robot.motorBackLeft.setPower((-lx + ly + rot_x) / denominator);
            robot.motorFrontRight.setPower((ly - lx - rot_x) / denominator);
            robot.motorBackRight.setPower((lx + ly - rot_x) / denominator);

            if (gamepad1.options) {
                // Options, reset IMU yaw
                robot.imu.resetYaw();
            }
            if (gamepad1.left_bumper && !(gamepad1.left_trigger > 0.2)) {
                // L1, left intake close
                robot.servoIntakeLeft.setPosition(Constants.intakeLeftClosed);
            }
            if (gamepad1.left_trigger >= Constants.leftTriggerFullyDown) {
                // L2, left intake open
                robot.servoIntakeLeft.setPosition(Constants.intakeLeftOpen);
            }
            if (gamepad1.right_bumper && !(gamepad1.right_trigger > 0.2)) {
                // R1, right intake close
                robot.servoIntakeRight.setPosition(Constants.intakeRightClosed);
            }
            if (gamepad1.right_trigger >= Constants.rightTriggerFullyDown) {
                // R2, right intake open
                robot.servoIntakeRight.setPosition(Constants.intakeRightOpen);
            }
            if (gamepad1.left_bumper && (gamepad1.left_trigger >= Constants.leftTriggerFullyDown)) {
                // L1 + L2, left arm toggle
                if (leftArmExtended) {
                    robot.motorArmLeft.setTargetPosition(Constants.armLeftRetracted);
                } else {
                    robot.motorArmLeft.setTargetPosition(Constants.armLeftExtended);
                }
                leftArmExtended = !leftArmExtended;
            }
            if (gamepad1.right_bumper && (gamepad2.right_trigger >= Constants.rightTriggerFullyDown)) {
                // R1 + R2, right arm toggle
                if (rightArmExtended) {
                    robot.motorArmRight.setTargetPosition(Constants.armRightRetracted);
                } else {
                    robot.motorArmRight.setTargetPosition(Constants.armRightExtended);
                }
                rightArmExtended = !rightArmExtended;
            }
            if (gamepad1.square) {
                if (rigged) {  // Extended; retract linear actuator.
                    robot.motorRiggingLinearActuatorLeft.setTargetPosition(Constants.getRiggingLinearActuatorRetracted);
                    robot.motorRiggingLinearActuatorRight.setTargetPosition(Constants.getRiggingLinearActuatorRetracted);
                } else {  // Retracted; extend linear actuator.
                    robot.motorRiggingLinearActuatorLeft.setTargetPosition(Constants.riggingLinearActuatorExtended);
                    robot.motorRiggingLinearActuatorRight.setTargetPosition(Constants.riggingLinearActuatorExtended);
                }
                rigged = !rigged;
            }
        }
    }
}
