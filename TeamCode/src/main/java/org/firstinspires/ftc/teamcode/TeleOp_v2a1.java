package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="v2 TeleOp A1")
public class TeleOp_v2a1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Hardware_v2 robot = new Hardware_v2();
        robot.init(hardwareMap, telemetry);
        robot.reset();

        // Variables
        final double ROTATION_MULTIPLIER = 0.8;
        final double BRAKE_MODE_MULTIPLIER = 0.3;

        double botHeading;
        double left_x;
        double left_y;
        double rot_x;
        double lx;
        double ly;
        double denominator;

        int stage = 0;

        boolean scored = false;

        robot.setIntakePosition();
        ElapsedTime timer = new ElapsedTime();
        waitForStart();

        while (opModeIsActive()) {
            // Fieldcentric controls
            botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            left_x = gamepad1.left_stick_x;
            left_y = -gamepad1.left_stick_y;
            rot_x = gamepad1.right_stick_x * ROTATION_MULTIPLIER;  // Make rotation less intense (80%)
            lx = left_x * Math.cos(-botHeading) - left_y * Math.sin(-botHeading);
            ly = left_x * Math.sin(-botHeading) + left_y * Math.cos(-botHeading);
            denominator = Math.max(Math.abs(left_x) + Math.abs(left_y) + Math.abs(rot_x), 1);

            // Stage 0: Intake
            if (stage == 0) {
                if (gamepad1.left_bumper) {
                    // Pressing left bumper will open both claw and set the claw pitch to intake position.
                    robot.openUpperClaw();
                    robot.openLowerClaw();
                    robot.setIntakePosition();  // Lower claw pitch further (to allow pixel intake).
                } else if (gamepad1.right_bumper) {
                    // Right bumper closes both claws.
                    robot.closeLowerClaw();
                    lx = 0; ly = 0; rot_x = 0;  // Pause all motors as sleep() is blocking.
                    sleep(175);  // Pause 100 ms as they close too fast.
                    robot.closeUpperClaw();
                    stage = 1;
                    timer.reset();
                }
            }

            // Stage 1: Transfer
            // After claw intake (pressing right bumper)
            if (stage == 1) {
                // Raise the arm to prevent grinding.
                if (timer.milliseconds() > 300 && timer.milliseconds() < 400) {robot.setTransferPosition();}

                // Raising slider and proceeding to next stage.
                if (gamepad1.triangle) {  // Press triangle to raise slider.
                    robot.setSliderPosition(1);
                }

                if (robot.motorSliderLeft.getCurrentPosition() > 1000) {
                    // Set to scoring position when slider is raised.
                    robot.setScoringPosition();
                    stage = 2;
                }

                // Return to previous stage.
                if (gamepad1.left_bumper) {
                    stage = 0;
                    // Reset claw back to intake position.
                    robot.setIntakePosition();
                    robot.openLowerClaw();
                    robot.openUpperClaw();
                }
            }

            // Stage 2: Scoring
            // Hold circle to score and release to reset to lower slider and set intake position.
            if (stage == 2 && robot.isInScoringPosition) {
                if (gamepad1.circle && !scored) {
                    // robot.closeUpperClaw();
                    robot.openLowerClaw();  // Release lower claw.

                    lx = 0; ly = 0; rot_x = 0;  // Pause motors as sleep() is blocking.
                    sleep(400);  // Delay 400 ms.
                    robot.openUpperClaw();  // Release upper claw.
                    sleep(250);
                    scored = true;
                    timer.reset();
                }

                if (scored) {
                    robot.closeUpperClaw();
                    robot.closeLowerClaw();  // Close both claws and proceed.
                    if (timer.milliseconds()>300) {  // Allow 300 ms for claws to close.
                        robot.setTransferPosition();
                        stage = 3;
                    }
                    if (timer.milliseconds()>700) {  // Allow 400 ms for intake to move (timer not reset).
                        stage = 3;
                    }
                }

                // Rigging control.
                if (gamepad1.dpad_up && !scored) {
                    robot.servoArmLeft.setPosition(0.45);
                    robot.servoArmRight.setPosition(0.45);
                    robot.servoClawPitchLeft.setPosition(0.32);
                    robot.servoClawPitchRight.setPosition(0.32);
                    robot.setSliderPosition(0, 0.35);
                }
            }

            // Stage 3: Returning to intake
            if (stage == 3) {
                scored = false;
                if (gamepad1.cross) {
                    robot.setSliderPosition(0);
                    robot.isInScoringPosition = false;
                }

                if (robot.motorSliderLeft.getCurrentPosition() < 100) {
                    robot.setTransferPosition();
                    stage = 0;
                }
            }

            // Reset IMU.
            if (gamepad1.share) {robot.imu.resetYaw();}

            // Drone launcher.
            if (gamepad1.dpad_down) {
                robot.servoDrone.setPower(1);
                sleep(130);
                robot.servoDrone.setPower(0);
            }

            if (gamepad1.left_trigger > 0 || robot.motorSliderLeft.getCurrentPosition() > 700)  {
                robot.motorFL.setPower((lx + ly + rot_x)*BRAKE_MODE_MULTIPLIER / denominator);
                robot.motorBL.setPower((-lx + ly + rot_x)*BRAKE_MODE_MULTIPLIER / denominator);
                robot.motorFR.setPower((ly - lx - rot_x)*BRAKE_MODE_MULTIPLIER / denominator);
                robot.motorBR.setPower((lx + ly - rot_x)*BRAKE_MODE_MULTIPLIER / denominator);
            } else {
                robot.motorFL.setPower((lx + ly + rot_x) / denominator);
                robot.motorBL.setPower((-lx + ly + rot_x) / denominator);
                robot.motorFR.setPower((ly - lx - rot_x) / denominator);
                robot.motorBR.setPower((lx + ly - rot_x) / denominator);
            }

            telemetry.addData("STAGE", stage);
            telemetry.addData("- in brake", (gamepad1.left_trigger > 0 || robot.isInScoringPosition));
            telemetry.addData("- in scoring", robot.isInScoringPosition);
            telemetry.update();
        }
    }
}