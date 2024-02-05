package org.firstinspires.ftc.teamcode.archive.v2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="v2 TeleOp")
public class TeleOp_v2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Hardware_v2 robot = new Hardware_v2();
        robot.init(hardwareMap, telemetry);
        robot.reset();

        // Variables
        final double rotationMultiplier = 0.8;
        final double brakeModeMultiplier = 0.3;

        double botHeading;
        double left_x;
        double left_y;
        double rot_x;
        double lx;
        double ly;
        double denominator;

        int returnStage = 0;

        boolean clawUpperOpen = false;
        boolean clawLowerOpen = false;
        boolean isInScoringPosition = false;
        boolean isInBrakeMode = false;
        boolean scored = false;
        int waiting = 0;

        robot.setIntakePosition();
        ElapsedTime timer = new ElapsedTime();
        waitForStart();

        while (opModeIsActive()) {

            // Fieldcentric controls
            botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            left_x = gamepad1.left_stick_x;
            left_y = -gamepad1.left_stick_y;
            rot_x = gamepad1.right_stick_x * rotationMultiplier;  // Make rotation less intense (80%)
            lx = left_x * Math.cos(-botHeading) - left_y * Math.sin(-botHeading);
            ly = left_x * Math.sin(-botHeading) + left_y * Math.cos(-botHeading);
            denominator = Math.max(Math.abs(left_x) + Math.abs(left_y) + Math.abs(rot_x), 1);



            if (gamepad1.left_bumper) {robot.servoClawUpper.setPosition(0.22); clawUpperOpen = false;}
            if (gamepad1.right_bumper) {robot.servoClawLower.setPosition(0.74); clawLowerOpen = false;}
            if (gamepad1.left_trigger > 0) {robot.servoClawUpper.setPosition(0); clawUpperOpen = true;}
            if (gamepad1.right_trigger > 0) {robot.servoClawLower.setPosition(1); clawLowerOpen = true;}

            // Scoring system.
            // Press triangle to raise slider and set arm/claw to scoring position.
            if (gamepad1.triangle && !clawLowerOpen && !clawUpperOpen) {
                robot.servoClawLower.setPosition(0.22);
                robot.servoClawUpper.setPosition(0.74);
                robot.setSliderPosition(1);

                while (robot.motorSliderLeft.getCurrentPosition() < 1000) {}
                robot.setScoringPosition();
                isInScoringPosition = true;
                scored = false;
            }

            // Hold circle to score and release to reset to lower slider and set intake position.
            if (gamepad1.circle && isInScoringPosition) {

                robot.servoClawLower.setPosition(1);
                lx = 0;
                ly = 0;
                rot_x = 0;
                sleep(400);
                robot.servoClawUpper.setPosition(0);
                scored = true;
            } else if (scored) {
                scored = false;
                robot.servoClawUpper.setPosition(0.22);
                robot.servoClawLower.setPosition(0.74);
                sleep(400);
                robot.setIntakePosition();
                sleep(400);
                returnStage = 1;
            }

            if (gamepad1.cross && returnStage == 1) {
                robot.setSliderPosition(0);
                isInScoringPosition = false;

                while (robot.motorSliderLeft.getCurrentPosition() < 100) {}
                robot.servoClawUpper.setPosition(0);
                robot.servoClawLower.setPosition(1);
                returnStage = 0;
            }

            // Adjust the pitch for wing intake.
            if (gamepad1.square) {
                robot.servoArmLeft.setPosition(0.967);
                robot.servoArmRight.setPosition(0.967);
                robot.servoClawPitchLeft.setPosition(0.47);
                robot.servoClawPitchRight.setPosition(0.47);
            }

            // Rigging control.
            if (gamepad1.dpad_left && !clawLowerOpen && !clawUpperOpen) {
                robot.setSliderPosition(0, 0.35);
                robot.servoArmLeft.setPosition(0.45);
                robot.servoArmRight.setPosition(0.45);
                robot.servoClawPitchLeft.setPosition(0.32);
                robot.servoClawPitchRight.setPosition(0.32);
            }

            // Brake mode
            isInBrakeMode = gamepad1.options;

            // Reset IMU.
            if (gamepad1.share) {robot.imu.resetYaw();}
            if (isInBrakeMode || isInScoringPosition) {
                robot.motorFL.setPower((lx + ly + rot_x)*brakeModeMultiplier / denominator);
                robot.motorBL.setPower((-lx + ly + rot_x)*brakeModeMultiplier / denominator);
                robot.motorFR.setPower((ly - lx - rot_x)*brakeModeMultiplier / denominator);
                robot.motorBR.setPower((lx + ly - rot_x)*brakeModeMultiplier / denominator);
            } else {
                robot.motorFL.setPower((lx + ly + rot_x) / denominator);
                robot.motorBL.setPower((-lx + ly + rot_x) / denominator);
                robot.motorFR.setPower((ly - lx - rot_x) / denominator);
                robot.motorBR.setPower((lx + ly - rot_x) / denominator);
            }

            telemetry.update();
        }
    }
}