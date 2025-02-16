package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Objects;

@TeleOp
public class TeleoperatedV1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Project2Hardware robot = new Project2Hardware(hardwareMap);
        State state = State.INIT;
        Gamepad gamepad = new Gamepad(), lastGamepad = new Gamepad();
        Gamepad operator = new Gamepad(), lastOperator = new Gamepad();

        Double autoAlignTarget;
        double vertical, horizontal, pivot, heading;
        boolean returning = false, chambered = false;

        waitForStart();
        while (opModeIsActive()) {
            lastGamepad.copy(gamepad); gamepad.copy(gamepad1);
            lastOperator.copy(operator); operator.copy(gamepad2);
            vertical = -gamepad.left_stick_y; horizontal = gamepad.left_stick_x;
            pivot = gamepad.right_stick_x; heading = robot.getIMUYaw();

            if (state == State.INIT) {
                robot.arm.setPower(0);

                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    robot.clawOpen();
                    state = State.INTAKE;
                }
            }

            else if (state == State.INTAKE) {
                robot.arm.setPower(0);

                if (gamepad.left_trigger > 0) robot.extendSlider(); else robot.retractSlider();
                if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                    if (robot.clawClosed) robot.clawOpen(); else robot.clawClose();
                }

                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    robot.clawClose();
                    robot.retractSlider();
                    state = State.TRANSFER_ARM;
                    returning = false;
                }
            }

            else if (state == State.TRANSFER_ARM) {
                robot.arm.setPower(1);

                if (!returning) {  // Forward
                    if (robot.sliderInPosition(5)) robot.setArm(100);
                    if (robot.getArmError() <= 2) state = State.TRANSFER_SLIDER;
                } else {  // Reverse
                    robot.retractSlider();
                    if (robot.sliderInPosition(5)) robot.setArm(0);
                    if (robot.getArmError() <= 2) state = State.INTAKE;
                }

                if (gamepad.left_bumper && !lastGamepad.left_bumper) returning = true;
                if (gamepad.right_bumper && !lastGamepad.right_bumper) returning = false;
            }

            else if (state == State.TRANSFER_SLIDER) {
                if (!returning) {  // Forward
                    robot.setSlider(900);  // TODO: adjust slider value

                    if (robot.sliderInPosition(5)) {
                        if (robot.scoringMode == ScoringMode.BASKET) state = State.SCORING_BASKET;
                        else state = State.SCORING_CHAMBER;
                        chambered = false;
                    }
                } else {  // Reverse
                    robot.retractSlider();
                    if (robot.sliderInPosition(5)) state = State.TRANSFER_ARM;
                }

                if (gamepad.left_bumper && !lastGamepad.left_bumper) returning = true;
                if (gamepad.right_bumper && !lastGamepad.right_bumper) returning = false;
            }

            else if (state == State.SCORING_BASKET) {
                if (gamepad.right_trigger > 0) {
                    if (robot.clawClosed) robot.clawOpen(); else robot.clawClose();
                }

                if ((gamepad.left_bumper && !lastGamepad.left_bumper
                        || (gamepad.right_bumper && !lastGamepad.right_bumper))) {
                    returning = true;
                    state = State.TRANSFER_SLIDER;
                }
            }

            else if (state == State.SCORING_CHAMBER) {
                if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                    // TODO: adjust
                    if (!chambered) {
                        robot.setSliderLength(robot.getSliderLength() - 10); chambered = true;
                    } else {
                        robot.setSliderLength(robot.getSliderLength() + 10); chambered = false;
                    }
                }

                if (robot.sliderInPosition(5)) {
                    if (chambered) robot.clawOpen(); else robot.clawClose();
                }

                if ((gamepad.left_bumper && !lastGamepad.left_bumper
                        || (gamepad.right_bumper && !lastGamepad.right_bumper))) {
                    returning = true;
                    state = State.TRANSFER_SLIDER;
                }
            }

            if (gamepad.dpad_up) autoAlignTarget = 0.0;  // Chamber alignment
            else if (gamepad.dpad_left) autoAlignTarget = 135.0;  // Basket alignment
            else if (gamepad.dpad_right) autoAlignTarget = 90.0;  // Submersible alignment
            else if (gamepad.dpad_down) autoAlignTarget = 180.0;  // Observation alignment
            else autoAlignTarget = null;

            if (Objects.nonNull(autoAlignTarget)) {
                assert autoAlignTarget != null;

                double current = Math.toDegrees(heading);
                double smallerAngle = Math.min(
                        Math.abs(current - autoAlignTarget),
                        360 - Math.abs(current - autoAlignTarget)
                );

                double resultant1 = current - smallerAngle;
                if (resultant1 <= -180) resultant1 += 360;
                double resultant2 = current + smallerAngle;
                if (resultant2 > 180) resultant2 -= 360;

                if (resultant1 == autoAlignTarget) pivot = Math.toRadians(smallerAngle);
                else if (resultant2 == autoAlignTarget) pivot = Math.toRadians(-smallerAngle);

                heading = 0;
                vertical *= 0.8;
                horizontal *= 0.2;
            }

            // Mode switching
            if (gamepad.square || operator.square) robot.scoringMode = ScoringMode.BASKET;
            if (gamepad.circle || operator.circle) robot.scoringMode = ScoringMode.CHAMBER;
            if (gamepad.triangle || operator.triangle) robot.scoringHeight = ScoringHeight.HIGH;
            if (gamepad.cross || operator.cross) robot.scoringHeight = ScoringHeight.LOW;

            // Emergency resets
            if (gamepad.dpad_right && !lastGamepad.dpad_right) robot.powerResetSlider();
            if (!gamepad.dpad_right && lastGamepad.dpad_right) robot.resetSlider();
            if (gamepad.dpad_down && !lastGamepad.dpad_down) robot.powerResetArm();
            if (!gamepad.dpad_down && lastGamepad.dpad_down) robot.resetArm();

            if (gamepad.touchpad) robot.imu.resetYaw();
            robot.drivetrain.remote(vertical, horizontal, pivot, heading);

            if (returning) telemetry.addData("State", state + " | *");
            else telemetry.addData("State", state);
            telemetry.addData("Scoring", robot.getScoringState());
            telemetry.addLine();
            telemetry.addData("Claw", robot.getClawString());
            telemetry.addData("Slider", robot.getSliderLength());
            telemetry.update();
        }
    }

    enum State {
        INIT,
        INTAKE,
        TRANSFER_ARM,
        TRANSFER_SLIDER,
        SCORING_BASKET,
        SCORING_CHAMBER
    }
}
