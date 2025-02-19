package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;

@TeleOp
public class TeleoperatedV1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Project2Hardware robot = new Project2Hardware(hardwareMap);
        State state = State.INIT;
        Gamepad gamepad = new Gamepad(), lastGamepad = new Gamepad();
        Gamepad operator = new Gamepad(), lastOperator = new Gamepad();
        ElapsedTime timer1 = new ElapsedTime();

        Double autoAlignTarget;
        double vertical, horizontal, pivot, heading;
        boolean returning = false;

        waitForStart();
        while (opModeIsActive()) {
            lastGamepad.copy(gamepad); gamepad.copy(gamepad1);
            lastOperator.copy(operator); operator.copy(gamepad2);
            vertical = gamepad.left_stick_y; horizontal = -gamepad.left_stick_x;
            pivot = -gamepad.right_stick_x; heading = robot.getIMUYaw();

            if (state == State.INIT) {
                robot.arm.setPower(0);

                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    robot.clawOpen();
                    state = State.INTAKE;
                    timer1.reset();
                }
            }

            else if (state == State.INTAKE) {
                returning = false;
                robot.arm.setPower(0);
                robot.slider.setPower(1);

                if (timer1.milliseconds() > 150) robot.retractSlider();
                if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                    robot.clawOpen();
                    state = State.INTAKE_EXTEND;
                    robot.extendSlider();
                }

                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    if (robot.clawClosed) robot.clawOpen(); else robot.clawClose();
                }

                if (gamepad.triangle && !lastGamepad.triangle) {
                    robot.clawClose();
                    robot.retractSlider();
                    state = State.TRANSFER_ARM;
                }
            }

            else if (state == State.INTAKE_EXTEND) {
                returning = false;
                robot.arm.setPower(0);

                // While holding -> keep going
                if (gamepad.right_trigger > 0) {
                    if (robot.getSlider() < 1400) {
                        robot.slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.slider.setPower(1);
                    }
                } else if (gamepad.left_trigger > 0) {
                    if (robot.getSlider() < 300) {  // Go back to INTAKE state
                        timer1.reset();
                        robot.setSlider(0);
                        state = State.INTAKE;
                    } else if (robot.getSlider() > 0) {
                        robot.slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.slider.setPower(-0.5);
                    }
                } else {
                    robot.slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    robot.slider.setTargetPosition(robot.slider.getCurrentPosition());
                    robot.slider.setPower(1);
                    robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                // Left bumper -> go back
                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    robot.clawClose();
                    timer1.reset();
                    state = State.INTAKE;
                    // Slider retracts in the INTAKE state.
                }

                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    if (robot.clawClosed) robot.clawOpen(); else robot.clawClose();
                }

                if (gamepad.triangle && !lastGamepad.triangle) {
                    robot.clawClose();
                    state = State.TRANSFER_EXTEND;
                    timer1.reset();
                    // Arm lifts in the next state.
                }
            }

            else if (state == State.TRANSFER_EXTEND) {
                if (timer1.milliseconds() > 150) {
                    robot.setArm(20);
                    if (robot.getArmError() <= 3) state = State.TRANSFER_ARM;
                }
            }

            else if (state == State.TRANSFER_ARM) {
                robot.arm.setPower(1);
                robot.setSlider(0);

                if (!returning) {  // Forward
                    if (robot.sliderInPosition(5)) {
                        robot.setArm(Project2Hardware.BASKET_ANGLE);

                        if (gamepad.right_bumper && !lastGamepad.right_bumper)
                            state = State.TRANSFER_SLIDER;
                    }
                } else {  // Reverse
                    robot.setArm(0);
                    robot.clawOpen();
                    if (robot.getArmError() <= 2
                            || (gamepad.triangle && !lastGamepad.triangle))
                        state = State.INTAKE;
                }

                if (gamepad.left_bumper && !lastGamepad.left_bumper) returning = true;
                if (gamepad.right_bumper && !lastGamepad.right_bumper) returning = false;
            }

            else if (state == State.TRANSFER_SLIDER) {
                robot.arm.setPower(1);
                if (!returning) {  // Forward
                    robot.setSlider(1400);  // TODO: adjust slider value
                    if (robot.sliderInPosition(5)) state = State.SCORING;
                } else {  // Reverse
                    robot.setSlider(0);
                    if (robot.sliderInPosition(5)) state = State.TRANSFER_ARM;
                }

                if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0))
                    state = State.SCORING;

                if (gamepad.left_bumper && !lastGamepad.left_bumper) returning = true;
                if (gamepad.right_bumper && !lastGamepad.right_bumper) returning = false;
            }

            else if (state == State.SCORING) {
                if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                    if (robot.clawClosed) robot.clawOpen(); else robot.clawClose();
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

                pivot *= 0.6;  // Adjust kP here

                heading = 0;
                vertical *= 0.8;
                horizontal *= 0.2;
            }

            // Mode switching & height switching
            if (gamepad.square || operator.square) robot.scoringMode = ScoringMode.BASKET;
            if (gamepad.circle || operator.circle) robot.scoringMode = ScoringMode.CHAMBER;
            if (operator.triangle) robot.scoringHeight = ScoringHeight.HIGH;
            if (operator.cross) robot.scoringHeight = ScoringHeight.LOW;

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
            telemetry.addData("Slider", robot.getSlider());
            telemetry.addData("Arm target", robot.armTargetAngle);
            telemetry.addData("Arm current angle", robot.getArmAngle());
            telemetry.addData("Arm error", robot.getArmError());
            telemetry.addData("Slider in pos?", robot.sliderInPosition(5));
            telemetry.addData("Slider target encoder", robot.slider.getTargetPosition());
            telemetry.addData("Slider current encoder", robot.slider.getCurrentPosition());
            telemetry.update();
        }
    }

    enum State {
        INIT,
        INTAKE,
        INTAKE_EXTEND,
        TRANSFER_EXTEND,
        TRANSFER_ARM,
        TRANSFER_SLIDER,
        SCORING
    }
}
