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

                final int LIMIT = 975;
                final int CONTROL = 100;

                // While holding -> keep going
                if (gamepad.right_trigger > 0 && robot.getSlider() < (LIMIT - 50)) {
                        robot.slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        if (robot.getSlider() > (LIMIT - CONTROL)) robot.slider.setPower(
                                Math.pow((double) Math.abs(robot.getSlider() - LIMIT) / LIMIT, 4)
                        );
                        else robot.slider.setPower(1);
                } else if (gamepad.left_trigger > 0) {
                    if (robot.getSlider() < 300) {  // Go back to INTAKE state
                        robot.slider.setPower(-1);
                        robot.setSlider(0);
                        state = State.INTAKE;
                        timer1.reset();
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
                    switch (robot.scoringHeight) {
                        case HIGH: robot.setSlider(Project2Hardware.SLIDER_HIGH); break;
                        case LOW: robot.setSlider(Project2Hardware.SLIDER_LOW); break;
                    }
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

                switch (robot.scoringHeight) {
                    case HIGH: robot.setSlider(Project2Hardware.SLIDER_HIGH); break;
                    case LOW: robot.setSlider(Project2Hardware.SLIDER_LOW); break;
                }
            }

            if (operator.triangle) autoAlignTarget = 0.0;  // Forward
            else if (operator.square) autoAlignTarget = 135.0;  // Basket alignment
            else if (operator.circle) autoAlignTarget = -90.0;  // Submersible alignment
            else if (operator.cross) autoAlignTarget = 180.0;  // Backwards
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

                pivot *= 0.2;  // Adjust kP here

                heading = 0;
                vertical *= 0.8;
                horizontal *= 0.2;
            }

            // Height switching
            if (operator.options) robot.scoringHeight = ScoringHeight.HIGH;
            if (operator.share) robot.scoringHeight = ScoringHeight.LOW;

            // Emergency resets
            if (operator.dpad_right) robot.powerResetSlider();
            if (!operator.dpad_right && lastOperator.dpad_right) robot.resetSlider();
            if (operator.dpad_down) {
                robot.powerResetArm();
                if (!operator.right_bumper) robot.clawOpen();
            }
            if (!operator.dpad_down && lastOperator.dpad_down) robot.resetArm();

            if (operator.left_bumper && operator.right_bumper && operator.touchpad) {
                state = State.INIT;
                robot.arm.setPower(0);
                robot.slider.setPower(0);
                robot.clawOpen();
            }

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
