package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="v2 TeleOp")
public class TeleOp_v2 extends LinearOpMode {
    @Override
    public void runOpMode() {

        Hardware_v2 robot = new Hardware_v2();
        robot.init(hardwareMap, telemetry);
        robot.reset();

        // Variables
        boolean clawUpperOpen = false;
        boolean clawLowerOpen = false;
        boolean isInScoringPosition = false;

        waitForStart();
        robot.setIntakePosition();

        while (opModeIsActive()) {
            // Fieldcentric controls
            double bot_heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double left_x = gamepad1.left_stick_x;
            double left_y = -gamepad1.left_stick_y;
            double rot_x = gamepad1.right_stick_x * 0.8;  // Make rotation less intense (80%)
            double lx = left_x * Math.cos(-bot_heading) - left_y * Math.sin(-bot_heading);
            double ly = left_x * Math.sin(-bot_heading) + left_y * Math.cos(-bot_heading);
            double denominator = Math.max(Math.abs(left_x) + Math.abs(left_y) + Math.abs(rot_x), 1);
            robot.motorFL.setPower(-(lx + ly + rot_x) / denominator);
            robot.motorBL.setPower((-lx + ly + rot_x) / denominator);
            robot.motorFR.setPower((ly - lx - rot_x) / denominator);
            robot.motorBR.setPower(-(lx + ly - rot_x) / denominator);

            // Slider height controls.

            if (gamepad1.right_bumper) {robot.setSliderPosition(0,1);}
            //if (gamepad1.right_trigger < 0.5) {robot.setSliderPosition(1);}
            if (gamepad1.left_bumper) {robot.setSliderPosition(1,1);}
            //if (gamepad1.left_trigger < 0.5) {robot.setSliderPosition(3);}

            // Claw controls.
            // Upper claw (toggle)
            /*
            if (gamepad1.dpad_up && !isInScoringPosition) {
                if (!clawUpperOpen) {robot.setClawPosition("upper", "open");}
                else {robot.setClawPosition("upper", "closed");}
                clawUpperOpen = !clawLowerOpen;
            } */



            // Lower claw (toggle)
            if (gamepad1.dpad_down && !isInScoringPosition) {
                if (!clawLowerOpen) {robot.setClawPosition("lower", "open");}
                else {robot.setClawPosition("lower", "closed");}
                clawLowerOpen = !clawLowerOpen;
            }

            // Transfer (toggle)
            if (gamepad1.options && clawUpperOpen && clawLowerOpen) {
                if (!isInScoringPosition) {robot.setScoringPosition();}  // Intake position; hence set to scoring position.
                else {robot.setIntakePosition();}  // Vice versa
                isInScoringPosition = !isInScoringPosition;
            }

            // Scoring (hit)
            if (gamepad1.dpad_left && isInScoringPosition) {
                robot.setClawPosition("upper", "open");
                robot.setClawPosition("lower", "open");
            }

            if (gamepad1.square) {
                robot.setClawPosition("upper", "closed");
            }

            if (gamepad1.triangle) {
                robot.setClawPosition("upper", "open");
            }

            if (gamepad1.circle) {
                robot.setClawPosition("lower", "closed");
            }

            if (gamepad1.cross) {
                robot.setClawPosition("lower", "open");
            }
            // Reset IMU.
            if (gamepad1.share) {robot.resetIMUYaw();}

            telemetry.update();
        }
    }
}