package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class SliderTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Project2Hardware robot = new Project2Hardware(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Arm", robot.getArmAngle());
            telemetry.addData("Slider Encoder",
                    robot.lengthToEncoderValueSlider(robot.getSliderLength()));
            telemetry.addData("Slider Length", robot.getSliderLength());
            telemetry.update();
        }
    }
}
