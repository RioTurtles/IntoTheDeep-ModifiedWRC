package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="v2 Measurement")
public class Measurement_v2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware_v2 robot = new Hardware_v2();
        robot.init(hardwareMap, telemetry);
        robot.reset();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("FL", robot.motorFL.getPower());
            telemetry.addData("FR", robot.motorFR.getPower());
            telemetry.addData("BL", robot.motorBL.getPower());
            telemetry.addData("BR", robot.motorBR.getPower());
            telemetry.addData("Slider (Left)", robot.motorSliderLeft.getCurrentPosition());
            telemetry.addData("Slider (Right)", robot.motorSliderRight.getCurrentPosition());
        }
    }
}
