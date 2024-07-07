package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="v2 Measurement")
public class Measurement_v2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot = new Project1Hardware();
        robot.init(hardwareMap, telemetry);

        robot.motorSliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorSliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("FL", robot.motorFL.getPower());
            telemetry.addData("FR", robot.motorFR.getPower());
            telemetry.addData("BL", robot.motorBL.getPower());
            telemetry.addData("BR", robot.motorBR.getPower());
            telemetry.addData("Slider (Left)", robot.motorSliderLeft.getCurrentPosition());
            telemetry.addData("Slider (Right)", robot.motorSliderRight.getCurrentPosition());
            telemetry.update();
        }
    }
}
