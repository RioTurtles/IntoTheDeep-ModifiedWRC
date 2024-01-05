package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="v2 TeleOp")
public class TeleOp_v2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        Hardware_v2 robot = new Hardware_v2(hardwareMap);
        telemetry.addData("Status", "Initialised");
        telemetry.update();

        waitForStart();
        telemetry.addData("Status", "Running");
        telemetry.update();

        while (opModeIsActive()) {

        }
    }
}
