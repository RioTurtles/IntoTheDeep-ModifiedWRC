package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="v2 Autonomous")
public class Autonomous_v2 extends LinearOpMode {
    public static int orientation = 0;

    @Override
    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.triangle) {
                orientation += 1;
            }
            telemetry.addData("orientation", orientation);
            telemetry.update();
        }
    }
}
