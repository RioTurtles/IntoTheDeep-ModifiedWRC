package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class AllianceSideSetter extends LinearOpMode {
    @Override
    public void runOpMode() {
        if (!(Storage.allianceSide == 1 || Storage.allianceSide == -1)) Storage.allianceSide = 1;
        
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_bumper) Storage.allianceSide = -1;
            else if (gamepad1.right_bumper) Storage.allianceSide = 1;

            telemetry.addLine("Blue = left bumper\n");
            telemetry.addLine("Red = right  bumper\n");
            
            if (Storage.allianceSide == 1) telemetry.addData("Side", "Red");
            else if (Storage.allianceSide == -1) telemetry.addData("Side", "Blue");
            else telemetry.addData("Side", "Unset/Error");
            telemetry.update();
        }
        
    }
}
