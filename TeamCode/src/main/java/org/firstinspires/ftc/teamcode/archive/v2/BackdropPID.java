package org.firstinspires.ftc.teamcode.archive.v2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="v2 Backdrop Distance Sensor")
public class BackdropPID extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        final double kP = 0.5;
        final double kI = 1;
        final double kD = 1;
        final double target = 3;
        double integral = 0;
        double derivative = 0;
        double lastError = 0;
        double error;
        double output;

        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        while (opModeIsActive()) {
            while (true) {
                double reading = distanceSensor.getDistance(DistanceUnit.CM);
                if (reading > target) {
                    error = target - reading;
                    integral += error;
                    derivative = error - lastError;
                    output = kP*error + kI*integral + kD*derivative;
                    lastError = error;

                    telemetry.addData("target", target);
                    telemetry.addData("actual", reading);
                    telemetry.addData("output", output);
                    telemetry.update();
                } else {
                    break;
                }
            }

        }
    }
}
