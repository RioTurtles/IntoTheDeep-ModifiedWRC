package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

@TeleOp(name="Distance/Ultrasonic Sensor Test")
public class DistanceUltrasonicTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Hardware_v3 robot = new Hardware_v3();
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        robot.init(hardwareMap, telemetry);
        robot.reset();

        waitForStart();
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive()) {
            if (gamepad1.triangle) {
                drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

//            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x + robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));

            telemetry.addData("Ultrasonic / F", robot.ultraF.getVoltage());
            telemetry.addData("Ultrasonic / L", robot.ultraL.getVoltage());
            telemetry.addData("Distance", robot.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Encoder para", robot.parallelEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
