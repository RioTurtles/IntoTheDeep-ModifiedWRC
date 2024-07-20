package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp
public class CancelableTrajectoryTest extends LinearOpMode {
    double botHeading, left_x, left_y, rot_x, lx, ly, denominator;
    DcMotor motorFL, motorFR, motorBL, motorBR;
    IMU imu;

    @Override
    public void runOpMode() {
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        State currentState = State.DRIVER_CONTROLLED;
        Gamepad gamepad = new Gamepad();
        Gamepad lastGamepad = new Gamepad();

        int targetPos = 1;
        final Pose2d start = new Pose2d(12, -64, Math.toRadians(90));

        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        imu = hardwareMap.get(IMU.class, "imu");

        waitForStart();
        drive.setPoseEstimate(start);

        while (opModeIsActive()) {
            lastGamepad.copy(gamepad);
            gamepad.copy(gamepad1);
            Pose2d pose2d = drive.getPoseEstimate();

            if (gamepad.square) targetPos = 0;
            if (gamepad.triangle) targetPos = 1;
            if (gamepad.circle) targetPos = 2;

            if (currentState == State.DRIVER_CONTROLLED) {
                if (gamepad.cross && !lastGamepad.cross) {
                    currentState = State.AUTOMATIC;
                }

                botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                left_x = gamepad1.left_stick_x;
                left_y = -gamepad1.left_stick_y;
                rot_x = gamepad1.right_stick_x;  // Make rotation less intense (80%)
                lx = left_x * Math.cos(-botHeading) - left_y * Math.sin(-botHeading);
                ly = left_x * Math.sin(-botHeading) + left_y * Math.cos(-botHeading);
                denominator = Math.max(abs(left_x) + abs(left_y) + abs(rot_x), 1);

                motorFL.setPower((lx + ly + rot_x) / denominator);
                motorBL.setPower((-lx + ly + rot_x) / denominator);
                motorFR.setPower((ly - lx - rot_x) / denominator);
                motorBR.setPower((lx + ly - rot_x) / denominator);
            }

            if (currentState == State.AUTOMATIC) {
                TrajectorySequence sequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .splineToSplineHeading(new Pose2d(51.00, -36.00, Math.toRadians(180.00)), Math.toRadians(0.00))
                                .build();
                drive.followTrajectorySequenceAsync(sequence);

                if (gamepad.cross && !lastGamepad.cross) {
                    drive.breakFollowing();
                    currentState = State.DRIVER_CONTROLLED;
                }
            }

            drive.update();
            telemetry.addData("State", currentState);
            telemetry.addData("Selected", targetPos);
            telemetry.addData("x", pose2d.getX());
            telemetry.addData("y", pose2d.getY());
            telemetry.addData("heading", Math.toDegrees(pose2d.getHeading()));
            telemetry.update();
        }
    }

    public enum State {DRIVER_CONTROLLED, AUTOMATIC}
}
