package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@TeleOp(name="AutoBackdrop, Ultrasonic")
public class AutoBackdropUltrasonic extends LinearOpMode {
    Hardware_v3 robot;
    SampleMecanumDriveCancelable drive;
    Position targetPosition;
    State currentState;
    Control controller;
    Pose2d pose;
    Vector2d vector;
    private enum Position {LEFT, CENTRE, RIGHT}
    private enum State {CORNER, TRUSS, BACKDROP}
    private enum Control {DRIVER, AUTOMATIC}

    private static PIDController controllerX = new PIDController(3.80 , 0.01, 0.1);
    private static PIDController controllerY = new PIDController(0.95, 0.1, 0.1);

    double x, y, heading;
    int transmissionInterval;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Hardware_v3();
        robot.init(hardwareMap, telemetry);
        robot.reset();
        drive = new SampleMecanumDriveCancelable(hardwareMap);

        targetPosition = Position.CENTRE;
        currentState = State.CORNER;
        controller = Control.DRIVER;

        controllerX.setTolerance(0.01);
        controllerY.setTolerance(0.01);

        waitForStart();
        drive.setPoseEstimate(new Pose2d(-37.098, -63.110, Math.toRadians(90.00)));
        while (opModeIsActive()) {
            pose = drive.getPoseEstimate();
            x = robot.ultraL.getVoltage();
            y = robot.ultraF.getVoltage();
            heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double outputHeading = Math.toRadians(((180 * (heading / Math.abs(heading))) - Math.toDegrees(heading)) * 0.775);

            if (controller == Control.DRIVER) {
                if (gamepad1.square) targetPosition = Position.LEFT;
                if (gamepad1.triangle) targetPosition = Position.CENTRE;
                if (gamepad1.circle) targetPosition = Position.RIGHT;
                if (gamepad1.cross) controller = Control.AUTOMATIC;
                if (gamepad1.right_trigger > 0) currentState = State.CORNER;
                if (gamepad1.touchpad) robot.imu.resetYaw();

                double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double lx = gamepad1.left_stick_x * Math.cos(-botHeading) + gamepad1.left_stick_y * Math.sin(-botHeading);
                double ly = gamepad1.left_stick_x * Math.sin(-botHeading) - gamepad1.left_stick_y * Math.cos(-botHeading);
                double rot_x = gamepad1.right_stick_x * 0.8;  // Make rotation less intense (80%)
                double denominator = Math.max(abs(gamepad1.left_stick_x) + abs(gamepad1.left_stick_x) + abs(rot_x), 1);
                robot.motorFL.setPower((lx + ly + rot_x) / denominator);
                robot.motorBL.setPower((-lx + ly + rot_x) / denominator);
                robot.motorFR.setPower((ly - lx - rot_x) / denominator);
                robot.motorBR.setPower((lx + ly - rot_x) / denominator);
            }

            if (controller == Control.AUTOMATIC) {
                if (gamepad1.right_bumper)  {
                    drive.breakFollowing();
                    robot.motorFL.setPower(0);
                    robot.motorFR.setPower(0);
                    robot.motorBL.setPower(0);
                    robot.motorBR.setPower(0);
                    controller = Control.DRIVER;
                    continue;
                }
                // TODO: continue tuning values here
                switch (currentState) {
                    case CORNER:
                        TrajectorySequence ts1 = drive.trajectorySequenceBuilder(pose)
                                .splineToLinearHeading(new Pose2d(-38.653, -60.518), Math.toRadians(180))
                                .build();

                        drive.followTrajectorySequenceAsync(ts1); if (!drive.isBusy()) {
                            currentState = State.TRUSS;
                            controller = Control.DRIVER;
                        }
                        break;
                }
            }

            drive.update();
            telemetry.addData("Controller", controller);
            telemetry.addData("Position", targetPosition);
            telemetry.addData("State", currentState);
            telemetry.addLine();
            telemetry.addData("PE", pose.getX() + ", " + pose.getY() + " | " + Math.toDegrees(pose.getHeading()));
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("h", heading);
            telemetry.addData("h(d)", Math.toDegrees(heading));
            telemetry.addData("output heading", outputHeading);
            telemetry.update();
            transmissionInterval = telemetry.getMsTransmissionInterval();
        }
    }
}
