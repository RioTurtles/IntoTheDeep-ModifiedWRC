package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="2+0 Red Far, Truss")
public class RRAutonRedFarTruss extends LinearOpMode {
    Objective objective = Objective.INITIALISE;
    OpenCvWebcam webcam;
    int randomizationResult = 2;
    boolean yReady;
    boolean parkRight;

    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot = new Project1Hardware();
        robot.init(hardwareMap, telemetry);
        robot.reset();
        robot.retractSlider();
        robot.bothClawClose();

        ElapsedTime timer1 = new ElapsedTime();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-40.33, -62.80, Math.toRadians(90.00));
        Pose2d nowPose;

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        webcam.setPipeline(new TeamPropPipeline());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);}

            @Override
            public void onError(int errorCode) {}
        });

        telemetry.addData("Randomization", randomizationResult);
        telemetry.addData("Status", "Initialised");
        telemetry.update();

        drive.setPoseEstimate(startPose);
        TrajectorySequence pLeft = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-36.61, -44.99, Math.toRadians(240.00)), Math.toRadians(253.60))
                .addTemporalMarker(() -> {
                    timer1.reset();
                    objective = Objective.SCORE_PURPLE;
                })
                .build();
        TrajectorySequence pMiddle = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-36.61, -44.99, Math.toRadians(270.00)), Math.toRadians(253.60))
                .addTemporalMarker(() -> {
                    timer1.reset();
                    objective = Objective.SCORE_PURPLE;
                })
                .build();
        TrajectorySequence pRight = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-36.61, -44.99, Math.toRadians(300.00)), Math.toRadians(253.60))
                .addTemporalMarker(() -> {
                    timer1.reset();
                    objective = Objective.SCORE_PURPLE;
                })
                .build();
        Trajectory yellow = null;

        waitForStart();
        webcam.stopRecordingPipeline();
        webcam.stopStreaming();
        timer1.reset();

        while (opModeIsActive()) {
            nowPose = drive.getPoseEstimate();

            if (objective == Objective.INITIALISE) {
                objective = Objective.PATH_TO_PURPLE;
            }

            if (objective == Objective.PATH_TO_PURPLE) {
                switch (randomizationResult) {
                    case 1: drive.followTrajectorySequence(pLeft); break;
                    default: case 2: drive.followTrajectorySequence(pMiddle); break;
                    case 3: drive.followTrajectorySequence(pRight); break;
                }
            }

            if (objective == Objective.SCORE_PURPLE) {
                if (timer1.milliseconds() > 1760) {
                    robot.bothClawClose();
                    robot.arm.setPower(0);
                    robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    objective = Objective.PATH_TO_YELLOW_GENERATION;
                } else if (timer1.milliseconds() > 1360) {
                    robot.clawRScoring();
                    robot.retractSlider();
                } else if (timer1.milliseconds() > 1060) robot.rightClawOpen();
                else if (timer1.milliseconds() > 0) {
                    robot.clawRIntake();

                    switch (randomizationResult) {
                        case 1: robot.setSlider(630); break;
                        default: case 2: robot.setSlider(135); break;
                        case 3: robot.setSlider(550); break;
                    }
                }
            }

            if (objective == Objective.PATH_TO_YELLOW_GENERATION) {
                TrajectoryBuilder builder = drive.trajectoryBuilder(nowPose)
                        .splineToLinearHeading(
                                new Pose2d(-32.51, -60.08, Math.toRadians(0.00)),
                                Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                null
                        )
                        .splineTo(new Vector2d(13.88, -60.08), Math.toRadians(0.00));

                switch (randomizationResult) {
                    case 1: builder.splineToConstantHeading(new Vector2d(35.20, -30.01), Math.toRadians(0.00)); break;
                    default: case 2: builder.splineToConstantHeading(new Vector2d(35.20, -34.51), Math.toRadians(0.00)); break;
                    case 3: builder.splineToConstantHeading(new Vector2d(35.20, -42.71), Math.toRadians(0.00)); break;
                }

                yellow = builder
                        .addSpatialMarker(new Vector2d(13.88, -60.08), () -> {
                            robot.setClawPAngle(180);
                            robot.setArm(154);
                        })
                        .build();
                objective = Objective.PATH_TO_YELLOW;
            }

            if (objective == Objective.PATH_TO_YELLOW) {
                if (!yReady) {
                    drive.followTrajectory(yellow);
                    yReady = true;
                }

                if ((robot.getArmAngle() > 135) && yReady) {
                    robot.setSlider(390);

                    if (robot.slider.getCurrentPosition() > 380) {
                        timer1.reset();
                        objective = Objective.SCORE_YELLOW;
                    }
                }
            }

            if (objective == Objective.SCORE_YELLOW) {
                if (timer1.milliseconds() > 555) {robot.setArm(0); robot.bothClawClose();}
                else if (timer1.milliseconds() > 300) robot.retractSlider();
                else if (timer1.milliseconds() > 0) robot.leftClawOpen();

                if (robot.getArmAngle() < 5) objective = Objective.PARK;
            }

            if (objective == Objective.PARK) {
                TrajectorySequence park;
                if (parkRight) {
                    park = drive.trajectorySequenceBuilder(nowPose)
                            .lineToConstantHeading(new Vector2d(50.97, -62.18))
                            .addTemporalMarker(() -> objective = Objective.END)
                            .build();
                } else {
                    park = drive.trajectorySequenceBuilder(nowPose)
                            .lineToConstantHeading(new Vector2d(50.97, -11.06))
                            .addTemporalMarker(() -> objective = Objective.END)
                            .build();
                }
                drive.followTrajectorySequence(park);
            }

            if (objective == Objective.END) {
                robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.arm.setPower(0);
            }

            drive.update();
            telemetry.addData("Objective", objective);
            if (parkRight) telemetry.addData("Park", "Right");
            else telemetry.addData("Park", "Left");
            telemetry.addLine();
            telemetry.addData("X", nowPose.getX());
            telemetry.addData("Y", nowPose.getY());
            telemetry.addData("H(D)", Math.toDegrees(nowPose.getHeading()));
            telemetry.addData("H(R)", nowPose.getHeading());
            telemetry.update();
        }
    }

    enum Objective {
        INITIALISE,
        PATH_TO_PURPLE,
        SCORE_PURPLE,
        PATH_TO_YELLOW_GENERATION,
        PATH_TO_YELLOW,
        SCORE_YELLOW,
        PARK,
        END
    }

    class TeamPropPipeline extends OpenCvPipeline {
        Mat YCbCr = new Mat();
        Mat leftCrop, middleCrop, rightCrop;
        double avgLFinal, avgMFinal, avgRFinal;
        double leftTarget, middleTarget, rightTarget;
        Mat output = new Mat();
        Scalar rectColour = new Scalar(0, 0.0, 255.0);

        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("Pipeline running.");

            Rect leftRect = new Rect(0, 100, 100, 79);
            Rect middleRect = new Rect(280, 100, 100, 79);
            Rect rightRect = new Rect(539, 150, 100, 79);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, rectColour, 2);
            Imgproc.rectangle(output, middleRect, rectColour, 2);
            Imgproc.rectangle(output, rightRect, rectColour, 2);

            leftCrop = YCbCr.submat(leftRect);
            middleCrop = YCbCr.submat(middleRect);
            rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop,1);  // Channel 2 = red
            Core.extractChannel(middleCrop, middleCrop, 1);
            Core.extractChannel(rightCrop, rightCrop, 0);

            Scalar leftAverage = Core.mean(leftCrop);
            Scalar middleAverage = Core.mean(middleCrop);
            Scalar rightAverage = Core.mean(rightCrop);

//            leftAverageFinal = Math.abs(leftAverage.val[0] - 105);
//            middleAverageFinal = Math.abs(middleAverage.val[0] - 105);
//            rightAverageFinal = Math.abs(rightAverage.val[0] - 105);

            avgLFinal = Math.abs(leftAverage.val[0] - leftTarget);
            avgMFinal = Math.abs(middleAverage.val[0] - middleTarget);
            avgRFinal = Math.abs(rightAverage.val[0] - rightTarget);

            if ((avgLFinal < avgMFinal) && (avgLFinal < avgRFinal)) {
                telemetry.addLine("left");
                randomizationResult = 1;
            } else if ((avgMFinal < avgLFinal) && (avgMFinal < avgRFinal)) {
                telemetry.addLine("middle");
                randomizationResult = 2;
            } else {
                telemetry.addLine("right");
                randomizationResult = 3;
            }

            if (gamepad1.dpad_left) leftTarget = leftAverage.val[0];
            if (gamepad1.dpad_up) middleTarget = middleAverage.val[0];
            if (gamepad1.dpad_right) rightTarget = rightAverage.val[0];

            telemetry.addData("leftAvg", leftAverage.val[0]);
            telemetry.addData("rightAvg", rightAverage.val[0]);
            telemetry.addData("middleAvg", middleAverage.val[0]);
            telemetry.addLine();
            telemetry.addData("left", avgLFinal);
            telemetry.addData("middle", avgMFinal);
            telemetry.addData("right", avgRFinal);
            telemetry.addData("result", randomizationResult);
            telemetry.addLine();
            telemetry.addData("leftTarget", leftTarget);
            telemetry.addData("middleTarget", middleTarget);
            telemetry.addData("rightTarget", rightTarget);
            telemetry.update();

            return output;
        }
    }
}
