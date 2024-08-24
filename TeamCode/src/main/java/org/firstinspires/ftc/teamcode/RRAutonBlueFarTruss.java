package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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


@Autonomous(name="2+0 BLUE Far, Truss")
public class RRAutonBlueFarTruss extends LinearOpMode {
    Objective objective = Objective.INITIALISE;
    OpenCvWebcam webcam;
    int randomizationResult = 2;
    boolean yReady, scoredPurple;
    boolean scoreRight = true;
    boolean parkLeft;

    TrajectorySequence purple, park;
    Trajectory yellow;
    Vector2d yellowVector;

    @Override
    public void runOpMode() throws InterruptedException {
        Storage.allianceSide = -1;
        Project1Hardware robot = new Project1Hardware();
        robot.init(hardwareMap, telemetry);
        robot.reset();
        robot.retractAndResetSlider(() -> sleep(500));
        robot.bothClawClose();

        ElapsedTime timer1 = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-40.33, 62.80, Math.toRadians(-90.00));
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

        TrajectorySequence purpleL = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-44.61, 44.99, Math.toRadians(135.00)))
                .addTemporalMarker(() -> {
                    timer1.reset();
                    robot.clawPIntake();
                    objective = Objective.SCORE_PURPLE;
                })
                .build();
        TrajectorySequence purpleM = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-44.61, 44.99, Math.toRadians(115.00)))
                .addTemporalMarker(() -> {
                    timer1.reset();
                    robot.clawPIntake();
                    objective = Objective.SCORE_PURPLE;
                })
                .build();
        TrajectorySequence purpleR = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-44.61, 44.99, Math.toRadians(73.00)))
                .addTemporalMarker(() -> {
                    timer1.reset();
                    robot.clawPIntake();
                    objective = Objective.SCORE_PURPLE;
                })
                .build();

        Vector2d yellowLL = new Vector2d(35.20, 39.46);
        Vector2d yellowML = new Vector2d(35.20, 33.01);
        Vector2d yellowRL = new Vector2d(35.20, 23.01);

        Vector2d yellowLR = new Vector2d(35.20, 39.36);
        Vector2d yellowMR = new Vector2d(35.20, 34.01);
        Vector2d yellowRR = new Vector2d(35.20, 23.31);

        waitForStart();
        robot.retractSlider();
        switch (randomizationResult) {
            case 1:
                if (scoreRight) yellowVector = yellowLR; else yellowVector = yellowLL;
                purple = purpleL;
                break;
            default:
            case 2:
                if (scoreRight) yellowVector = yellowMR; else yellowVector = yellowML;
                purple = purpleM;
                break;
            case 3:
                if (scoreRight) yellowVector = yellowRR; else yellowVector = yellowRL;
                purple = purpleR;
                break;
        }

        webcam.stopRecordingPipeline();
        webcam.stopStreaming();
        timer1.reset();

        while (opModeIsActive()) {
            nowPose = drive.getPoseEstimate();

            if (objective == Objective.INITIALISE) {
                objective = Objective.PATH_TO_PURPLE;
            }

            if (objective == Objective.PATH_TO_PURPLE) {
                drive.followTrajectorySequence(purple);
                objective = Objective.TRANSITION_TO_PURPLE;
                timer1.reset();
            }

            if (objective == Objective.TRANSITION_TO_PURPLE) {
                switch (randomizationResult) {
                    case 1: robot.setSlider(650); break;
                    default: case 2: robot.setSlider(500); break;
                    case 3: robot.setSlider(100); break;
                }

                if (robot.sliderInPosition(3) || timer1.milliseconds() > 3000) {
                    objective = Objective.SCORE_PURPLE;
                    timer1.reset();
                }
            }

            if (objective == Objective.SCORE_PURPLE) {
                if (timer1.milliseconds() > 1760 || scoredPurple && timer2.milliseconds() > 300) {
                    robot.arm.setPower(0);
                    robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    objective = Objective.PATH_TO_YELLOW;
                } else if (timer1.milliseconds() > 1360 || scoredPurple) {
                    robot.setClawPAngle(160);
                    robot.retractSlider();

                    yellow = drive.trajectoryBuilder(nowPose)
                            .splineToSplineHeading(
                                    new Pose2d(-32.51, 58.08, Math.toRadians(0.00)),
                                    Math.toRadians(0.00),
                                    SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(17.5)
                            )
                            .splineToConstantHeading(new Vector2d(13.88, 58.08), Math.toRadians(0.00))
                            .splineToConstantHeading(yellowVector, Math.toRadians(0.00))
                            .addSpatialMarker(new Vector2d(13.88, 58.08), () -> robot.setArm(143))
                            .build();
                    robot.bothClawClose();
                } else if (timer1.milliseconds() > 0) {
                    robot.leftClawOpen();
                    scoredPurple = true;
                    timer2.reset();
                }
            }

            if (objective == Objective.PATH_TO_YELLOW) {
                if (!yReady) {
                    drive.followTrajectory(yellow);
                    robot.clawPScoring();
                    yReady = true;
                }

                if ((robot.getArmAngle() > 135) && yReady) {
                    robot.setSlider(655);

                    if (robot.slider.getCurrentPosition() > 635) {
                        timer1.reset();
                        objective = Objective.SCORE_YELLOW;
                    }
                }
            }

            if (objective == Objective.SCORE_YELLOW) {
                if (timer1.milliseconds() > 705) {robot.setArm(0); robot.bothClawClose();}
                else if (timer1.milliseconds() > 600) robot.retractSlider();
                else if (timer1.milliseconds() > 150) robot.rightClawOpen();

                if (robot.getArmAngle() < 5) objective = Objective.PARK;

                if (parkLeft) {
                    park = drive.trajectorySequenceBuilder(nowPose)
                            .lineToLinearHeading(new Pose2d(48.97, 62.18, Math.toRadians(90.00)))
                            .addTemporalMarker(() -> objective = Objective.END)
                            .build();
                } else {
                    park = drive.trajectorySequenceBuilder(nowPose)
                            .lineToLinearHeading(new Pose2d(48.97, 11.06, Math.toRadians(90.00)))
                            .addTemporalMarker(() -> objective = Objective.END)
                            .build();
                }
            }

            if (objective == Objective.PARK) {
                drive.followTrajectorySequence(park);
                timer1.reset();
            }

            if (objective == Objective.END) {
                if (timer1.milliseconds() > 5000) {
                    robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    robot.arm.setPower(0);
                }
                Storage.robotPose = drive.getPoseEstimate();
            }

            drive.update();
            telemetry.addData("Objective", objective);
            if (parkLeft) telemetry.addData("Park", "Left");
            else telemetry.addData("Park", "Right");
            telemetry.addLine();
            telemetry.addData("X", nowPose.getX());
            telemetry.addData("Y", nowPose.getY());
            telemetry.addData("H(D)", Math.toDegrees(nowPose.getHeading()));
            telemetry.addData("H(R)", nowPose.getHeading());
            telemetry.update();
        }
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

            /*Rect middleRect = new Rect(0, 100, 100, 79);
            Rect rightRect = new Rect(280, 100, 100, 79);
            Rect middleRect = new Rect(539, 150, 100, 79);*/

            Rect middleRect = new Rect(200, 130, 85, 64);
            Rect rightRect = new Rect(510, 140, 90, 70);
            //Rect middleRect = new Rect(539, 150, 100, 79);

            input.copyTo(output);
            Imgproc.rectangle(output, middleRect, rectColour, 2);
            Imgproc.rectangle(output, rightRect, rectColour, 2);
            //Imgproc.rectangle(output, middleRect, rectColour, 2);

            rightCrop = YCbCr.submat(rightRect);
            middleCrop = YCbCr.submat(middleRect);
            //rightCrop = YCbCr.submat(middleRect);

            Core.extractChannel(rightCrop, rightCrop,1);  // Channel 2 = red
            Core.extractChannel(middleCrop, middleCrop, 1);
            //Core.extractChannel(rightCrop, rightCrop, 0);

            Scalar rightAverage = Core.mean(rightCrop);
            Scalar middleAverage = Core.mean(middleCrop);
            //Scalar rightAverage = Core.mean(rightCrop);

//            leftAverageFinal = Math.abs(leftAverage.val[0] - 105);
//            middleAverageFinal = Math.abs(middleAverage.val[0] - 105);
//            rightAverageFinal = Math.abs(rightAverage.val[0] - 105);

            avgRFinal = Math.abs(rightAverage.val[0] - rightTarget);
            avgMFinal = Math.abs(middleAverage.val[0] - middleTarget);
            //avgRFinal = Math.abs(rightAverage.val[0] - rightTarget);

            /*if ((avgLFinal < avgMFinal) && (avgLFinal < avgRFinal)) {
                telemetry.addLine("LEFT");
                randomizationResult = 1;
            } else if ((avgMFinal < avgLFinal) && (avgMFinal < avgRFinal)) {
                telemetry.addLine("MIDDLE");
                randomizationResult = 2;
            } else {
                telemetry.addLine("RIGHT");
                randomizationResult = 3;
            }*/

            if (avgRFinal > 5 && avgMFinal > 5) {
                telemetry.addLine("LEFT");
                randomizationResult = 1;
            } else if (avgRFinal < avgMFinal) {
                telemetry.addLine("RIGHT");
                randomizationResult = 3;
            } else if (avgMFinal < avgRFinal) {
                telemetry.addLine("MIDDLE");
                randomizationResult = 2;
            }

            if (scoreRight) telemetry.addData("Score on", "Right");
            else telemetry.addData("Score on", "Left");
            if (parkLeft) telemetry.addData("Park", "Left");
            else telemetry.addData("Park", "Right");
            telemetry.addLine();

            if (gamepad1.dpad_right) rightTarget = rightAverage.val[0];
            if (gamepad1.dpad_up) middleTarget = middleAverage.val[0];
            //if (gamepad1.dpad_right) rightTarget = rightAverage.val[0];

            if (gamepad1.square) parkLeft = true;
            else if (gamepad1.circle) parkLeft = false;
            if (gamepad1.left_bumper) scoreRight = false;
            else if (gamepad1.right_bumper) scoreRight = true;

            telemetry.addData("rightAvg", rightAverage.val[0]);
            //telemetry.addData("leftAvg", leftAverage.val[0]);
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

    enum Objective {
        INITIALISE,
        PATH_TO_PURPLE,
        TRANSITION_TO_PURPLE,
        SCORE_PURPLE,
        PATH_TO_YELLOW,
        SCORE_YELLOW,
        PARK,
        END
    }
}