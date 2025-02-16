package org.firstinspires.ftc.teamcode.archive.apoc_wrc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

@Disabled
@Autonomous(name="2+0 BLUE Close")
public class RRAutonBlueClose extends LinearOpMode {
    Objective objective = Objective.INITIALISE;
    OpenCvWebcam webcam;
    int randomizationResult = 2;
    boolean yReady, scoredPurple;
    boolean scoreLeft = true;
    boolean parkLeft = true;

    TrajectorySequence purple, yellow, park;
    TrajectorySequence yellowLL, yellowML, yellowRL;
    TrajectorySequence yellowLR, yellowMR, yellowRR;

    @Override
    public void runOpMode() throws InterruptedException {
        Storage.allianceSide = -1;
        Project1Hardware robot = new Project1Hardware();
        robot.init(hardwareMap, telemetry);
        robot.reset();
        robot.retractAndResetSlider(() -> sleep(500));
        robot.bothClawClose();

        ElapsedTime timer1 = new ElapsedTime();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(15.43, 62.80, Math.toRadians(-90.00));
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
                .lineToSplineHeading(new Pose2d(35.20, 32.01, Math.toRadians(0.00)))
                .addTemporalMarker(() -> {robot.clawPIntake(); objective = Objective.SCORE_PURPLE;})
                .build();
        TrajectorySequence purpleM = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(35.20, 25.51, Math.toRadians(0.00)))
                .addTemporalMarker(() -> {robot.clawPIntake(); objective = Objective.SCORE_PURPLE;})
                .build();
        TrajectorySequence purpleR = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(32.45, 32.01, Math.toRadians(0.00)))
                .addTemporalMarker(() -> {robot.clawPIntake(); objective = Objective.SCORE_PURPLE;})
                .build();

        yellowLL = drive.trajectorySequenceBuilder(purpleL.end())
                .lineToConstantHeading(new Vector2d(35.20, 41.71))
                .addTemporalMarker(() -> yReady = true)
                .build();
        yellowML = drive.trajectorySequenceBuilder(purpleM.end())
                .lineToSplineHeading(new Pose2d(35.20, 36.01, Math.toRadians(0.00)))
                .addTemporalMarker(() -> yReady = true)
                .build();
        yellowRL = drive.trajectorySequenceBuilder(purpleR.end())
                .lineToConstantHeading(new Vector2d(35.20, 28.11))
                .addTemporalMarker(() -> yReady = true)
                .build();

        yellowLR = drive.trajectorySequenceBuilder(purpleL.end())
                .lineToConstantHeading(new Vector2d(35.20, 37.51))
                .addTemporalMarker(() -> yReady = true)
                .build();
        yellowMR = drive.trajectorySequenceBuilder(purpleM.end())
                .lineToConstantHeading(new Vector2d(35.20, 30.81))
                .addTemporalMarker(() -> yReady = true)
                .build();
        yellowRR = drive.trajectorySequenceBuilder(purpleR.end())
                .lineToConstantHeading(new Vector2d(35.20, 26.11))
                .addTemporalMarker(() -> yReady = true)
                .build();

        waitForStart();
        robot.retractSlider();
        switch (randomizationResult) {
            case 1:
                if (scoreLeft) yellow = yellowLL; else yellow = yellowLR;
                purple = purpleL;
                break;
            default:
            case 2:
                if (scoreLeft) yellow = yellowML; else yellow = yellowMR;
                purple = purpleM;
                break;
            case 3:
                if (scoreLeft) yellow = yellowRL; else yellow = yellowRR;
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
                timer1.reset();
            }

            if (objective == Objective.SCORE_PURPLE) {
                switch (randomizationResult) {
                    case 1: robot.setSlider(0); break;
                    default: case 2: robot.setSlider(400); break;
                    case 3: robot.setSlider(875); break;
                }

                if ((timer1.milliseconds() > 1500) || scoredPurple) {
                    timer1.reset();
                    objective = Objective.TRANSITION_TO_YELLOW;
                } else if (robot.sliderInPosition(5) && timer1.milliseconds() > 500 || timer1.milliseconds() > 1200
                ) {robot.leftClawOpen(); scoredPurple = true;}
            }

            if (objective == Objective.TRANSITION_TO_YELLOW) {
                if (timer1.milliseconds() > 100) {
                    drive.followTrajectorySequence(yellow);
                } else if (timer1.milliseconds() > 0) {
                    robot.retractSlider();
                    robot.setArm(148);
                }

                if (robot.getArmAngle() > 125) robot.setSlider(520);

                if (robot.slider.getCurrentPosition() > 515 && yReady) {
                    robot.clawPScoring();
                    timer1.reset();
                    objective = Objective.SCORE_YELLOW;
                }

                robot.setClawPAngle(160);
            }

            if (objective == Objective.SCORE_YELLOW) {
                if (timer1.milliseconds() > 600) {
                    timer1.reset();
                    objective = Objective.TRANSITION_TO_PARK;
                } else if (timer1.milliseconds() > 300) robot.rightClawOpen();
            }

            if (objective == Objective.TRANSITION_TO_PARK) {
                robot.setArm(0);
                robot.setClawPAngle(180);
                robot.retractSlider();
                if (robot.getArmAngle() < 130) robot.bothClawClose();
                if (robot.getArmAngle() < 5) objective = Objective.PARK;

                if (parkLeft) {
                    park = drive.trajectorySequenceBuilder(nowPose)
                            .lineToLinearHeading(new Pose2d(48.97, 62.18, Math.toRadians(-90.00)))
                            .addTemporalMarker(() -> objective = Objective.END)
                            .build();
                } else {
                    park = drive.trajectorySequenceBuilder(nowPose)
                            .lineToLinearHeading(new Pose2d(48.97, 11.06, Math.toRadians(-90.00)))
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
            if (parkLeft) telemetry.addData("Park", "Right");
            else telemetry.addData("Park", "Left");
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

            /*Rect leftRect = new Rect(0, 100, 100, 79);
            Rect middleRect = new Rect(280, 100, 100, 79);
            Rect rightRect = new Rect(539, 150, 100, 79);*/

            Rect leftRect = new Rect(70, 130, 85, 64);
            Rect middleRect = new Rect(375, 140, 90, 70);
            //Rect rightRect = new Rect(539, 150, 100, 79);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, rectColour, 2);
            Imgproc.rectangle(output, middleRect, rectColour, 2);
            //Imgproc.rectangle(output, rightRect, rectColour, 2);

            leftCrop = YCbCr.submat(leftRect);
            middleCrop = YCbCr.submat(middleRect);
            //rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop,2);  // Channel 2 = red
            Core.extractChannel(middleCrop, middleCrop, 2);
            //Core.extractChannel(rightCrop, rightCrop, 0);

            Scalar leftAverage = Core.mean(leftCrop);
            Scalar middleAverage = Core.mean(middleCrop);
            //Scalar rightAverage = Core.mean(rightCrop);

//            leftAverageFinal = Math.abs(leftAverage.val[0] - 105);
//            middleAverageFinal = Math.abs(middleAverage.val[0] - 105);
//            rightAverageFinal = Math.abs(rightAverage.val[0] - 105);

            avgLFinal = Math.abs(leftAverage.val[0] - leftTarget);
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

            if (avgLFinal > 5 && avgMFinal > 5) {
                telemetry.addLine("RIGHT");
                randomizationResult = 3;
            } else if (avgLFinal < avgMFinal) {
                telemetry.addLine("LEFT");
                randomizationResult = 1;
            } else if (avgMFinal < avgLFinal) {
                telemetry.addLine("MIDDLE");
                randomizationResult = 2;
            }

            if (scoreLeft) telemetry.addData("Score on", "Left");
            else telemetry.addData("Score on", "Right");
            if (parkLeft) telemetry.addData("Park", "Left");
            else telemetry.addData("Park", "Right");
            telemetry.addLine();

            if (gamepad1.dpad_left) leftTarget = leftAverage.val[0];
            if (gamepad1.dpad_up) middleTarget = middleAverage.val[0];
            //if (gamepad1.dpad_right) rightTarget = rightAverage.val[0];

            if (gamepad1.square) parkLeft = true;
            else if (gamepad1.circle) parkLeft = false;
            if (gamepad1.left_bumper) scoreLeft = true;
            else if (gamepad1.right_bumper) scoreLeft = false;

            telemetry.addData("leftAvg", leftAverage.val[0]);
            //telemetry.addData("rightAvg", rightAverage.val[0]);
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
        SCORE_PURPLE,
        TRANSITION_TO_YELLOW,
        SCORE_YELLOW,
        TRANSITION_TO_PARK,
        PARK,
        END
    }
}
