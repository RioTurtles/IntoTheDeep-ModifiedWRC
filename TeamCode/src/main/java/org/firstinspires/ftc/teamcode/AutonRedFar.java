package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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

@Autonomous(name="Red Far")
public class AutonRedFar extends LinearOpMode {
    int randomizationResult;
    OpenCvWebcam webcam = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Hardware_v3 robot = new Hardware_v3();
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.init(hardwareMap, telemetry);
        robot.reset();
        robot.closeUpperClaw();
        robot.closeLowerClaw();

        int stage = 0;
        Pose2d positionStarting = new Pose2d(-39, -64, Math.toRadians(90));

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        @SuppressLint("DiscouragedApi") int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        webcam.setPipeline(new TeamPropPipeline());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);}

            @Override
            public void onError(int errorCode) {}
        });

        waitForStart();
        webcam.stopRecordingPipeline();
        webcam.stopStreaming();
        drive.setPoseEstimate(positionStarting);

        while (opModeIsActive()) {
            TrajectorySequence trajectoryLeft = drive.trajectorySequenceBuilder(positionStarting)
                    .lineToSplineHeading(new Pose2d(-41, -32, Math.toRadians(180)))
                    .addDisplacementMarker(() -> {
                        robot.openLowerClaw();
                        robot.closeUpperClaw();
                        telemetry.addData("Purple pixel scored", true);
                        telemetry.update();
                    })
                    .lineToConstantHeading(new Vector2d(-33.75, -13.00))
                    .lineToConstantHeading(new Vector2d(36.00, -13.00))
                    .build();

            TrajectorySequence trajectoryYellowLeft = drive.trajectorySequenceBuilder(trajectoryLeft.end())
                    .lineToConstantHeading(new Vector2d(51.00, -32))
                    .build();

            TrajectorySequence trajectoryCentre = drive.trajectorySequenceBuilder(positionStarting)
                    .lineToSplineHeading(new Pose2d(-39, -19, Math.toRadians(270)))
                    .addDisplacementMarker(() -> {
                        robot.openLowerClaw();
                        robot.closeUpperClaw();
                        telemetry.addData("Purple pixel scored", true);
                        telemetry.update();
                    })
                    .back(5)
                    .lineToSplineHeading(new Pose2d(36.00, -13.00, Math.toRadians(180)))
                    .build();

            TrajectorySequence trajectoryYellowCentre = drive.trajectorySequenceBuilder(trajectoryCentre.end())
                    .lineToSplineHeading(new Pose2d(51.00, -39.0, Math.toRadians(180)))
                    .build();

            TrajectorySequence trajectoryRight = drive.trajectorySequenceBuilder(positionStarting)
                    .splineToSplineHeading(new Pose2d(-31.6, -38.15, Math.toRadians(30.00)), Math.toRadians(30.00))
                    .addDisplacementMarker(() -> {
                        robot.openLowerClaw();
                        robot.closeUpperClaw();
                        telemetry.addData("Purple pixel scored", true);
                        telemetry.update();
                    })
                    .back(10)
                    .lineToSplineHeading(new Pose2d(-35.67, -13.00, Math.toRadians(180)))
                    .lineTo(new Vector2d(36.11, -13.00))
                    .build();

            TrajectorySequence trajectoryYellowRight = drive.trajectorySequenceBuilder(trajectoryRight.end())
                    .lineToConstantHeading(new Vector2d(51.00, -42.00))
                    .build();

            TrajectorySequence trajectoryCycleLeft = drive.trajectorySequenceBuilder(trajectoryYellowLeft.end())
                    .lineToConstantHeading(new Vector2d(26.00, -12.00))
                    .lineToConstantHeading(new Vector2d(-62.00, -12.00))
                    .lineToConstantHeading(new Vector2d(26.00, -12.00))
                    .build();

            TrajectorySequence trajectoryCycleCentre = drive.trajectorySequenceBuilder(trajectoryYellowCentre.end())
                    .lineToConstantHeading(new Vector2d(26.00, -12.00))
                    .lineToConstantHeading(new Vector2d(-62.00, -12.00))
                    .lineToConstantHeading(new Vector2d(26.00, -12.00))
                    .build();

            TrajectorySequence trajectoryCycleRight = drive.trajectorySequenceBuilder(trajectoryYellowRight.end())
                    .lineToConstantHeading(new Vector2d(26.00, -12.00))
                    .lineToConstantHeading(new Vector2d(-62.00, -12.00))
                    .lineToConstantHeading(new Vector2d(26.00, -12.00))
                    .build();

            TrajectorySequence trajectoryCycleScoreLeft = drive.trajectorySequenceBuilder(trajectoryCycleLeft.end())
                    .lineToConstantHeading(new Vector2d(51, -32))
                    .build();

            TrajectorySequence trajectoryCycleScoreCentre = drive.trajectorySequenceBuilder(trajectoryCycleCentre.end())
                    .lineToConstantHeading(new Vector2d(51, -38))
                    .build();

            TrajectorySequence trajectoryCycleScoreRight = drive.trajectorySequenceBuilder(trajectoryCycleRight.end())
                    .lineToConstantHeading(new Vector2d(51, -46))
                    .build();

            TrajectorySequence trajectoryParkLeft = drive.trajectorySequenceBuilder(trajectoryCycleScoreLeft.end())
                    .lineToConstantHeading(new Vector2d(51, -13))
                    .build();

            TrajectorySequence trajectoryParkCentre = drive.trajectorySequenceBuilder(trajectoryCycleScoreCentre.end())
                    .lineToConstantHeading(new Vector2d(51, -13))
                    .build();

            TrajectorySequence trajectoryParkRight = drive.trajectorySequenceBuilder(trajectoryCycleScoreRight.end())
                    .lineToConstantHeading(new Vector2d(51, -13))
                    .build();

            if (stage == 0) {
                switch (randomizationResult) {
                    case 1: drive.followTrajectorySequence(trajectoryLeft); break;
                    case 2: drive.followTrajectorySequence(trajectoryCentre); break;
                    case 3: drive.followTrajectorySequence(trajectoryRight); break;
                }
                stage = 1;
            }

            if (stage == 1) {
                sleep(1000);
                stage = 2;
            }

            if (stage == 2) {
                robot.setSliderPosition(1);
                switch (randomizationResult) {
                    case 1:drive.followTrajectorySequence(trajectoryYellowLeft); break;
                    case 2: drive.followTrajectorySequence(trajectoryYellowCentre); break;
                    case 3: drive.followTrajectorySequence(trajectoryYellowRight); break;
                }
                stage = 3;
            }

            if (stage == 3) {
                sleep(400);
                robot.setSliderPosition(1);
                sleep(1000);
                robot.setSliderPosition(0);
                stage = 4;
            }

            if (stage == 4) {
                switch (randomizationResult) {
                    case 1: drive.followTrajectorySequence(trajectoryCycleLeft); break;
                    case 2: drive.followTrajectorySequence(trajectoryCycleCentre); break;
                    case 3: drive.followTrajectorySequence(trajectoryCycleRight); break;
                }
                stage = 5;
            }

            if (stage == 5) {
                robot.setSliderPosition(1);
                switch (randomizationResult) {
                    case 1: drive.followTrajectorySequence(trajectoryCycleScoreLeft); break;
                    case 2: drive.followTrajectorySequence(trajectoryCycleScoreCentre); break;
                    case 3: drive.followTrajectorySequence(trajectoryCycleScoreRight); break;
                }
                stage = 6;
            }

            if (stage == 6) {
                robot.setSliderPosition(0);
                switch (randomizationResult) {
                    case 1: drive.followTrajectorySequence(trajectoryParkLeft); break;
                    case 2: drive.followTrajectorySequence(trajectoryParkCentre); break;
                    case 3: drive.followTrajectorySequence(trajectoryParkRight); break;
                }
                stage = 7;
            }

            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("stage", stage);
            telemetry.addLine();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }

    }

    class TeamPropPipeline extends OpenCvPipeline {
        Mat YCbCr = new Mat();
        Mat leftCrop, middleCrop, rightCrop;
        double leftAverageFinal, middleAverageFinal, rightAverageFinal;
        Mat output = new Mat();
        Scalar rectColour = new Scalar(255.0, 0.0, 0.0);

        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("Pipeline running.");

            // TODO: tune values when new car
            Rect leftRect = new Rect(30, 100, 100, 79);
            Rect middleRect = new Rect(300, 100, 100, 79);
            Rect rightRect = new Rect(539, 100, 100, 79);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, rectColour, 2);
            Imgproc.rectangle(output, middleRect, rectColour, 2);
            Imgproc.rectangle(output, rightRect, rectColour, 2);

            leftCrop = YCbCr.submat(leftRect);
            middleCrop = YCbCr.submat(middleRect);
            rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop, 2);  // Channel 2 = red
            Core.extractChannel(middleCrop, middleCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);

            Scalar leftAverage = Core.mean(leftCrop);
            Scalar middleAverage = Core.mean(middleCrop);
            Scalar rightAverage = Core.mean(rightCrop);

            leftAverageFinal = Math.abs(leftAverage.val[0] - 100);
            middleAverageFinal = Math.abs(middleAverage.val[0] - 100);
            rightAverageFinal = Math.abs(rightAverage.val[0] - 100);

            if ((leftAverageFinal < middleAverageFinal) && (leftAverageFinal < rightAverageFinal)) {
                telemetry.addLine("1 - LEFT");
                randomizationResult = 1;
            } else if ((middleAverageFinal < leftAverageFinal) &&  (middleAverageFinal < rightAverageFinal)) {
                telemetry.addLine("2 - MIDDLE");
                randomizationResult = 2;
            } else {
                telemetry.addLine("3 - RIGHT");
                randomizationResult = 3;
            }

            telemetry.addLine();
            telemetry.addData("L", leftAverageFinal);
            telemetry.addData("M", middleAverageFinal);
            telemetry.addData("R", rightAverageFinal);

            telemetry.update();

            return output;
        }
    }
}
