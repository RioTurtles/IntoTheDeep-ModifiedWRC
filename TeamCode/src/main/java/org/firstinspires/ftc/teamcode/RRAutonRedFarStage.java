package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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

@Autonomous(group="new")
public class RRAutonRedFarStage extends LinearOpMode {
    Objective objective = Objective.INITIALISE;
    OpenCvWebcam webcam;
    int randomizationResult = 2;
    boolean yReady;

    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot = new Project1Hardware();
        robot.init(hardwareMap, telemetry);
        robot.reset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-40.33, -62.80, Math.toRadians(90.00));

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
                .lineToSplineHeading(new Pose2d(-39.03, -11.83, Math.toRadians(50.00)))
                .build();
        TrajectorySequence pMiddle = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-34.18, -12.02, Math.toRadians(60.00)))
                .build();
        TrajectorySequence pRight = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-39.03, -11.83, Math.toRadians(130.00)))
                .build();
        Trajectory yLeft = drive.trajectoryBuilder(pLeft.end())
                .lineToSplineHeading(new Pose2d(23.94, -13.51, Math.toRadians(0.00)))
                .splineToConstantHeading(new Vector2d(51.32, -29.71), Math.toRadians(-45.00))
                .build();
        Trajectory yMiddle = drive.trajectoryBuilder(pMiddle.end())
                .lineToLinearHeading(new Pose2d(23.94, -13.51, Math.toRadians(0.00)))
                .splineToConstantHeading(new Vector2d(50.02, -35.49), Math.toRadians(-50.00))
                .build();
        Trajectory yRight = drive.trajectoryBuilder(pRight.end())
                .lineToSplineHeading(new Pose2d(23.94, -13.51, Math.toRadians(0.00)))
                .splineToConstantHeading(new Vector2d(51.32, -42.38), Math.toRadians(-60.00))
                .build();
        TrajectorySequence rLeft = drive.trajectorySequenceBuilder(yLeft.end()).build();
        TrajectorySequence rMiddle = drive.trajectorySequenceBuilder(yMiddle.end()).build();
        TrajectorySequence rRight = drive.trajectorySequenceBuilder(yRight.end()).build();

        waitForStart();
        webcam.stopRecordingPipeline();
        webcam.stopStreaming();

        while (opModeIsActive()) {
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
                robot.rightClawOpen();
                sleep(1000);
                objective = Objective.PATH_TO_YELLOW;
            }

            if (objective == Objective.PATH_TO_YELLOW) {
                switch (randomizationResult) {
                    case 1: drive.followTrajectory(yLeft); break;
                    default: case 2: drive.followTrajectory(yMiddle); break;
                    case 3: drive.followTrajectory(yRight); break;
                }

                if ((robot.getArmAngle() > 150) && yReady) objective = Objective.SCORE_YELLOW;
            }

            if (objective == Objective.SCORE_YELLOW) {
                robot.leftClawOpen();
                sleep(1500);
                robot.setArm(0);
                if (robot.getArmAngle() < 5) objective = Objective.PARK;
            }

            if (objective == Objective.PARK) {
                switch (randomizationResult) {
                    case 1: drive.followTrajectorySequence(rLeft);
                    default: case 2: drive.followTrajectorySequence(rMiddle);
                    case 3: drive.followTrajectorySequence(rRight);
                }
            }

            if (objective == Objective.END) {
                robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.arm.setPower(0);
            }

            drive.update();
            telemetry.addData("Objective", objective);
            telemetry.update();
        }
    }

    enum Objective {
        INITIALISE,
        PATH_TO_PURPLE,
        SCORE_PURPLE,
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
