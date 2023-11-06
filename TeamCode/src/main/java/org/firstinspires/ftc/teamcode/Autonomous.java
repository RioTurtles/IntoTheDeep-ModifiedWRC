package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    AprilTagDetection tagOfInterest = null;
    final double fx = 578.272;
    final double fy = 578.272;
    final double cx = 402.145;
    final double cy = 221.506;
    final double tagSize = 0.166;
    final int APRILTAG_LEFT = 1;
    final int APRILTAG_CENTER = 2;
    final int APRILTAG_RIGHT = 3;
    int k = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewID);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        // INIT-Loop, replaces waitForStart()
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag: currentDetections) {
                    if (tag.id == APRILTAG_LEFT || tag.id == APRILTAG_CENTER || tag.id == APRILTAG_RIGHT) {
                        // Tag found!
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest seen, location data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Tag of interest not seen");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(never seen)");
                        k++;
                        telemetry.addData("k", k);
                        telemetry.update();
                    } else {
                        telemetry.addLine("seen before, at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            } else {
                telemetry.addLine("Tag of interest not seen");

                if (tagOfInterest == null) {
                    telemetry.addLine("(never seen)");
                } else {
                    telemetry.addLine("seen before, at:");
                    tagToTelemetry(tagOfInterest);
                }
            }

            telemetry.update();
            sleep(20);
        }

        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, never sighted during init-loop");
            telemetry.update();
        }

        telemetry.update();

    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nTag ID %d", detection.id));
        telemetry.addLine(String.format("Tag X: %.2f ft", detection.pose.x*Constants.feetPerMetre));
        telemetry.addLine(String.format("Tag Y: %.2f ft", detection.pose.y*Constants.feetPerMetre));
        telemetry.addLine(String.format("Tag Z: %.2f ft", detection.pose.z*Constants.feetPerMetre));
    }
}
