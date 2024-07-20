/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp(name="AutoBackdrop, AprilTags")
public class AutoBackdropAprilTag extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    AprilTagDetection targetTag;
    Hardware_v3 robot;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int numFramesWithoutDetection = 0;
    int targetID = 2;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    double errorX, errorY, errorZ, errorH;
//    final static double LEFT_X = -0.09;
//    final static double LEFT_Y = 0.12;
//    final static double LEFT_Z = 1.09;
//    final static double CENTRE_X = -0.08;
//    final static double CENTRE_Y = 0.12;
//    final static double CENTRE_Z = 1.07;
//    final static double RIGHT_X = -0.37;
//    final static double RIGHT_Y = 0.13;
//    final static double RIGHT_Z = 1.09;

    private final static double LEFT_X = 0;
    private final static double LEFT_Y = 0;
    private final static double LEFT_Z = 0;
    private final static double CENTRE_X = 0;
    private final static double CENTRE_Y = 0;
    private final static double CENTRE_Z = 0;
    private final static double RIGHT_X = 0;
    private final static double RIGHT_Y = 0;
    private final static double RIGHT_Z = 0;
    private final static double kPX = 0.9, kPY = 0.1, kPZ = 0.4, kPH = 0.7;
    private final static double kIX = 0.0001, kIY = 0.006, kIZ = 0.0001 ;
    private final static double kDX = 0.2, kDY = 0.5, kDZ = 0.5;
    private double lastErrorX, lastErrorY, lastErrorZ;
    private double integralX, integralY, integralZ;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        @SuppressLint("DiscouragedApi") int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        robot = new Hardware_v3();
        robot.init(hardwareMap, telemetry);
        robot.reset();

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {}
        });

        waitForStart();
        robot.imu.resetYaw();
        telemetry.setMsTransmissionInterval(50);

        while (opModeIsActive()) {
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            telemetry.addData("selected", targetID);
            telemetry.addLine();
            telemetry.addData("errorX", errorX);
            telemetry.addData("errorY", errorY);
            telemetry.addData("errorZ", errorZ);
            telemetry.addData("errorH", errorH);

            if (gamepad1.square) targetID = 1;
            if (gamepad1.triangle) targetID = 2;
            if (gamepad1.circle) targetID = 3;

            // If there's been a new frame...
            if (detections != null) {
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                // If we don't see any tags
                if (detections.size() == 0) {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                } else {  // We do see tags!
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    for (AprilTagDetection detection : detections) {
//                        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
                        if (detection.id == targetID) {targetTag = detection; break;}

                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                    }
                }
            }

            if (gamepad1.cross) {
                double heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                switch (targetID) {
                    case 1:
                        errorX = targetTag.pose.x - LEFT_X;
                        errorY = targetTag.pose.y - LEFT_Y;
                        errorZ = targetTag.pose.z - LEFT_Z;
                        break;
                    case 3:
                        errorX = targetTag.pose.x - RIGHT_X;
                        errorY = targetTag.pose.y - RIGHT_Y;
                        errorZ = targetTag.pose.z - RIGHT_Z;
                        break;
                    case 2:
                    default:
                        errorX = targetTag.pose.x - CENTRE_X;
                        errorY = targetTag.pose.y - CENTRE_Y;
                        errorZ = targetTag.pose.z - CENTRE_Z;
                        break;
                }

                integralX += errorX * 0.02;
                double dX = (errorX - lastErrorX) / 0.02;
                double outputX = errorX * kPX + integralX * kIX + dX * kDX;
                lastErrorX = errorX;

                integralY += errorY * 0.02;
                double dY = (errorY - lastErrorY) / 0.02;
                double outputY = errorY * kPY + integralY * kIY + dY * kDY;
                lastErrorY = errorY;

                integralZ += errorZ * 0.02;
                double dZ = (errorZ - lastErrorZ) / 0.02;
                double outputZ = errorZ * kPZ + integralZ * kIZ + dZ * kDZ;
                lastErrorZ = errorZ;

                // Limit values
                double vertical = Math.min(outputY + outputZ, 0.5);
                outputX = Math.min(outputX, 0.7);

                errorH = heading;
                robot.remote(vertical, outputX, -heading * kPH, heading);
                // yeehaw
                // when i wrote this code, only god and i knew what we were doing
                // now, only god knows
            } else if (gamepad1.right_bumper) {
                robot.motorFL.setPower(0);
                robot.motorFR.setPower(0);
                robot.motorBL.setPower(0);
                robot.motorBR.setPower(0);
            } else {
                double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double lx = gamepad1.left_stick_x * Math.cos(-botHeading) + gamepad1.left_stick_y * Math.sin(-botHeading);
                double ly = gamepad1.left_stick_x * Math.sin(-botHeading) - gamepad1.left_stick_y * Math.cos(-botHeading);
                double denominator = Math.max(abs(gamepad1.left_stick_x) + abs(gamepad1.left_stick_y) + abs(gamepad1.right_stick_x), 1);
                robot.motorFL.setPower((lx + ly + gamepad1.right_stick_x) / denominator);
                robot.motorBL.setPower((-lx + ly + gamepad1.right_stick_x) / denominator);
                robot.motorFR.setPower((ly - lx - gamepad1.right_stick_x) / denominator);
                robot.motorBR.setPower((lx + ly - gamepad1.right_stick_x) / denominator);
            }

            if (gamepad1.touchpad) robot.imu.resetYaw();

            telemetry.update();
            sleep(20);
        }
    }
}
