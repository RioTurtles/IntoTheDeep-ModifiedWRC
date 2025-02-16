package org.firstinspires.ftc.teamcode.archive.preapoc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.archive.apoc_wrc.Project1Hardware;
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
@Autonomous(name="v2 Team Prop EOCV")
public class TeamProp_v2 extends OpMode {
    Project1Hardware robot = new Project1Hardware();
    OpenCvWebcam webcam = null;
    int result = 0;

    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class,"Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        webcam.setPipeline(new TeamPropPipeline());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }

    @Override
    public void loop() {
        telemetry.addData("team prop location", result);
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
            Rect leftRect = new Rect(1, 100, 100, 79);
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
                telemetry.addLine("left");
            } else if ((middleAverageFinal < leftAverageFinal) &&  (middleAverageFinal < rightAverageFinal)) {
                telemetry.addLine("middle");
            } else {
                telemetry.addLine("right");
            }
            telemetry.addData("left", leftAverageFinal);
            telemetry.addData("middle", middleAverageFinal);
            telemetry.addData("right", rightAverageFinal);

            telemetry.update();

            return output;
        }
    }
}
