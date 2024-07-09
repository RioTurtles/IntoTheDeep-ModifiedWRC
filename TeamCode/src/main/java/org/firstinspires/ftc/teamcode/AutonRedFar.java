package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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

@Autonomous(name="RedFar")
public class AutonRedFar extends LinearOpMode {

    public enum states {
        INIT,
        GROUND,
        GROUND_EXTEND,
        GROUND_GRIP,
        EXTEND_GRIP,
        READY_SCORE,
        SCORING,
        RETURN_TO_INIT,
        SIMPLE_SCORING
    }

    //Teleop_v2.states state = Teleop_v2.states.INIT;
    OpenCvWebcam webcam = null;
    int randomizationResult = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot = new Project1Hardware();
        MecanumDrive drivetrain = new MecanumDrive(robot);
        robot.init(hardwareMap, telemetry);
        robot.reset();

        // Variables
        double xTarget = 0;
        double yTarget = 0;
        double headingTarget = 0;
        double botHeading;
        double left_x;
        double left_y;
        double rot_x;
        double lx;
        double ly;
        double denominator;
        double error1;
        double lastError1=0;
        double integral1=0;
        double kp1 = 0.16;
        double ki1 =0.02;


        double kd1 =0.6         ;
        double error2;
        double lastError2=0;
        double integral2=0;
        double kp2 = 0.05;
        double ki2 =0;
        double kd2 =0;
        double error3;
        double lastError3 = 0;
        double integral3=0;
        double kp3 = 1;
        double ki3=0.001;
        double kd3 = 1;




        int moveStep = 1;
        ElapsedTime timer1 = new ElapsedTime();
        ElapsedTime auton30 = new ElapsedTime();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        drive.setPoseEstimate(new Pose2d(-36, -63, Math.toRadians(270)));
        robot.bothClawClose();
        robot.retractSlider();

        waitForStart();

        webcam.stopRecordingPipeline();
        webcam.stopStreaming();
        /*while(!gamepad1.share){
            if(gamepad1.right_bumper) {
                if (gamepad1.triangle) {
                    kp1 += 0.01;
                }
                if (gamepad1.cross) {
                    kp1 += -0.01;
                }
                if (gamepad1.dpad_up){
                    kd1 += 0.01;
                }
                if (gamepad1.dpad_down){
                    kd1 += -0.01;
                }
                if(gamepad1.dpad_right){
                    ki1+=0.01;
                }
                if(gamepad1.dpad_left){
                    ki1+=-0.01;
                }
            } else {
                if (gamepad1.triangle) {
                    kp3 += 0.01;
                }
                if (gamepad1.cross) {
                    kp3 += -0.01;
                }
                if (gamepad1.dpad_up){
                    kd3 += 0.01;
                }
                if (gamepad1.dpad_down){
                    kd3 += -0.01;
                }
                if(gamepad1.dpad_right){
                    ki3+=0.01;
                }
                if(gamepad1.dpad_left){
                    ki3+=-0.01;
                }
            }


            telemetry.addData("kp1",kp1);
            telemetry.addData("ki1",ki1);
            telemetry.addData("kd1",kd1);
            telemetry.addData("kp3",kp3);
            telemetry.addData("ki3",ki3);
            telemetry.addData("kd3",kd3);

        }

         */
        kp2=kp1;
        ki2=ki1;
        kd2=kd1;
        while (opModeIsActive()) {
            Pose2d poseEstimate = drive.getPoseEstimate();
            // heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


            // Initial movement
            if (moveStep == 1) {
                robot.bothClawClose();
                headingTarget = 270;

                xTarget = -36;
                yTarget = -36;


                if ((Math.abs(poseEstimate.getX() - xTarget) > 1) || (Math.abs(poseEstimate.getY() - yTarget) > 2)) {timer1.reset();}
                if (timer1.milliseconds() > 300) {
                    moveStep = 2;

                    timer1.reset();
                }
            }

            // Rotate (purple pixel)
            if (moveStep == 2) {
                xTarget = -37;

                robot.bothClawClose();
                robot.arm.setPower(0);
                robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                if (randomizationResult == 1) {
                    headingTarget = 0;

                } else if (randomizationResult == 2) {
                    yTarget = -15;
                    headingTarget = 90;

                } else if (randomizationResult == 3) {
                    headingTarget = 210;
                }

                /*if (robot.getArmAngle() > 50) {
                    robot.setClawPAngle(180-robot.getArmAngle()+15);
                }*/

                if ((Math.abs(poseEstimate.getX() - xTarget) > 1) || (Math.abs(poseEstimate.getY() - yTarget) > 1)) {timer1.reset();}

                if (timer1.milliseconds() > 400) {
                    integral1 = 0;
                    integral2 = 0;
                    robot.clawRIntake();
                    moveStep = 3;
                    timer1.reset();
                }
            }

            // Path to pixel position (purple pixel
            if (moveStep == 3) {
/*
                if (randomizationResult == 1) {
                    robot.setSlider(400);
                    //robot.leftClawOpen();

                } else if (randomizationResult == 2) {

                    robot.setSlider(200);
                    //robot.leftClawOpen();

                } else if (randomizationResult == 3) {
                    robot.setSlider(350);

                }*/

                /*if (robot.getArmAngle() > 150) {
                    robot.setSliderLength(140);
                }

                if (robot.getArmAngle() > 50) {
                    robot.setClawPAngle(180 - robot.getArmAngle() + 15);
                }
                if(robot.getArmAngle() < 155){
                    timer1.reset();
                }
                if (timer1.milliseconds() > 400 && robot.slider.getCurrentPosition() > 135) {
                    robot.rightClawOpen();
                    moveStep = 4;
                    timer1.reset();
                }

                 */
                if(timer1.milliseconds() > 200) {
                    integral1 = 0;
                    integral2 = 0;
                    robot.rightClawOpen();

                    moveStep = 4;
                    timer1.reset();
                }
            }

            // Score purple pixel
            if (moveStep == 4) {

                if (randomizationResult == 3) {
                    xTarget = -41;
                }
                if(randomizationResult == 2) {
                    yTarget = -12;
                }


                if ((Math.abs(poseEstimate.getX() - xTarget) > 1) || (Math.abs(poseEstimate.getY() - yTarget) > 1)) {timer1.reset();}

                if (timer1.milliseconds() > 400) {
                    integral1 = 0;
                    integral2 = 0;

                    robot.bothClawClose();
                    robot.setClawPAngle(180);
                    moveStep = 5;
                    timer1.reset();
                }
            }
            // Also score purple pixel
            if (moveStep == 5){
                yTarget = -10;

                if(randomizationResult == 3){
                    xTarget = -41;
                }

                headingTarget = 0;

                if ((Math.abs(poseEstimate.getX() - xTarget) > 1) || (Math.abs(poseEstimate.getY() - yTarget) > 1)) {timer1.reset();}


                if (timer1.milliseconds() > 300) {
                    integral1 = 0;
                    integral2 = 0;

                    moveStep = 6;
                }
            }

            // Back up (yellow pixel)
            if (moveStep == 6) {
                integral1 = 0;
                integral2 = 0;
                yTarget = -12;
                xTarget = 32;

                if ((Math.abs(poseEstimate.getX() - xTarget) > 1) || (Math.abs(poseEstimate.getY() - yTarget) > 1)) {timer1.reset();}

                if (timer1.milliseconds() > 300) {
                    integral1 = 0;
                    integral2 = 0;

                    robot.leftClawClose();
                    moveStep = 7;
                    timer1.reset();
                }
            }
            if (moveStep == 7) {
                integral1 = 0;
                integral2 = 0;
                yTarget = -12;
                xTarget = 32;

                if ((Math.abs(poseEstimate.getX() - xTarget) > 1) || (Math.abs(poseEstimate.getY() - yTarget) > 1)) {timer1.reset();}

                if (timer1.milliseconds() > 300) {
                    robot.leftClawClose();
                    moveStep = 8;
                    timer1.reset();
                }
            }

            if(moveStep == 8) {
                robot.setArm(152);
                headingTarget = 0;

                if (robot.getArmAngle() > 145) {
                    robot.setSlider(570);
                }


                if (randomizationResult == 1) {
                    yTarget = -25;

                } else if (randomizationResult == 2) {
                    yTarget = -32;

                } else if (randomizationResult == 3) {
                    yTarget = -37;
                }

                if ((Math.abs(poseEstimate.getX() - xTarget) > 1) || (Math.abs(poseEstimate.getY() - yTarget) > 1)) {
                    timer1.reset();
                }

                if (timer1.milliseconds() > 400) {
                    moveStep = 9;
                }
            }

            if (moveStep == 9) {
                if(robot.getArmAngle() > 145) {
                    robot.setSlider(570);
                }

                if (robot.slider.getCurrentPosition() < 530) {
                    timer1.reset();
                }

                if (timer1.milliseconds() > 500) {
                    integral1 = 0;
                    integral2 = 0;

                    robot.leftClawOpen();
                    moveStep = 10;
                    timer1.reset();
                }
            }

            if (moveStep == 10) {
                integral1 = 0;
                integral2 = 0;

                if (timer1.milliseconds() > 300) {
                    robot.retractSlider();
                    robot.setArm(0);
                }

                if (robot.getArmAngle() < 120) {
                    robot.setClawPAngle(180);
                    moveStep = 11;
                    timer1.reset();
                }
            }

            if (moveStep == 11) {
                integral1 = 0;
                integral2 = 0;
                xTarget = 40;

                if (robot.getArmAngle() < 120) {
                    robot.setClawPAngle(180);
                }

                if(robot.getArmAngle() < 20){
                    robot.arm.setPower(0);
                    robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }

                if ((Math.abs(poseEstimate.getX() - xTarget) > 1)|| (Math.abs(poseEstimate.getY() - yTarget) > 1)){
                    timer1.reset();
                }

                if (timer1.milliseconds() > 300) {
                    moveStep = 12;
                    timer1.reset();
                }
            }
            if(moveStep == 12){
                integral1 = 0;
                integral2 = 0;

                xTarget = 45;
                yTarget = -12;
                headingTarget = 90;
            }


            /*if(robot.motorSliderLeft.getCurrentPosition()<800){
                robot.setTransferPosition();
            }*/



            botHeading = poseEstimate.getHeading();
            error1 = (xTarget - poseEstimate.getX());
            left_y = ((error1) * kp1 + integral1*ki1 +((error1 - lastError1) * kd1));
            error2 = (poseEstimate.getY() - yTarget);
            left_x = -((error2) * kp2 + integral1*ki2 +((error2 - lastError2) * kd2));
            error3 = (Math.toRadians(headingTarget) - botHeading);
            if (error3 > Math.PI) {
                error3 -= 2 * Math.PI;
            }
            if (error3 < -Math.PI) {
                error3 += 2 * Math.PI;
            }


            rot_x = -((error3) * kp3 + integral3*ki3 +((error3 - lastError3) * kd3));
            if (Math.abs(error3) < Math.toRadians(1)) {
                rot_x = 0;
            }
            integral1 += error1;
            integral2 += error2;
            integral3 +=error3;
            lastError1 = error1;
            lastError2 = error2;
            lastError3 = error3;
            //rot_x = (headingTarget - poseEstimate.getHeading()) * -kp3;
            if(left_y>0.5){
                left_y=0.5;
            }
            if(left_x>0.5){
                left_x=0.5;
            }
            if(left_y<(-0.5)){
                left_y=(-0.5);
            }
            if(left_x<(-0.5)){
                left_x=(-0.5);
            }

            drivetrain.remote(-left_y,left_x,-rot_x,poseEstimate.getHeading());


            /*if(moveStep==11){
                robot.motorBR.setPower(0);
                robot.motorFR.setPower(0);
                robot.motorBL.setPower(0);
                robot.motorFL.setPower(0);
            }*/







            drive.update();

            // telemetry.addData("ly", left_y);
            //telemetry.addData("lx", left_x);
            //telemetry.addData("rot", rot_x);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("result",randomizationResult);
            telemetry.addData("Move",moveStep);
            telemetry.addData("xT",xTarget);
            telemetry.addData("yT",yTarget);


//
            //telemetry.addData("stage", moveStep);

            telemetry.update();
        }
    }


    class TeamPropPipeline extends OpenCvPipeline {
        Mat YCbCr = new Mat();
        Mat leftCrop, middleCrop, rightCrop;
        double leftAverageFinal, middleAverageFinal, rightAverageFinal;
        Mat output = new Mat();
        Scalar rectColour = new Scalar(0, 0.0, 255.0);

        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("Pipeline running.");

            // TODO: tune values when new car
            Rect leftRect = new Rect(0, 100, 100, 79);
            Rect middleRect = new Rect(280, 100, 100, 79);
            Rect rightRect = new Rect(539, 80, 100, 79);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, rectColour, 2);
            Imgproc.rectangle(output, middleRect, rectColour, 2);
            Imgproc.rectangle(output, rightRect, rectColour, 2);

            leftCrop = YCbCr.submat(leftRect);
            middleCrop = YCbCr.submat(middleRect);
            rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop,2);  // Channel 2 = red
            Core.extractChannel(middleCrop, middleCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);

            Scalar leftAverage = Core.mean(leftCrop);
            Scalar middleAverage = Core.mean(middleCrop);
            Scalar rightAverage = Core.mean(rightCrop);

            leftAverageFinal = Math.abs(leftAverage.val[0] - 115);
            middleAverageFinal = Math.abs(middleAverage.val[0] - 115);
            rightAverageFinal = Math.abs(rightAverage.val[0] - 115);

            if ((leftAverageFinal < middleAverageFinal) && (leftAverageFinal < rightAverageFinal)) {
                telemetry.addLine("left");
                randomizationResult = 1;
            } else if ((middleAverageFinal < leftAverageFinal) &&  (middleAverageFinal < rightAverageFinal)) {
                telemetry.addLine("middle");
                randomizationResult = 2;
            } else {
                telemetry.addLine("right");
                randomizationResult = 3;
            }

            telemetry.addData("left", leftAverageFinal);
            telemetry.addData("middle", middleAverageFinal);
            telemetry.addData("right", rightAverageFinal);
            telemetry.addData("result",randomizationResult);

            telemetry.update();

            return output;
        }
    }
}