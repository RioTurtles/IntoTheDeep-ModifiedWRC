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

@Autonomous(name="BlueFar")
public class AutonBlueFar extends LinearOpMode {

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
    int randomizationResult = 2;

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
        drive.setPoseEstimate(new Pose2d(-36, 63, Math.toRadians(90)));
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
                robot.retractSlider();
                robot.bothClawClose();
                headingTarget=90;

                xTarget= -36;

                yTarget = 36;


                if ((Math.abs(poseEstimate.getX() - xTarget) > 1) || (Math.abs(poseEstimate.getY() - yTarget) > 2)) {timer1.reset();}
                if (timer1.milliseconds() > 300) {
                    moveStep = 2;
                    integral1 = 0;
                    integral2 = 0;

                    timer1.reset();
                }
            }

            // Rotate (purple pixel)
            if (moveStep == 2) {
                xTarget = -36;

                robot.bothClawClose();
                robot.arm.setPower(0);
                robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                if (randomizationResult == 1) {
                    headingTarget = 150;


                } else if (randomizationResult == 2) {
                    yTarget = 13;
                    headingTarget = 270;

                } else if (randomizationResult == 3) {
                    headingTarget = 0;

                }

                /*if (robot.getArmAngle() > 50) {
                    robot.setClawPAngle(180-robot.getArmAngle()+15);
                }*/

                if ((Math.abs(poseEstimate.getX() - xTarget) > 1) || (Math.abs(poseEstimate.getY() - yTarget) > 1)) {timer1.reset();}


                if (timer1.milliseconds() > 400) {
                    integral1=0;
                    integral2=0;

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
                if(timer1.milliseconds() > 400) {
                    integral1 = 0;
                    integral2 = 0;

                    robot.leftClawOpen();
                    moveStep = 4;
                    timer1.reset();

                }
            }

            // Score purple pixel
            if (moveStep == 4) {


                if (randomizationResult == 1 && timer1.milliseconds() > 300) {
                    integral1=0;
                    integral2=0;
                    xTarget = -41;
                }
                if(randomizationResult == 2) {
                    yTarget = 12;
                }

                if (timer1.milliseconds() > 200) {
                    robot.setClawPAngle(180);
                }

                if ((Math.abs(poseEstimate.getX() - xTarget) > 1.5) || (Math.abs(poseEstimate.getY() - yTarget) > 1.5)) {timer1.reset();}

                if (timer1.milliseconds() > 400) {
                    integral1 = 0;
                    integral2 = 0;

                    robot.leftClawClose();

                    moveStep = 5;
                    timer1.reset();
                }
            }
            // Also score purple pixel
            if (moveStep == 5){

                if (timer1.milliseconds() > 300) {
                    headingTarget = 0;
                }

                yTarget = 10;

                if (randomizationResult == 1){
                    xTarget = -41;
                }


                if ((Math.abs(poseEstimate.getX() - xTarget) > 1.5) || (Math.abs(poseEstimate.getY() - yTarget) > 1.5)) {timer1.reset();}


                if (timer1.milliseconds() > 300) {
                    integral1 = 0;
                    integral2 = 0;

                    moveStep = 6;
                    timer1.reset();
                }
            }

            // Back up (yellow pixel)
            if (moveStep == 6) {
                integral1 = 0;
                integral2 = 0;

                yTarget = 12;
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

                yTarget = 12;
                xTarget = 32;

                if ((Math.abs(poseEstimate.getX() - xTarget) > 1) || (Math.abs(poseEstimate.getY() - yTarget) > 1)) {timer1.reset();}




                if (timer1.milliseconds() > 300) {
                    robot.leftClawClose();
                    moveStep = 8;
                    integral1 = 0;
                    integral2 = 0;
                    timer1.reset();
                }
            }




            if(moveStep==8) {
                robot.setArm(154);
                headingTarget = 0;

                if (robot.getArmAngle() > 140) {
                    robot.setSlider(580);
                }

                if (randomizationResult == 1) {
                    yTarget = 45;

                } else if (randomizationResult == 2) {
                    yTarget = 33.5;

                } else if (randomizationResult == 3) {
                    yTarget = 28;
                }
                if ((Math.abs(poseEstimate.getX() - xTarget) > 1) || (Math.abs(poseEstimate.getY() - yTarget) > 1)) {
                    timer1.reset();
                }

                if (timer1.milliseconds() > 400) {
                    moveStep = 9;
                    timer1.reset();
                }
            }

            if (moveStep == 9) {
                integral1 = 0;
                integral2 = 0;

                if(robot.getArmAngle() > 145) {
                    robot.setSlider(580);
                }


                if (robot.slider.getCurrentPosition() < 530) {
                    timer1.reset();
                }

                if (timer1.milliseconds() > 600) {
                    integral1=0;
                    integral2=0;

                    robot.rightClawOpen();
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
                    integral1 = 0;
                    integral2 = 0;

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
                    integral1 = 0;
                    integral2 = 0;
                    timer1.reset();
                }

            }
            if(moveStep == 12){
                integral1 = 0;
                integral2 = 0;

                xTarget = 45;
                yTarget = 12;
                headingTarget = 270;
                robot.bothClawClose();
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
            if((Math.abs(left_x)>0.5||Math.abs(left_y)>0.5)&&(left_y!=0)&&left_x!=0) {


                if (Math.abs(left_y) > Math.abs(left_x)) {
                    left_x = left_x * Math.abs((0.5 / left_y));
                    if(left_y>0){
                        left_y=0.5;
                    }else {
                        left_y=-0.5;
                    }
                }else{
                    left_y = left_y * Math.abs((0.5 / left_x));
                    if(left_x>0){
                        left_x=0.5;
                    }else {
                        left_x=-0.5;
                    }
                }
            }
            //rot_x = (headingTarget - poseEstimate.getHeading()) * -kp3;
            /*if(left_y>0.5){
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

             */

            drivetrain.remote(-left_y,left_x,-rot_x,poseEstimate.getHeading());
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
        double leftTarget, middleTarget, rightTarget;
        Mat output = new Mat();
        Scalar rectColour = new Scalar(0, 0.0, 255.0);

        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("Pipeline running.");

            // TODO: tune values when new car
            Rect leftRect = new Rect(0, 100, 100, 79);
            Rect middleRect = new Rect(280, 100, 100, 79);
            Rect rightRect = new Rect(539, 150, 100, 79);
            //Rect rightRect = new Rect(539, 90, 100, 79);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, rectColour, 2);
            Imgproc.rectangle(output, middleRect, rectColour, 2);
            Imgproc.rectangle(output, rightRect, rectColour, 2);

            leftCrop = YCbCr.submat(leftRect);
            middleCrop = YCbCr.submat(middleRect);
            rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop,0);  // Channel 2 = red
            Core.extractChannel(middleCrop, middleCrop, 0);
            Core.extractChannel(rightCrop, rightCrop, 0);

            Scalar leftAverage = Core.mean(leftCrop);
            Scalar middleAverage = Core.mean(middleCrop);
            Scalar rightAverage = Core.mean(rightCrop);

            /*leftAverageFinal = Math.abs(leftAverage.val[0] - 105);
            middleAverageFinal = Math.abs(middleAverage.val[0] - 105);
            rightAverageFinal = Math.abs(rightAverage.val[0] - 105);*/

            leftAverageFinal = Math.abs(leftAverage.val[0] - leftTarget);
            middleAverageFinal = Math.abs(middleAverage.val[0] - middleTarget);
            rightAverageFinal = Math.abs(rightAverage.val[0] - rightTarget);

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

            if (gamepad1.dpad_left) {
                leftTarget = leftAverage.val[0];
            }
            if (gamepad1.dpad_up) {
                middleTarget = middleAverage.val[0];
            }
            if (gamepad1.dpad_right) {
                rightTarget = rightAverage.val[0];
            }

            telemetry.addData("leftAvg",leftAverage.val[0]);
            telemetry.addData("rightAvg",rightAverage.val[0]);
            telemetry.addData("middleAvg",middleAverage.val[0]);
            telemetry.addLine();
            telemetry.addData("left", leftAverageFinal);
            telemetry.addData("middle", middleAverageFinal);
            telemetry.addData("right", rightAverageFinal);
            telemetry.addData("result",randomizationResult);
            telemetry.addLine();
            telemetry.addData("leftTarget", leftTarget);
            telemetry.addData("middleTarget", middleTarget);
            telemetry.addData("rightTarget", rightTarget);

            telemetry.update();

            return output;
        }
    }
}