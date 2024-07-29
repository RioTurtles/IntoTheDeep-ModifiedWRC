package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDController;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp
public class Teleop_Kinematics2 extends LinearOpMode {
    public enum states {
        INIT,
        GROUND,
        GROUND_EXTEND,
        GROUND_GRIP,
        EXTEND_GRIP,
        READY_SCORE,
        AUTO_ALIGN,
        SCORING,
        SIMPLE_SCORING,
        RETURN_TO_INIT,
    }
    double direction_y, direction_x, pivot, heading;
    double CPR, revolutions, angle, angleNormalized;
    double lDis = 0, rDis = 0;
    double avgDis = 0;
    public static double  kP = 2, kI = 0.1, kD = 0.08;
    public static double clawPAngle = 174;
    double boardHeight = 0;

    int position;

    ElapsedTime Timer1 = new ElapsedTime();
    // int state = 0;
    states state = states.INIT;

    int riggingState = 0;

    double [] simpleScoreArmAngle = {165, 160, 155, 150, 145, 140};
    int simpleHeight = 0;
    double [] scoreArmAngle = {40, 60, 80, 100};
    int scoreHeight = 0;
    double boardHeading = -Math.PI/2;
    boolean scoring_extend = false;
    PIDController heading_pid = new PIDController(kP, kI, kD);
    double horizontalOffset = 0, maxScoringDis = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot = new Project1Hardware();
        MecanumDrive drivetrain = new MecanumDrive(robot);
        robot.init(hardwareMap,telemetry);
        robot.reset();

        /*Gamepad gamepad = new Gamepad();
        Gamepad lastGamepad = new Gamepad();*/

        Gamepad Gamepad1 = new Gamepad();
        Gamepad lastGamepad1 = new Gamepad();

        Gamepad Gamepad2 = new Gamepad();
        Gamepad lastGamepad2 = new Gamepad();

        robot.bothClawClose();
        robot.setArm(-12);
        robot.setClawPAngle(170);
        robot.setSlider(0);

        waitForStart();
        robot.imu.resetYaw();
        drivetrain.remote(0, 0, 0, 0);
        robot.slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeIsActive()) {
            direction_y = gamepad1.left_stick_y;
            direction_x = -gamepad1.left_stick_x;
            pivot = gamepad1.right_stick_x * 0.8;
            heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            avgDis = (lDis + rDis) / 2;
            lDis = robot.leftDis.getDistance(DistanceUnit.CM);
            rDis = robot.rightDis.getDistance(DistanceUnit.CM);

            /*lastGamepad.copy(gamepad);
            gamepad.copy(gamepad1);*/

            lastGamepad1.copy(Gamepad1);
            Gamepad1.copy(gamepad1);

            lastGamepad2.copy(Gamepad2);
            Gamepad2.copy(gamepad2);

            CPR = 3895.9;
            position = robot.arm.getCurrentPosition();
            revolutions = position / CPR;
            angle = revolutions * 360;
            angleNormalized = angle % 360;
            heading_pid.setPID(kP, kI, kD);

            if (Gamepad1.options) {
                robot.imu.resetYaw();
            }

            if (Gamepad1.square || Gamepad2.square) {
                robot.droneLaunch();
            }

            /*if (state == 99){
                robot.setclawPAngle(90 - robot.getArmAngle() -6);
                if ( robot.getDis() > 0) {
                    if (Gamepad1.dpad_up) {  //Max
                        //robot.setArm(37);
                        boardHeight = 66;
                    }
                    if (Gamepad1.dpad_left) {  //High
                        //robot.setArm(23);
                        boardHeight = 57;
                    }
                    if (Gamepad1.dpad_down) {  //Middle
                        //robot.setArm(12.5);
                        boardHeight = 37;
                    }
                    if (Gamepad1.dpad_right) {  //Low
                        //robot.setArm(6);
                        boardHeight = 18;
                    }
                    //robot.setArm(Math.atan(boardHeight / (robot.getDis() + boardHeight / Math.tan(60))));
                    robot.setSliderLength(Math.sqrt(Math.pow(boardHeight, 2) + Math.pow((robot.getDis() + boardHeight / Math.tan(60)), 2)) - );
                    //robot.setSliderLength();r
                }*/

            //boardHeight=10;
            //robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            //robot.setSliderLength((robot.getDis()/Math.cos(Math.toRadians(180-robot.getArmAngle()))-50)+boardHeight);
            //  telemetry.addData("Testing",Math.sqrt(Math.pow(robot.getDis(),2)+Math.pow(boardHeight,2)-2* robot.getDis()*boardHeight*Math.cos(Math.toRadians(120))));
            //}


            //target position - board heading
            //Reset claws and stuff
            switch (state) {
                case INIT:
                    robot.setSlider(0);
                    robot.slider.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

                    robot.arm.setPower(0);
                    robot.arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

                    robot.setClawPAngle(180);
                    robot.bothClawClose();

                    if(Gamepad1.right_bumper && !lastGamepad1.right_bumper){
                        state = states.GROUND;
                        Timer1.reset();
                    }
                    //jump to scoring
                    if(Gamepad1.left_trigger > 0.3 && Gamepad1.right_trigger > 0.3){
                        state = states.SIMPLE_SCORING;
                        Timer1.reset();
                    }
                    break;

                //Set claw to intake position
                case GROUND:
                    robot.retractSlider();
                    robot.clawPIntake();
                    robot.arm.setPower(0);
                    robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                    if (Timer1.milliseconds() > 100){
                        robot.bothClawOpen();
                    }
                    if(Gamepad1.right_trigger > 0 && !(lastGamepad1.right_trigger > 0)) {
                        state = states.GROUND_EXTEND;
                    }

                    if (Gamepad1.right_bumper && !lastGamepad1.right_bumper) {
                        state = states.GROUND_GRIP;
                        Timer1.reset();
                    }

                    if (Gamepad1.left_bumper && !lastGamepad1.left_bumper) {
                        state = states.INIT;
                        Timer1.reset();
                    }
                    break;

                //Ready for intake, no extend
                case GROUND_GRIP:
                    robot.retractSlider();
                    robot.clawPIntake();

                    if (Gamepad1.right_trigger > 0) {
                        robot.rightClawOpen();
                    } else {
                        robot.rightClawClose();
                    }

                    if (Gamepad1.left_trigger > 0) {
                        robot.leftClawOpen();
                    } else {
                        robot.leftClawClose();
                    }
                    if(Gamepad1.right_bumper && !lastGamepad1.right_bumper){
                        state = states.READY_SCORE;
                        Timer1.reset();
                    }
                    if(Gamepad1.left_bumper && !lastGamepad1.left_bumper){
                        state = states.GROUND;
                        Timer1.reset();
                    }
                    break;

                //Ready for intake, extended slider
                case GROUND_EXTEND:
                    robot.bothClawOpen();
                    robot.clawPIntake();

                    direction_x = direction_x * 0.5;
                    direction_y = direction_y * 0.5;

                    robot.setSlider(700);
                    robot.arm.setPower(0);
                    robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                    if (Gamepad1.right_bumper && !lastGamepad1.right_bumper) {
                        state = states.EXTEND_GRIP;
                        Timer1.reset();
                    }
                    if(Gamepad1.left_bumper && !lastGamepad1.left_bumper){
                        state=states.GROUND;
                        Timer1.reset();
                    }

                    if(Gamepad1.right_trigger > 0 && !(lastGamepad1.right_trigger > 0)){
                        state = states.EXTEND_GRIP;
                        Timer1.reset();
                    }
                    break;

                //Claw close, can open for further uses
                case EXTEND_GRIP:
                    robot.arm.setPower(0);
                    robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    robot.setSlider(950);
                    robot.clawPIntake();

                    if (Timer1.milliseconds() > 200) {
                        if (Gamepad1.right_trigger > 0) {
                            robot.rightClawOpen();
                        } else {
                            robot.rightClawClose();
                        }

                        if (Gamepad1.left_trigger > 0) {
                            robot.leftClawOpen();
                        } else {
                            robot.leftClawClose();
                        }
                    }

                    if (Gamepad1.right_bumper && !lastGamepad1.right_bumper) {
                        state = states.READY_SCORE;
                        Timer1.reset();
                    }
                    if(Gamepad1.left_bumper && !lastGamepad1.left_bumper){
                        state=states.GROUND_EXTEND;
                    }
                    break;

                //Retract slider and pixels possessed
                case READY_SCORE:
                    if (Timer1.milliseconds() > 1000) {
                        robot.arm.setPower(0);
                        robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    }
                    scoring_extend = false;

                    robot.bothClawClose();
                    robot.setClawPAngle(180);
                    robot.retractSlider();

                    if (Gamepad1.right_bumper && !lastGamepad1.right_bumper) {
                        state = states.SCORING;
                        Timer1.reset();
                    }
                    if(Gamepad1.left_bumper && !lastGamepad1.left_bumper){
                        state = states.GROUND;
                        Timer1.reset();
                    }
                    break;

                // case AUTO_ALIGN:
                    /*drivetrain.remote(0,0,0,0);
                    Gamepad1.right_stick_x = 0;
                    drivetrain.remote2(direction_y, direction_x, (heading - alignTarget) * kP, heading);*/

                //break;
                case SCORING:
                    if(heading > Math.PI/2) {
                        heading -= 2 * Math.PI;
                    }
                    double align_output = heading_pid.calculate(
                            heading, boardHeading
                    );
                    pivot = -align_output;

                    if (Gamepad1.right_stick_x > 0.1 || Gamepad1.right_stick_x < 0.1) {
                        pivot = Gamepad1.right_stick_x * 0.8;
                    }

                    /*if (gamepad1.dpad_up) {  //Max
                        boardHeight = 66;
                        maxScoringDis = 0;
                    }
                    if (gamepad1.dpad_left) {  //High
                        boardHeight = 57;
                        maxScoringDis = 0;
                    }
                    if (gamepad1.dpad_down) {  //Middle
                        boardHeight = 37;
                        maxScoringDis = 0;
                    }
                    if (gamepad1.dpad_right) {  //Low
                        boardHeight = 18;
                        maxScoringDis = 0;
                    }*/

                    if (scoreHeight == 0) maxScoringDis = 0;
                    if (scoreHeight == 1) maxScoringDis = 0;
                    if (scoreHeight == 2) maxScoringDis = 0;
                    if (scoreHeight == 3) maxScoringDis = 0;


                    //if (avgDis < maxScoringDis) {
                        robot.setArm(Math.atan(Math.toRadians(scoreArmAngle[scoreHeight] / (avgDis + scoreArmAngle[scoreHeight] / Math.tan(Math.toRadians(60))))));
                    //}

                    if (Gamepad1.triangle && !lastGamepad1.triangle && robot.getArmAngle() > 130) {
                        scoring_extend = !scoring_extend;
                    }

                    if (scoring_extend) {
                        robot.setSliderLength(Math.sqrt(Math.pow(scoreArmAngle[scoreHeight], 2) + Math.pow((avgDis + horizontalOffset + scoreArmAngle[scoreHeight] / Math.tan(Math.toRadians(60))), 2)));
                    } else {
                        robot.setSliderLength(-1);
                    }

                    if (robot.getArmAngle() > 90) {
                        robot.clawPScoring();
                    }

                    if (gamepad1.left_trigger > 0) {
                        robot.leftClawOpen();
                    } else {
                        robot.leftClawClose();
                    }

                    if (gamepad1.right_trigger > 0) {
                        robot.rightClawOpen();
                    } else {
                        robot.rightClawClose();
                    }

                    if (gamepad1.right_bumper && !lastGamepad1.right_bumper) {
                        state = states.RETURN_TO_INIT;
                        Timer1.reset();
                    }
                    if(gamepad1.left_bumper && !lastGamepad1.left_bumper){
                        state = states.READY_SCORE;
                        robot.setArm(0);
                        Timer1.reset();
                    }

                    break;

//                case SIMPLE_SCORING:
//
//                    if(heading > Math.PI/2){
//                        heading -= 2 * Math.PI;
//                    }
//                    double align_output = heading_pid.calculate(
//                            heading, boardHeading
//                    );
//                    pivot = -align_output;
//
//                    if (Gamepad1.right_stick_x > 0.1 || Gamepad1.right_stick_x < 0.1) {
//                        pivot = Gamepad1.right_stick_x * 0.8;
//                    }
//
//                    robot.setArm(simpleScoreArmAngle[simpleHeight]);
//                    //robot.arm.setVelocity(1000);
//
//                    if (Gamepad1.triangle && !lastGamepad1.triangle && robot.getArmAngle() > 130) {
//                        scoring_extend = !scoring_extend;
//                    }
//
//                    if (scoring_extend) {
//                        robot.setSlider(900);
//                    } else {
//                        robot.setSlider(0);
//                    }
//
//                    if (robot.getArmAngle() > 70) {
//                        robot.clawPScoring();
//                    }
//
//                    if (Gamepad1.left_trigger > 0) {
//                        robot.rightClawOpen();
//                    } else {
//                        robot.rightClawClose();
//                    }
//                    if (Gamepad1.right_trigger > 0) {
//                        robot.leftClawOpen();
//                    } else {
//                        robot.leftClawClose();
//                    }
//
//                    if (Gamepad1.right_bumper && !lastGamepad1.right_bumper) {
//                        state = states.RETURN_TO_INIT;
//                        Timer1.reset();
//                    }
//                    if(Gamepad1.left_bumper && !lastGamepad1.left_bumper){
//                        state = states.READY_SCORE;
//                        Timer1.reset();
//                    }
//
//                    break;


                //Back to intake position
                case RETURN_TO_INIT:
                    scoring_extend = false;

                    robot.setArm(0);
                    //robot.arm.setVelocity(1600);
                    robot.retractSlider();

                    if (robot.getArmAngle() < 130) {
                        robot.setClawPAngle(180);
                    }

                    if (robot.getArmAngle() < 0) {
                        state = states.INIT;
                        Timer1.reset();
                    }
                    break;
            }

            //Arm height placement
            if(Gamepad1.dpad_up && !lastGamepad1.dpad_up || Gamepad2.dpad_up && !lastGamepad2.dpad_up) {
                simpleHeight += 1;
            }

            if(Gamepad1.dpad_down && !lastGamepad1.dpad_down || Gamepad2.dpad_down && !lastGamepad2.dpad_down) {
                simpleHeight -= 1;
            }

            if (simpleHeight < 0) {
                simpleHeight = 0;
            }

            if (simpleHeight > 5) {
                simpleHeight = 5;
            }

            //Rigging
            if (riggingState == 0) {
                robot.rRiggingUp.setPwmDisable();
                robot.lRiggingUp.setPwmDisable();
            }

            if (riggingState == 1) {
                robot.extendRiggingServo();

                if (Gamepad1.dpad_up) {
                    robot.lRigging.setPower(1);
                } else if (Gamepad1.dpad_down) {
                    robot.lRigging.setPower(-1);
                } else {
                    robot.lRigging.setPower(0);
                }

                if (Gamepad1.triangle) {
                    robot.rRigging.setPower(1);
                } else if (Gamepad1.cross) {
                    robot.rRigging.setPower(-1);
                } else {
                    robot.rRigging.setPower(0);
                }

                if (Gamepad1.circle) drivetrain.remote(0,0,0,0);
            }

            if (riggingState == 2) {
                robot.rRiggingUp.setPwmDisable();
                robot.lRiggingUp.setPwmDisable();
            }

            if (riggingState == 3) {
                if (Gamepad1.triangle) {
                    robot.extendRiggingMotor();
                } else if (Gamepad1.cross) {
                    robot.retractRiggingMotor();
                } else {
                    robot.lRigging.setPower(0);
                    robot.rRigging.setPower(0);
                }
            }

            if (Gamepad1.dpad_left && !lastGamepad1.dpad_left) {
                riggingState += 1;
            }

            if (Gamepad1.dpad_right && !lastGamepad1.dpad_right) {
                riggingState -= 1;
            }

            if (Gamepad1.dpad_left && Gamepad1.dpad_right) {
                robot.retractRiggingServo();
            }

            if (riggingState > 3) {
                riggingState = 3;
            }

            if (riggingState < 0) {
                riggingState = 0;
            }

           /* if (Gamepad1.cross) {
                pivot = heading - boardHeading;

                if (pivot > Math.PI) {
                    pivot -= 2 * Math.PI;
                }
                if (pivot < -Math.PI) {
                    pivot += 2 * Math.PI;
                }
                pivot = pivot * kP;
                //boardHeading = pivot;
            }*/

            //Auto align
            if (Gamepad1.circle) {
                if(heading > Math.PI/2){
                    heading -= 2 * Math.PI;
                }
                double align_output = heading_pid.calculate(
                        heading, boardHeading
                );
                pivot = -align_output;
            }





            /*telemetry.addData("Encoder Angle (Degrees)", angle);
            telemetry.addData("Encoder Angle - Normalized (Degrees)", angleNormalized);*/

            telemetry.addData("state", state);
            telemetry.addData("RiggingState", riggingState);
            telemetry.addData("arm", robot.getArmAngle());
            telemetry.addData("Arm height", simpleHeight);
            telemetry.addLine();
            telemetry.addData("pivot", pivot);
            telemetry.addData("heading", heading);
            telemetry.addLine();
            telemetry.addData("simpleHeight", simpleHeight);
            telemetry.addData("array", scoreArmAngle[simpleHeight]);
            drivetrain.remote(direction_y, direction_x, -pivot, heading);
            telemetry.update();
        }
    }
}