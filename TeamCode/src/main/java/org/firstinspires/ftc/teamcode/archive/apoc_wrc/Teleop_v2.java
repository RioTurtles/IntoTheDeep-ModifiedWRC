package org.firstinspires.ftc.teamcode.archive.apoc_wrc;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
@TeleOp (group = "Peasant Rabbits 2024")
public class  Teleop_v2 extends LinearOpMode {
    public enum states {
        INIT,
        GROUND,
        GROUND_EXTEND,
        GROUND_GRIP,
        EXTEND_GRIP,
        READY_SCORE,
        SCORING,
        SIMPLE_SCORING,
        RETURN_TO_INIT,
    }
    double direction_y, direction_x, pivot, heading;
    double CPR, revolutions, angle, angleNormalized;
    double lDis = 0, rDis = 0;
    double boardHeight = 0;
    int x = 0, y = 0;

    int position;

    ElapsedTime Timer1 = new ElapsedTime();
    // int state = 0;
    states state = states.INIT;

    int riggingState = 0;

    //TODO: Find arrays
    int[] sliderScoringPosition = {};
    double[] ArmScoringPosition = {6, 12.5, 23, 37};
    double[] clawScoringPosition = {};

    double [] simpleScoreArmAngle = {165,160, 155,150,145, 140};

    int simpleHeight = 0;
    boolean scoring_extend = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot = new Project1Hardware();
        MecanumDrive drivetrain = new MecanumDrive(robot);
        robot.init(hardwareMap,telemetry);
        robot.reset();

        Gamepad gamepad = new Gamepad();
        Gamepad lastGamepad = new Gamepad();

        robot.bothClawClose();
        robot.setArm(-12);
        robot.setClawPAngle(170);
        robot.setSlider(0);

        /*lDis = robot.leftDis.getDistance(DistanceUnit.CM);
        rDis = robot.rightDis.getDistance(DistanceUnit.CM);*/

        waitForStart();
        robot.imu.resetYaw();
        drivetrain.remote(0, 0, 0, 0);
        robot.slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeIsActive()) {
            direction_y = gamepad1.left_stick_y;
            direction_x = -gamepad1.left_stick_x;
            pivot = gamepad1.right_stick_x * 0.8;
            heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            lastGamepad.copy(gamepad);
            gamepad.copy(gamepad1);

            CPR = 3895.9;
            position = robot.arm.getCurrentPosition();
            revolutions = position / CPR;
            angle = revolutions * 360;
            angleNormalized = angle % 360;

            if (gamepad.options) {
                robot.imu.resetYaw();
            }

            if(gamepad.square) {
                robot.droneLaunch();
            }

            /*if (state==99){
                robot.setclawPAngle(90 - robot.getArmAngle() -6);
                if ( robot.getDis() > 0) {
                    if (gamepad1.dpad_up) {  //Max
                        //robot.setArm(37);
                        boardHeight = 66;
                    }
                    if (gamepad1.dpad_left) {  //High
                        //robot.setArm(23);
                        boardHeight = 57;
                    }
                    if (gamepad1.dpad_down) {  //Middle
                        //robot.setArm(12.5);
                        boardHeight = 37;
                    }
                    if (gamepad1.dpad_right) {  //Low
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

            //Reset claws and stuff
            switch (state) {
                case INIT:
                    robot.setSlider(0);
                    robot.slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    robot.arm.setPower(0);
                    robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                    robot.setClawPAngle(180);
                   robot.bothClawClose();

                    if(gamepad.right_bumper && !lastGamepad.right_bumper){
                        state = states.GROUND;
                        Timer1.reset();
                    }
                    //jump to scoring
                    if(gamepad.left_trigger > 0.3 && gamepad.right_trigger > 0.3){
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
                    if(gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)){
                        state = states.GROUND_EXTEND;
                    }

                    if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        state = states.GROUND_GRIP;
                        Timer1.reset();
                    }

                    if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                        state = states.INIT;
                        Timer1.reset();
                    }
                    break;

                //Ready for intake, no extend
                case GROUND_GRIP:
                    robot.retractSlider();
                    robot.clawPIntake();
                    if (gamepad.right_trigger > 0) {
                        robot.rightClawOpen();
                    } else {
                        robot.rightClawClose();
                    }

                    if (gamepad.left_trigger > 0) {
                        robot.leftClawOpen();
                    } else {
                        robot.leftClawClose();
                    }
                    if(gamepad.right_bumper && !lastGamepad.right_bumper){
                        state = states.READY_SCORE;
                        Timer1.reset();
                    }
                    if(gamepad.left_bumper && !lastGamepad.left_bumper){
                        state=states.GROUND;
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

                    if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        state = states.EXTEND_GRIP;
                        Timer1.reset();
                    }
                    if(gamepad.left_bumper && !lastGamepad.left_bumper){
                        state=states.GROUND;
                        Timer1.reset();
                    }
                    if(gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)){
                        state = states.GROUND;
                        Timer1.reset();
                    }
                    break;

                //Claw close, can open for further uses
                case EXTEND_GRIP:
                    robot.arm.setPower(0);
                    robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    robot.setSlider(900);
                    robot.clawPIntake();

                    //if(Timer1.milliseconds() > 300) {
                        if (gamepad.right_trigger > 0) {
                            robot.rightClawOpen();
                        } else {
                            robot.rightClawClose();
                        //}

                        if (gamepad.left_trigger > 0) {
                            robot.leftClawOpen();
                        } else {
                            robot.leftClawClose();
                        }
                    }
                    //if(Timer1.milliseconds() > 500) {
                        if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                            state = states.READY_SCORE;
                            Timer1.reset();
                        }
                    //}
                        if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                            state = states.GROUND_EXTEND;
                        }

                    break;

                //Retract slider and pixels possessed
                case READY_SCORE:
                    robot.arm.setPower(0);
                    robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    scoring_extend = false;

                    robot.bothClawClose();
                    robot.setClawPAngle(180);
                    robot.retractSlider();

                    if(gamepad.dpad_up && !lastGamepad.dpad_up) {
                        simpleHeight += 1;
                    }

                    if(gamepad.dpad_down && !lastGamepad.dpad_down) {
                        simpleHeight -= 1;
                    }

                    if(simpleHeight < 0) {
                        simpleHeight = 0;
                    }

                    if(simpleHeight > 4) {
                        simpleHeight = 4;
                    }

                    if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        state = states.SIMPLE_SCORING;
                        Timer1.reset();
                    }
                    if(gamepad.left_bumper && !lastGamepad.left_bumper){
                        state = states.GROUND;
                        Timer1.reset();
                    }
                    break;

                // Scoring in position and free to control pixel distribution
                case SCORING:
                    robot.setArm(160);

                    boardHeight = 0;
                    // TODO: Set board height
                    if (gamepad1.dpad_up) {  //Max
                        boardHeight = 66;
                    }
                    if (gamepad1.dpad_left) {  //High
                        boardHeight = 57;
                    }
                    if (gamepad1.dpad_down) {  //Middle
                        boardHeight = 37;
                    }
                    if (gamepad1.dpad_right) {  //Low
                        boardHeight = 18;
                    }

                    /*if (robot.getDis() < 0) {
                        robot.setArm(Math.atan(boardHeight / (robot.getDis() + boardHeight / Math.tan(60))));
                        robot.setSliderLength(Math.sqrt(Math.pow(boardHeight, 2) + Math.pow((robot.getDis() + boardHeight / Math.tan(60)), 2)));
                    }*/

                    if (robot.getArmAngle() > 90) {
                        robot.clawPScoring();
                    }

                    if (gamepad.triangle && !lastGamepad.triangle && robot.getArmAngle() > 90 && robot.getArmAngle() > 120 && robot.getArmAngle()<170) {
                        robot.setSlider(900);
                    } else {
                        robot.setSlider(0);
                    }

                    if (gamepad.left_trigger > 0) {
                        robot.leftClawOpen();
                    } else {
                        robot.leftClawClose();
                    }

                    if (gamepad.right_trigger > 0) {
                        robot.rightClawOpen();
                    } else {
                        robot.rightClawClose();
                    }

                    if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        state = states.RETURN_TO_INIT;
                        Timer1.reset();
                    }
                    if(gamepad.left_bumper && !lastGamepad.left_bumper) {
                        state = states.READY_SCORE;
                        Timer1.reset();
                    }
                    break;

                case SIMPLE_SCORING:

                    robot.setArm(simpleScoreArmAngle[simpleHeight]);
                   // robot.arm.setVelocity(1000);
                    //robot.setArm();

                    if(gamepad.dpad_up && !lastGamepad.dpad_up) {
                        simpleHeight += 1;
                    }

                    if(gamepad.dpad_down && !lastGamepad.dpad_down) {
                        simpleHeight -= 1;
                    }

                    if(simpleHeight < 0) {
                        simpleHeight = 0;
                    }

                    if(simpleHeight > 4) {
                        simpleHeight = 4;
                    }

                    if(gamepad.triangle && !lastGamepad.triangle && robot.getArmAngle() > 130) {
                        scoring_extend = !scoring_extend;
                    }

                    if(scoring_extend) {
                        robot.setSlider(900);
                    } else {
                        robot.setSlider(0);
                    }

                    if (robot.getArmAngle() > 70) {
                        robot.clawPScoring();
                    }

                    if (gamepad.left_trigger > 0) {
                        robot.rightClawOpen();
                    } else {
                        robot.rightClawClose();
                    }
                    if (gamepad.right_trigger > 0) {
                        robot.leftClawOpen();
                    } else {
                        robot.leftClawClose();
                    }

                    if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        state = states.RETURN_TO_INIT;
                        Timer1.reset();
                    }
                    if(gamepad.left_bumper && !lastGamepad.left_bumper){
                        state = states.READY_SCORE;
                        Timer1.reset();
                    }
                    break;


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
            if(riggingState == 0){
                robot.rRiggingUp.setPwmDisable();
                robot.lRiggingUp.setPwmDisable();
            }

            if (riggingState == 1) {
                robot.extendRiggingServo();

                if (gamepad.dpad_up) {
                    robot.lRigging.setPower(1);
                } else if (gamepad.dpad_down) {
                    robot.lRigging.setPower(-1);
                } else {
                    robot.lRigging.setPower(0);
                }

                if (gamepad.triangle) {
                    robot.rRigging.setPower(1);
                } else if (gamepad.cross) {
                    robot.rRigging.setPower(-1);
                } else {
                    robot.rRigging.setPower(0);
                }
            }

            if (riggingState == 2) {
                robot.rRiggingUp.setPwmDisable();
                robot.lRiggingUp.setPwmDisable();
            }

            if (riggingState == 3) {
                if (gamepad.triangle) {
                    robot.extendRiggingMotor();
                } else if (gamepad.cross) {
                    robot.retractRiggingMotor();
                } else {
                    robot.lRigging.setPower(0);
                    robot.rRigging.setPower(0);
                }
            }

            if (gamepad.dpad_left && !lastGamepad.dpad_left) {
                riggingState += 1;
            }

            if (gamepad.dpad_right && !lastGamepad.dpad_right) {
                riggingState -= 1;
            }

            if (riggingState > 3) {
                riggingState = 3;
            }

            if (riggingState < 0) {
                riggingState = 0;
            }

            //if (state < 0) {state = 0;}
            //if (state > 6) {state = 6;}


            /*telemetry.addData("Encoder Angle (Degrees)", angle);
            telemetry.addData("Encoder Angle - Normalized (Degrees)", angleNormalized);*/

            /*telemetry.addData("Distance", robot.getDis());
            telemetry.addData("Arm position", robot.getArmAngle());
            telemetry.addData("Slider length ", robot.getSliderLength());
            telemetry.addData("Board height", boardHeight);

             */
            telemetry.addData("state", state);
            telemetry.addData("RiggingState", riggingState);
            telemetry.addData("arm", robot.getArmAngle());

            drivetrain.remote(direction_y, direction_x, -pivot, heading);
            telemetry.update();
        }
    }
}
