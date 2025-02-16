package org.firstinspires.ftc.teamcode.archive.apoc_wrc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDController;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@Disabled
@TeleOp (group = "Peasant Rabbits 2024")
public class Teleop_v3_WRC_Blue extends LinearOpMode {
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
        DRAW
    }

    double direction_y, direction_x, pivot, heading;
    double CPR, revolutions, angle, angleNormalized;
    double lDis = 0, rDis = 0;
    double avgDis = 0;
    public static double kP = 1.8, kI = 0.1, kD = 0.08;
    //public static double offset = 25;

    int position;

    ElapsedTime timer1 = new ElapsedTime();
    // int state = 0;
    states state = states.INIT;

    int riggingState = 0;

    int[] mosaicScoreSliderLength = {0, 200, 300, 400, 500, 600, 700, 800, 900};
    double[] cycleScoreArmAngle = {145, 140, 135, 130, 125, 120, 115, 110};
    double[] drawArmAngle = {160, 155, 150, 145, 140, 135, 130, 125, 120};
    int mosaicScoreHeight = 0;
    int  cycleScoreHeight = 0;
    int drawScoreHeight = 0;
    boolean mosaicMode = true, cycleMode = false;
    double boardHeading = -Math.PI / 2;
    boolean scoring_extend = false;
    boolean droneConfirmed = false;
    PIDController heading_pid = new PIDController(kP, kI, kD);

    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot = new Project1Hardware();
        MecanumDrive drivetrain = new MecanumDrive(robot);
        MultipleTelemetry telemetry1 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry);
        robot.reset();

        /*Gamepad gamepad = new Gamepad();
        Gamepad lastGamepad = new Gamepad();*/

        Gamepad Gamepad1 = new Gamepad();
        Gamepad lastGamepad1 = new Gamepad();

        Gamepad Gamepad2 = new Gamepad();
        Gamepad lastGamepad2 = new Gamepad();

        robot.bothClawClose();
        robot.setArm(-12);
        robot.setClawPAngle(180);
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

            if (Gamepad1.options) robot.imu.resetYaw();

            if (Gamepad2.square && !lastGamepad2.square) {
                if (!droneConfirmed) droneConfirmed = true;
                else robot.droneLaunch();
            }

            if (Gamepad2.circle && !lastGamepad2.circle) {
                state = states.DRAW;
                timer1.reset();
            }

            // Reset slider
            if (Gamepad2.dpad_down && !lastGamepad2.dpad_down) robot.resetRetractSlider(() -> sleep(300));
            // Reset arm
            if (Gamepad2.dpad_up && !lastGamepad2.dpad_up) robot.resetArm(() -> sleep(300));

            /*if (state == 99){
                robot.setClawPAngle(90 - robot.getArmAngle() -6);
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
                    robot.setArm(Math.atan(boardHeight / (robot.getDis() + boardHeight / Math.tan(60))));
                    robot.setSliderLength(Math.sqrt(Math.pow(boardHeight, 2) + Math.pow((robot.getDis() + boardHeight / Math.tan(60)), 2)) - );
                    robot.setSliderLength();
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

                    if (Gamepad1.right_bumper && !lastGamepad1.right_bumper) {
                        state = states.GROUND;
                        timer1.reset();
                    }
                    //jump to scoring
                    if (Gamepad1.left_trigger > 0.3 && Gamepad1.right_trigger > 0.3) {
                        state = states.SIMPLE_SCORING;
                        timer1.reset();
                    }
                    break;

                //Set claw to intake position
                case GROUND:
                    robot.retractSlider();
                    robot.clawPIntake();
                    robot.arm.setPower(0);
                    robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                    if (timer1.milliseconds() > 400) {
                        robot.bothClawOpen();
                    }
                    if (Gamepad1.right_trigger > 0 && !(lastGamepad1.right_trigger > 0)) {
                        state = states.GROUND_EXTEND;
                    }

                    if (Gamepad1.right_bumper && !lastGamepad1.right_bumper) {
                        state = states.GROUND_GRIP;
                        timer1.reset();
                    }

                    if (Gamepad1.left_bumper && !lastGamepad1.left_bumper) {
                        state = states.INIT;
                        timer1.reset();
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
                    if (Gamepad1.right_bumper && !lastGamepad1.right_bumper) {
                        state = states.READY_SCORE;
                        timer1.reset();
                    }
                    if (Gamepad1.left_bumper && !lastGamepad1.left_bumper) {
                        state = states.GROUND;
                        timer1.reset();
                    }
                    break;

                //Ready for intake, extended slider
                case GROUND_EXTEND:
                    robot.bothClawOpen();

                    if (timer1.milliseconds() > 400) robot.clawPIntakeExtend();

                    direction_x = direction_x * 0.5;
                    direction_y = direction_y * 0.5;

                    robot.setSlider(700);
                    robot.arm.setPower(0);
                    robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                    if (Gamepad1.right_bumper && !lastGamepad1.right_bumper) {
                        state = states.EXTEND_GRIP;
                        timer1.reset();
                    }
                    if (Gamepad1.left_bumper && !lastGamepad1.left_bumper) {
                        state = states.GROUND;
                        timer1.reset();
                    }

                    if (Gamepad1.right_trigger > 0 && !(lastGamepad1.right_trigger > 0)) {
                        state = states.GROUND;
                        timer1.reset();
                    }
                    break;

                //Claw close, can open for further uses
                case EXTEND_GRIP:
                    robot.arm.setPower(0);
                    robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    robot.setSlider(900);
                    robot.clawPIntakeExtend();

                    if (timer1.milliseconds() > 200) {
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
                        timer1.reset();
                    }
                    if (Gamepad1.left_bumper && !lastGamepad1.left_bumper) {
                        state = states.GROUND_EXTEND;
                        timer1.reset();
                    }
                    break;

                //Retract slider and pixels possessed
                case READY_SCORE:
                    if (timer1.milliseconds() > 1000) {
                        robot.arm.setPower(0);
                        robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    }
                    scoring_extend = false;

                    robot.bothClawClose();
                    if (timer1.milliseconds() > 400) {
                        robot.setClawPAngle(180);
                    }
                    robot.retractSlider();

                    if (Gamepad1.right_bumper && !lastGamepad1.right_bumper) {
                        state = states.SIMPLE_SCORING;
                        timer1.reset();
                    }
                    if (Gamepad1.left_bumper && !lastGamepad1.left_bumper) {
                        state = states.GROUND;
                        timer1.reset();
                    }
                    break;

                // case AUTO_ALIGN:
                    /*drivetrain.remote(0,0,0,0);
                    Gamepad1.right_stick_x = 0;
                    drivetrain.remote2(direction_y, direction_x, (heading - alignTarget) * kP, heading);*/

                // break;

                case SIMPLE_SCORING:

                    if (heading < -Math.PI / 2) heading += 2 * Math.PI;

                    double align_output = heading_pid.calculate(
                            heading, -boardHeading
                    );
                    pivot = -align_output;


                    if (Gamepad1.right_stick_x > 0.1 || Gamepad1.right_stick_x < 0.1) {
                        pivot = Gamepad1.right_stick_x * 0.8;
                    }

                    if (mosaicMode) {
                        //mosaic
                        //robot.arm.setVelocity(1000);

                        if (Gamepad1.triangle && !lastGamepad1.triangle && robot.getArmAngle() > 100) {
                            scoring_extend = !scoring_extend;
                        }

                        if (scoring_extend) {
                            robot.setSlider(mosaicScoreSliderLength[mosaicScoreHeight]);
                        } else {
                            robot.setSlider(0);
                        }

                        if (mosaicScoreHeight == 0) robot.setArm(127);
                        else robot.setArm(120);
                    } else if (cycleMode) {
                        //cycle
                        robot.setArm(cycleScoreArmAngle[cycleScoreHeight]);

                        if (Gamepad1.triangle && !lastGamepad1.triangle && robot.getArmAngle() > 100) {
                            scoring_extend = !scoring_extend;
                        }

                        if (scoring_extend) {
                            robot.setSlider(900);
                        } else {
                            robot.setSlider(0);
                        }
                    }

                    if (robot.getArmAngle() > 70) {
                        robot.clawPScoring();
                    }

                    if (Gamepad1.left_trigger > 0) {
                        robot.rightClawOpen();
                    } else {
                        robot.rightClawClose();
                    }
                    if (Gamepad1.right_trigger > 0) {
                        robot.leftClawOpen();
                    } else {
                        robot.leftClawClose();
                    }

                    if (Gamepad1.right_bumper && !lastGamepad1.right_bumper) {
                        state = states.RETURN_TO_INIT;
                        timer1.reset();
                    }
                    if (Gamepad1.left_bumper && !lastGamepad1.left_bumper) {
                        state = states.READY_SCORE;
                        robot.setArm(0);
                        timer1.reset();
                    }
                    break;


                // Back to intake position
                case RETURN_TO_INIT:
                    scoring_extend = false;

                    boolean delayNeeded = false;
                    int diff = robot.slider.getTargetPosition() - 600;
                    if (diff > 0) delayNeeded = true;

                    robot.retractSlider();

                    if (delayNeeded) {
                        if (timer1.milliseconds() > diff * 6) robot.setArm(0);
                    } else {
                        robot.setArm(0);
                    }
                    // robot.arm.setVelocity(1600);

                    if (robot.getArmAngle() > 70 && robot.getArmAngle() < 100) {
                        robot.bothClawClose();
                    }

                    if (robot.getArmAngle() < 130) {
                        robot.setClawPAngle(180);
                    }

                    if (robot.getArmAngle() < 0) {
                        state = states.INIT;
                        timer1.reset();
                    }
                    break;

                case DRAW:
                    robot.setArm(drawArmAngle[drawScoreHeight]);
                    if (Gamepad1.right_trigger > 0 && !(lastGamepad1.right_trigger > 0)) drawScoreHeight += 1;
                    if (Gamepad1.left_trigger > 0 && !(lastGamepad1.left_trigger > 0)) drawScoreHeight -= 1;

                    if (robot.getArmAngle() > 90) robot.setClawPAngle(80);

                    if (Gamepad1.triangle && !lastGamepad1.triangle && robot.getArmAngle() > 100) {
                        scoring_extend = !scoring_extend;
                    }

                    if (scoring_extend) {
                        robot.setSlider(mosaicScoreSliderLength[mosaicScoreHeight]);
                    } else {
                        robot.setSlider(0);
                    }

                    if (Gamepad1.right_bumper && !lastGamepad1.right_bumper) {
                        state = states.RETURN_TO_INIT;
                        timer1.reset();
                    }
                    if (Gamepad1.left_bumper && !lastGamepad1.left_bumper) {
                        state = states.INIT;
                    }
            }

            //Arm height placement
            if (mosaicMode) {
                // mosaic
                if (Gamepad2.left_bumper && !lastGamepad2.left_bumper) {
                    mosaicScoreHeight += 1;
                }

                if (Gamepad2.right_bumper && !lastGamepad2.right_bumper) {
                    mosaicScoreHeight -= 1;
                }

                if (Gamepad2.cross && !lastGamepad2.cross) {
                    mosaicMode = false;
                    cycleMode = true;
                }
            } else if (cycleMode) {
                // cycle
                if (Gamepad2.right_bumper && !lastGamepad2.right_bumper) {
                    cycleScoreHeight += 1;
                }

                if (Gamepad2.left_bumper && !lastGamepad2.left_bumper) {
                    cycleScoreHeight -= 1;
                }

                if (Gamepad2.triangle && !lastGamepad2.triangle) {
                    mosaicMode = true;
                    cycleMode = false;
                }
            }

            if (mosaicScoreHeight < 0) {
                mosaicScoreHeight = 0;
            }
            if (mosaicScoreHeight > 8) {
                mosaicScoreHeight = 8;
            }

            if (cycleScoreHeight < 0) {
                cycleScoreHeight = 0;
            }
            if (cycleScoreHeight > 7) {
                cycleScoreHeight = 7;
            }

            if (drawScoreHeight < 0) {
                drawScoreHeight = 0;
            }
            if (drawScoreHeight > 8) {
                drawScoreHeight = 8;
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

            if (riggingState > 0) {
                pivot = Gamepad1.right_stick_x * 0.8;
            }

            if (Gamepad1.dpad_left && !lastGamepad1.dpad_left) {
                riggingState += 1;
            }

            if (Gamepad1.dpad_right && !lastGamepad1.dpad_right) {
                riggingState -= 1;
            }

            /*if (Gamepad1.dpad_left && Gamepad1.dpad_right) {
                robot.retractRiggingServo();
            }*/

            if (riggingState > 3) {
                riggingState = 3;
            }

            if (riggingState < 0) {
                riggingState = 0;
            }

            // Auto align
            if (Gamepad1.circle) {
                if (heading < -Math.PI / 2) heading += 2 * Math.PI;

                double align_output = heading_pid.calculate(
                        heading, -boardHeading
                );
                pivot = -align_output;
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


//            telemetry.addData("Encoder Angle (Degrees)", angle);
//            telemetry.addData("Encoder Angle - Normalized (Degrees)", angleNormalized);

            telemetry.addData("state", state);
            telemetry.addData("RiggingState", riggingState);
            telemetry.addLine();
            telemetry.addData("arm", robot.getArmAngle());
            telemetry.addData("slider", robot.slider.getCurrentPosition());
            telemetry.addLine();

            if (mosaicMode) telemetry.addData("Mode", "Mosaic");
            else telemetry.addData("Mode", "Cycle");

            telemetry.addData("Arm height", mosaicScoreSliderLength[mosaicScoreHeight]);
            telemetry.addData("Slider extension", cycleScoreArmAngle[cycleScoreHeight]);

            telemetry.update();
            drivetrain.remote(direction_y, direction_x, -pivot, heading);
        }
    }
}
