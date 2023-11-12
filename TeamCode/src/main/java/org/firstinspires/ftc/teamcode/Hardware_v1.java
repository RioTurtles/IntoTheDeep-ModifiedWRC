package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Hardware_v1 {
    Telemetry telemetry;
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotorEx motorSliderLeft;
    DcMotorEx motorSliderRight;
    CRServo servoArmRight;
    CRServo servoArmLeft;
    Servo servoIntakePitchLeft;
    Servo servoIntakePitchRight;
    Servo servoDroneUpper;
    Servo servoDroneLower;
    CRServo servoIntakeLeft;
    CRServo servoIntakeRight;
    IMU imu;

    int sliderPosition;
    ElapsedTime sliderTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        // Chassis
        this.motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        this.motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        this.motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        this.motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        // Slider
        this.motorSliderLeft = hardwareMap.get(DcMotorEx.class, "motorSliderLeft");
        this.motorSliderRight = hardwareMap.get(DcMotorEx.class, "motorSliderRight");
        // Arm
        this.servoArmLeft = hardwareMap.get(CRServo.class, "servoArmLeft");
        this.servoArmRight = hardwareMap.get(CRServo.class, "servoArmRight");
        // Intake
        this.servoIntakeLeft = hardwareMap.get(CRServo.class, "servoIntakeLeft");
        this.servoIntakeRight = hardwareMap.get(CRServo.class, "servoIntakeRight");
        // Intake pitch
        this.servoIntakePitchLeft = hardwareMap.get(Servo.class, "servoIntakePitchLeft");
        this.servoIntakePitchRight = hardwareMap.get(Servo.class, "servoIntakePitchRight");
        // Drone
        this.servoDroneUpper = hardwareMap.get(Servo.class, "servoDroneUpper");
        this.servoDroneLower = hardwareMap.get(Servo.class, "servoDroneLower");
        // IMU
        this.imu = hardwareMap.get(IMU.class, "imu");


        // Configuration
        // Chassis motors
        this.motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        // Brake
        this.motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // IMU
        this.imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        // Slider motors
        this.motorSliderRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorSliderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorSliderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Intake
        this.servoIntakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.servoIntakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        this.sliderPosition = 0;
    }

    public void reset() {
        this.resetIMU();
        this.sliderPosition = 0;
        this.sliderTimer.reset();
        this.setArmPosition(1);
    }

    public void resetIMU() {
        this.imu.resetYaw();
    }

    public void setSliderPosition(boolean direction) {
        if (direction) {
            this.motorSliderLeft.setPower(0.7);
            this.motorSliderRight.setPower(0.7);
        } else {
            this.motorSliderLeft.setPower(-0.7);
            this.motorSliderRight.setPower(-0.7);
        }
        this.motorSliderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorSliderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void setArmPosition(int position) {
        if (position == 0) {  // Intake position
            this.servoArmLeft.setPower(1);
            this.servoArmRight.setPower(-1);
        } else if (position == 1) {  // Scoring position
            this.servoArmLeft.setPower(-0.2);
            this.servoArmRight.setPower(0.2);
        }
    }

    public void startIntake(int speed) {  // Starts the intake.
        this.servoIntakeLeft.setPower(speed);
        this.servoIntakeRight.setPower(speed);
    }
    public void startIntake() {this.startIntake(1);}  // Overload function (i.e. make parameter "speed" optional and defaults to 1
    public void startOuttake(int speed) {this.startIntake(speed*-1);}  // Start outtake through reversing both servos
    public void startOuttake() {startOuttake(1);}  // Overload function for optional parameters
    public void stopIntake() {this.startIntake(0);}  // Stops the intake through setting powers of both servos to 0

    public void setIntakePitch(int position) {
        if (position == 0) {
            this.servoIntakePitchLeft.setPosition(-1);
            this.servoIntakePitchRight.setPosition(-1);
        } else if (position == 1) {
            this.servoIntakePitchLeft.setPosition(0.1);
            this.servoIntakePitchRight.setPosition(0.1);
        }
    }
}
