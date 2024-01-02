package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Hardware_v1 {
    Telemetry telemetry;
    DcMotor motorFrontL, motorFrontR, motorBackL, motorBackR;
    DcMotorEx motorSliderL, motorSliderR;
    CRServo servoArmR, servoArmL, servoIntakeR, servoIntakeL;
    Servo servoIntakePitchL, servoIntakePitchR, servoDroneH, servoDroneL;
    IMU imu;
    private int sliderHeight; // 0 = intake, 1 = scoring
    // ElapsedTime sliderTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    // robot constructor
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        // Chassis
        motorFrontL = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontR = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackL = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackR = hardwareMap.get(DcMotor.class, "motorBackRight");
        // Slider
        motorSliderL = hardwareMap.get(DcMotorEx.class, "motorSliderLeft");
        motorSliderR = hardwareMap.get(DcMotorEx.class, "motorSliderRight");
        // Arm
        servoArmL = hardwareMap.get(CRServo.class, "servoArmLeft");
        servoArmR = hardwareMap.get(CRServo.class, "servoArmRight");
        // Intake
        servoIntakeL = hardwareMap.get(CRServo.class, "servoIntakeLeft");
        servoIntakeR = hardwareMap.get(CRServo.class, "servoIntakeRight");
        // Intake pitch
        servoIntakePitchL = hardwareMap.get(Servo.class, "servoIntakePitchLeft");
        servoIntakePitchR = hardwareMap.get(Servo.class, "servoIntakePitchRight");
        // Drone
        servoDroneH = hardwareMap.get(Servo.class, "servoDroneUpper");
        servoDroneL = hardwareMap.get(Servo.class, "servoDroneLower");
        // IMU
        imu = hardwareMap.get(IMU.class, "imu");


        // Config
        // Chassis motors
        motorBackL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackR.setDirection(DcMotorSimple.Direction.REVERSE);
        // Brake
        motorFrontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // IMU
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        // Slider motors
        motorSliderR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSliderL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSliderR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Intake
        servoIntakeR.setDirection(DcMotorSimple.Direction.REVERSE);
        // Arm
        servoArmR.setDirection(DcMotorSimple.Direction.REVERSE);
        // Variables
        sliderHeight = 0;
    }

    public void reset() {
        resetIMU();
        // sliderTimer.reset();
        setArmPosition(1);
    }

    public void resetIMU() {
        imu.resetYaw();
    }

    public void setSliderPosition(boolean direction) { // less power (0.7)
        if (direction) { // Scoring
            sliderHeight = 1;
            motorSliderL.setPower(0.7);
            motorSliderR.setPower(0.7);
        } else { // Intake
            sliderHeight = 0;
            motorSliderL.setPower(-0.7);
            motorSliderR.setPower(-0.7);
        }
        motorSliderL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSliderR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setArmPosition(int position) { // Arm position
        if (position == 0) {  // Scoring
            servoArmL.setPower(1);
            servoArmR.setPower(-1);
        } else if (position == 1) {  // Intake
            servoArmL.setPower(-0.2);
            servoArmR.setPower(0.2);
        }
    }

    public void servoIntakeSetSpeed(int speed) {
        servoIntakeL.setPower(speed);
        servoIntakeR.setPower(speed);
    }
    public void startIntake(int speed) {
        servoIntakeSetSpeed(speed);} // servo forward
    public void startIntake() {startIntake(1);}
    public void startOuttake(int speed) {
        servoIntakeSetSpeed(speed*-1);}  // reverse servos
    public void startOuttake() {startOuttake(1);}
    public void stopIntake() {
        servoIntakeSetSpeed(0);}  //  set powers of both servos to 0

    public void setIntakePitch(int position) { // Intake pitch position
        if (position == 0) { // Scoring
            this.servoIntakePitchL.setPosition(-1);
            this.servoIntakePitchR.setPosition(-1);
        } else if (position == 1) { // Intake
            this.servoIntakePitchL.setPosition(0.1);
            this.servoIntakePitchR.setPosition(0.1);
        }
    }

    public int getSliderHeight() { // Slider height variable getter
        return sliderHeight;
    }

    public void setSliderHeight(int position) { // Slider height variable setter
        if (position < 0 || position > 1)
            return;

        sliderHeight = position;
    }
}
