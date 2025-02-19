package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * This class represents the robot object. This is a modified version in order to avoid conflicts
 * with TeleOp hardware classes.
 */
public class Project2Hardware {
    MecanumDrive drivetrain;
    DcMotor frontLeft, frontRight;
    DcMotor backLeft, backRight;
    DcMotorEx slider, arm;
    ServoImplEx claw;
    IMU imu;

    ScoringMode scoringMode;
    ScoringHeight scoringHeight;
    double armTargetAngle;
    boolean clawClosed;

    public static final double BASKET_ANGLE = 75.5;

    public Project2Hardware(@NonNull HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        slider = hardwareMap.get(DcMotorEx.class, "slider");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));

        reset();

        drivetrain = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        this.armTargetAngle = getArmAngle();
        this.scoringMode = ScoringMode.BASKET;
        this.scoringHeight = ScoringHeight.HIGH;
        this.clawClosed = false;

    }

    private void reset() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slider.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        slider.setDirection(DcMotorEx.Direction.REVERSE);
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);

        arm.setPower(0);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // Regular methods.
    public void setSlider(int pos) {
        if (pos > 1375) pos = 1375;

        slider.setTargetPosition(pos + (arm.getCurrentPosition() / 19));
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slider.setPower(1);
    }

    public int getSlider() {
        return (int) (slider.getCurrentPosition() - (arm.getCurrentPosition() / 19));
    }

    public boolean sliderInPosition(int tolerance) {
        return Math.abs(lengthToEncoderValueSlider(getSliderLength()) - slider.getTargetPosition())
                < tolerance;
    }

    public void setScoringArm(double angle) {
        arm.setTargetPosition(angleToEncoderValueArm(angle));
    }

    // Extensions
    public void extendSlider() {
        slider.setPower(1);
        setSlider(1000);
        slider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    // Retractions
    public void retractSlider() {
        slider.setPower(1);
        slider.setTargetPosition(0);
        slider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    private int angleToEncoderValueArm(double angle) {
        double CPR = 3895.9;
        double revolutions = angle / 360;
        double tmp = revolutions * CPR;
        return (int) tmp;
    }

    public void setArm(double angle) {
        armTargetAngle = angle;
//        arm.setVelocity(1300);
        arm.setPower(1);

        int tmp = angleToEncoderValueArm(angle);
        if (tmp > 1900) tmp = 1900;
        arm.setPositionPIDFCoefficients(5);
        arm.setTargetPosition(angleToEncoderValueArm(angle));
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double getArmAngle() {
        double CPR = 3895.9;
        int position = arm.getCurrentPosition();
        double revolutions = position / CPR;
        return revolutions * 360;
    }

    public double getArmError() {return Math.abs(getArmAngle() - armTargetAngle);}

    public int lengthToEncoderValueSlider(double length) {
        double CPR = 145.1 * 1.4;
        double revolutions = length / 35.65 / Math.PI;
        double tmp = revolutions * CPR;
        return (int) tmp;
    }

    public void setSliderLength(double length) {
//        if(length < 10) length = 0;
//        if (length > 1000) length = 1000;
        slider.setTargetPosition(lengthToEncoderValueSlider(length));
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(1);
    }

    public double getSliderLength() {
        double CPR = 145.1 * 1.4;
        int position = slider.getCurrentPosition();
        double revolutions = position / CPR;
        return revolutions * 35.65 * Math.PI;
    }

    public void powerResetArm() {
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(-0.8);
    }

    public void powerResetSlider() {
        slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slider.setPower(-0.8);
    }

    public void resetArm() {
        arm.setPower(0);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(arm.getCurrentPosition());
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetSlider() {
        slider.setPower(0);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(slider.getCurrentPosition());
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void clawOpen() {claw.setPosition(0.55); clawClosed = false;}
    public void clawClose() {claw.setPosition(0.16); clawClosed = true;}
    public String getClawString() {if (clawClosed) return "CLOSED"; else return "OPEN";}
    public String getScoringState() {return scoringMode + " | " + scoringHeight;}
    public double getIMUYaw() {return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);}
}

class MecanumDrive{
    double sin, cos, theta, max, power;
    double vertical, horizontal, pivot, heading;
    double powerFL, powerFR, powerBL, powerBR;
    DcMotor frontLeft, frontRight, backLeft, backRight;

    public MecanumDrive(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
        this.frontLeft = fl;
        this.frontRight = fr;
        this.backLeft = bl;
        this.backRight = br;
    }

    public void remote(double vertical, double horizontal, double pivot, double heading) {
        this.vertical = vertical;
        this.horizontal = horizontal;
        this.pivot = pivot;
        this.heading = heading ;

        theta = 2 * Math.PI + Math.atan2(vertical, horizontal) - heading;
        power = Math.hypot(horizontal, vertical);

        sin = Math.sin(theta - Math.PI/4);
        cos = Math.cos(theta - Math.PI/4);
        max = Math.max(Math.abs(sin), Math.abs(cos));

        powerFL = power * (cos/max) + pivot;
        powerFR = power * (sin/max) - pivot;
        powerBL = power * -(sin/max) - pivot;
        powerBR = power * -(cos/max) + pivot;

        frontLeft.setPower(-powerFL);
        frontRight.setPower(-powerFR);
        backLeft.setPower(powerBL);
        backRight.setPower(powerBR);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Deprecated
    public void remote2(double vertical, double horizontal, double pivot, double heading) {
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        this.vertical = vertical;
        this.horizontal = horizontal;
        this.pivot = pivot;
        this.heading = heading + (Math.PI/2);

        theta = 2 * Math.PI + Math.atan2(vertical,horizontal) - heading;
        power = Math.hypot(horizontal, vertical);

        sin = Math.sin(theta - Math.PI/4);
        cos = Math.cos(theta - Math.PI/4);
        max = Math.max(Math.abs(sin), Math.abs(cos));


//        FLPower = power * (cos/max) + pivot;
//        FRPower = power * (sin/max) - pivot;
//        BLPower = power * (sin/max) + pivot;
//        BRPower = power * (cos/max) - pivot;

        powerFL = power * (cos/max) - pivot;
        powerFR = power * (sin/max) + pivot;
        powerBL = power * (sin/max) - pivot;
        powerBR = power * (cos/max) + pivot;

        frontLeft.setPower(powerFL);
        frontRight.setPower(powerFR);
        backLeft.setPower(powerBL);
        backRight.setPower(powerBR);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Deprecated
    public void part1(double theta, double pivot, double power) {
        theta = 2 * Math.PI + (theta / 360 * 2 * Math.PI) - Math.PI / 2;

        sin = Math.sin(theta -Math.PI/4);
        cos = Math.cos(theta - Math.PI/4);
        max = Math.max(Math.abs(sin), Math.abs(cos));

        powerFL = power * (cos/max) + pivot;
        powerFR = power * sin/max - pivot;
        powerBL = power * -(sin/max) - pivot;
        powerBR = power * -(cos/max) + pivot;

        frontLeft.setPower(-powerFL);
        frontRight.setPower(-powerFR);
        backLeft.setPower(powerBL);
        backRight.setPower(powerBR);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Deprecated
    public void drive(double target, double power, double pivot, double distance) {
        this.theta = Math.PI + (target * Math.PI/180);
        sin = Math.sin(theta - Math.PI/4);
        cos = Math.cos(theta - Math.PI/4);
        max = Math.max(Math.abs(sin), Math.abs(cos));

        int FL = frontLeft.getCurrentPosition();
        int FR = frontRight.getCurrentPosition();
        int BL = backLeft.getCurrentPosition();
        int BR = backRight.getCurrentPosition();

        double orig = FL;
        double cur = orig;

        while (Math.abs(cur - orig) <= distance) {
            FL = frontLeft.getCurrentPosition();
            FR = frontRight.getCurrentPosition();
            BL = backLeft.getCurrentPosition();
            BR = backRight.getCurrentPosition();

            cur = FL;

            powerFL = power * -(cos/max) + pivot;
            powerFR = power * sin/max + pivot;
            powerBL = power * -(sin/max) + pivot;
            powerBR = power * cos/max + pivot;

            frontLeft.setPower(-powerFL);
            frontRight.setPower(-powerFR);
            backLeft.setPower(powerBL);
            backRight.setPower(powerBR);

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
}

enum ScoringMode {BASKET, CHAMBER}
enum ScoringHeight {HIGH, LOW}
