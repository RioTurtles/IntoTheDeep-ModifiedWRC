package org.firstinspires.ftc.teamcode.archive.apoc_wrc;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This class represents the robot object.
 */
public class Project1Hardware {
    DcMotor motorFL, motorFR;
    DcMotor motorBL, motorBR;
    IMU imu;
    Telemetry telemetry;
    DcMotorEx slider, arm, lRigging, rRigging;
    ServoImplEx leftClaw, rightClaw, clawP, drone;
    ServoImplEx lRiggingUp, rRiggingUp;
    DistanceSensor leftDis, rightDis;
    HardwareMap hwmap;

    double armTargetAngle;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");

        slider = hardwareMap.get(DcMotorEx.class, "slider");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        lRigging = hardwareMap.get(DcMotorEx.class,"lRigging");
        rRigging = hardwareMap.get(DcMotorEx.class,"rRigging");

        leftClaw = hardwareMap.get(ServoImplEx.class, "lClaw");
        rightClaw = hardwareMap.get(ServoImplEx.class, "rClaw");
        clawP = hardwareMap.get(ServoImplEx.class, "clawP");
        drone = hardwareMap.get(ServoImplEx.class,"drone");
        lRiggingUp = hardwareMap.get(ServoImplEx.class,"lRiggingUp");
        rRiggingUp = hardwareMap.get(ServoImplEx.class,"rRiggingUp");

        leftDis = hardwareMap.get(DistanceSensor.class, "lDis");
        rightDis = hardwareMap.get(DistanceSensor.class, "rDis");

        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
        this.telemetry = telemetry;
        this.armTargetAngle = getArmAngle();
    }

    public void reset() {
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slider.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lRigging.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rRigging.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lRigging.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rRigging.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        slider.setDirection(DcMotorEx.Direction.FORWARD);
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        lRigging.setDirection(DcMotorEx.Direction.REVERSE);
        rRigging.setDirection(DcMotorEx.Direction.FORWARD);

        lRiggingUp.setDirection(ServoImplEx.Direction.FORWARD);
        rRiggingUp.setDirection(ServoImplEx.Direction.REVERSE);
        leftClaw.setDirection(ServoImplEx.Direction.FORWARD);
        rightClaw.setDirection(ServoImplEx.Direction.REVERSE);
        //clawP.setDirection(ServoImplEx.Direction.REVERSE);
        drone.setDirection(Servo.Direction.FORWARD);
        arm.setPower(0);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        bothClawClose();
        setClawPAngle(180);

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));
    }

    // Regular methods.
    public void setSlider(int pos) {

        if (pos > 900) {
            pos = 900;
        }

        slider.setTargetPosition(pos + (int) (arm.getCurrentPosition() / 19));
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slider.setPower(1);
    }

    public boolean sliderInPosition(int tolerance) {
        return Math.abs(slider.getCurrentPosition() - slider.getTargetPosition()) < tolerance;
    }

    public void setScoringArm(double angle) {
        arm.setTargetPosition(angleToEncoderValueArm(angle));
    }
    public void setClaw() {
        leftClaw.setPosition(0.05);
        rightClaw.setPosition(0);
    }
    //Movements
    public void extendRiggingServo() {
        // lRiggingUp.setPosition(0.5);
        lRiggingUp.setPosition(1);
        rRiggingUp.setPosition(1);
    }
    /* public void retractRiggingServo() {
        lRiggingUp.setPosition(0);
        rRiggingUp.setPosition(0);
    }*/
    public void retractRiggingMotor() {
        lRigging.setPower(-1);
        rRigging.setPower(-1);
    }
    public void extendRiggingMotor() {
        lRigging.setPower(1);
        rRigging.setPower(1);
    }
    public void droneLaunch() {
        drone.setPosition(0.8);
    }
    public void bothClawOpen() {
        leftClaw.setPosition(0.15);
        rightClaw.setPosition(0.25);
    }

    public void leftClawOpen() {
        leftClaw.setPosition(0.15);
    }

    public void rightClawOpen() {
        rightClaw.setPosition(0.25);
    }

    public void leftClawClose() {
        leftClaw.setPosition(0.65);
        //leftClaw.setPosition(0.7);
    }

    public void rightClawClose() {
        rightClaw.setPosition(0.75);
        //rightClaw.setPosition(0.8);
    }

    public void bothClawClose() {
        rightClawClose();
        leftClawClose();
    }

    //Extensions
    public void extendSlider() {
        slider.setPower(1);
        slider.setTargetPosition(1000);
        slider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    //Retractions
    public void retractSlider() {
        slider.setPower(1);
        slider.setTargetPosition(0);
        slider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
    public void setClawPAngle(double angle) {
        clawP.setPosition(angle / 204);
    }

    public int angleToEncoderValueArm(double angle) {
        double CPR = 3895.9;
        double revolutions = (angle + 12) / 360;
        double tmp = revolutions * CPR;
        return (int) tmp;
    }

    public void setArm(double angle) {
        armTargetAngle = angle;
        int tmp;
        //arm.setVelocity(1300);
        arm.setPower(1);

        tmp = angleToEncoderValueArm(angle);
        if(tmp > 1900){
            tmp = 1900;
        }
        arm.setPositionPIDFCoefficients(5);

        arm.setTargetPosition(angleToEncoderValueArm(angle));
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double getArmAngle() {
        double CPR = 3895.9;
        int position = arm.getCurrentPosition();
        double revolutions = position / CPR;
        double angle = revolutions * 360 - 12;
        return angle;
    }

    public double getArmError() {
        return Math.abs(getArmAngle() - armTargetAngle);
    }

    public int lengthToEncoderValueSlider(double length) {
        double CPR = 145.1 * 1.4;
        double revolutions = length / 35.65 / Math.PI;
        double tmp = revolutions * CPR;
        return (int) tmp;
    }

    public void setSliderLength(double length) {
        // if(length < 10) length = 0;
        //if (length > 1000) length = 1000;
        slider.setTargetPosition(lengthToEncoderValueSlider(length));

        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(1);
    }

    public double getSliderLength() {
        double CPR = 145.1 * 1.4;
        int position = slider.getCurrentPosition();
        double revolutions = position / CPR;
        double length = revolutions * 35.65 * Math.PI;
        return length;
    }
    public void clawPIntake() {
        setClawPAngle(90 - getArmAngle() * 0.5 - 33);
    }
    public void clawPIntakeExtend() {
        setClawPAngle(90 - getArmAngle() * 0.5 - 38);
    }

    /*public void clawPIntakeExtendDynamic() {
        setClawPAngle(
                (90 - getArmAngle() * 0.5 - 33)
                + ((90 - getArmAngle() * 0.5 - 33) - (90 - getArmAngle() * 0.5 - 38))
                * slider.getCurrentPosition() / 700
        );
    }*/

    public void clawPScoring() {
        // setClawPAngle(180 - getArmAngle() + 8);
         setClawPAngle(180 - getArmAngle() + 90 + 35);
        // setClawPAngle(180);
    }
    public void clawPScoring2() {
        setClawPAngle(180 - getArmAngle() + 90 - 105);
    }

    public interface SleepFunction {
        void call();
    }

    /*public void retractAndResetSlider(SleepFunction sleepFunction) {
        slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slider.setPower(-1);
        sleepFunction.call();
        slider.setPower(0);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(slider.getCurrentPosition());
        slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slider.setPower(0);
    }*/

    public void retractAndResetSlider(SleepFunction sleepFunction) {
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(slider.getCurrentPosition());
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(-0.8);
        slider.setTargetPosition(-100);
        sleepFunction.call();
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /*public void resetSlider(SleepFunction sleepFunction) {
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(slider.getCurrentPosition());
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(-100);
        slider.setPower(1);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleepFunction.call();
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }*/

    public void resetRetractSlider(SleepFunction sleepFunction) {
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(0);
        slider.setPower(1);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(-100);
        slider.setPower(1);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleepFunction.call();
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void resetArm(SleepFunction sleepFunction) {
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm.setTargetPosition(0);
//        arm.setPower(1);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setArm(getArmAngle() - 10.0);
//        arm.setPower(1);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setPower(-0.5);
        sleepFunction.call();
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void resetArmEncoder(SleepFunction sleepFunction) {
        arm.setPower(0);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        sleepFunction.call();
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void releaseSliderPower() {
        slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slider.setPower(0);
    }
}