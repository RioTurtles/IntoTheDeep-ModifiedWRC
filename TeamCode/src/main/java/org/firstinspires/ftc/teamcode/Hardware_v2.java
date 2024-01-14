package org.firstinspires.ftc.teamcode;

import com.fasterxml.jackson.databind.deser.UnresolvedForwardReference;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This class represents the robot object.
 */
public class Hardware_v2 {
    DcMotor motorFL, motorFR;
    DcMotor motorBL, motorBR;
    DcMotorEx motorSliderLeft, motorSliderRight;
    Servo servoClawUpper, servoClawLower;
    Servo servoClawPitchLeft, servoClawPitchRight;
    Servo servoArmLeft, servoArmRight;
    IMU imu;
    Telemetry telemetry;

    int sliderPosition = 0;

    /**
     * Init method. Call upon the initialisation of an OpMode. Maps hardware to its variables. Call <code>reset()</code> afterwards.
     * @param hardwareMap the HardwareMap object used to map hardware.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorSliderLeft = hardwareMap.get(DcMotorEx.class, "motorSliderLeft");
        motorSliderRight = hardwareMap.get(DcMotorEx.class, "motorSliderRight");
        servoClawUpper = hardwareMap.get(Servo.class, "servoClawUpper");
        servoClawLower = hardwareMap.get(Servo.class, "servoClawLower");
        servoClawPitchLeft = hardwareMap.get(Servo.class, "servoClawPitchLeft");
        servoClawPitchRight = hardwareMap.get(Servo.class, "servoClawPitchRight");
        servoArmLeft = hardwareMap.get(Servo.class, "servoArmLeft");
        servoArmRight = hardwareMap.get(Servo.class, "servoArmRight");
        imu = hardwareMap.get(IMU.class, "imu");
        this.telemetry = telemetry;
    }

    public void reset() {
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorSliderLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorSliderRight.setDirection(DcMotorSimple.Direction.REVERSE);
        servoClawPitchLeft.setDirection(Servo.Direction.FORWARD);
        servoClawPitchRight.setDirection(Servo.Direction.REVERSE);
        servoClawUpper.setDirection(Servo.Direction.FORWARD);
        servoClawLower.setDirection(Servo.Direction.FORWARD);
        servoArmLeft.setDirection(Servo.Direction.FORWARD);
        servoArmRight.setDirection(Servo.Direction.REVERSE);

        motorSliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Sets motor brakes. Used for most motors.
     */
    private void setMotorBrakes() {
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSliderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSliderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Resets slider live encoder position values. Called upon initialisation.
     */
    private void resetSliderPositionValues() {
        motorSliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void resetServoDirections() {
        servoClawLower.setDirection(Servo.Direction.REVERSE);
        servoClawPitchLeft.setDirection(Servo.Direction.REVERSE);
        servoArmLeft.setDirection(Servo.Direction.REVERSE);
    }

    // Regular methods.
    /**
     * Sets slider positions.
     * @param position The target position, typically the backdrop's set line height. Integer value from 0-3.
     * @param power The speed of the motors to run on. Double value from 0-1.
     */
    public void setSliderPosition(int position, double power) {
        sliderPosition = position;
        switch (position) {
            case 0:
                motorSliderLeft.setTargetPosition(0);
                motorSliderRight.setTargetPosition(0);
                break;
            case 1:
                motorSliderLeft.setTargetPosition(900);
                motorSliderRight.setTargetPosition(900);
        }

        motorSliderLeft.setPower(power);
        motorSliderRight.setPower(power);
        motorSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Sets slider positions. Speed is set to 1. Pass a speed parameter for speed control.
     * @param position The target position, typically the backdrop's set line height. Integer value from 0-3.
     */
    public void setSliderPosition(int position) {setSliderPosition(position, 0.7);}

    /**
     * Sets sliders' heights back to 0.
     * @param power The speed of the motors to run on. Double value from 0-1.
     */
    public void resetSliderPosition(double power) {
        sliderPosition = 0;
        setSliderPosition(0, power);
    }

    /**
     * Resets slider's heights back to 0. Speed is set to 1. Pass a speed parameter for speed control.
     */
    public void resetSliderPosition() {resetSliderPosition(0.7);}

    public void resetClawPosition() {
        setClawPosition("upper", "open");
        setClawPosition("lower", "open");
    }

    public void resetIntakePitch() {
        servoClawPitchLeft.setPosition(0);
        servoClawPitchRight.setPosition(0);
    }

    public void resetArmPosition() {
        servoArmLeft.setPosition(0);
        servoArmRight.setPosition(0);
    }

    public void setClawPosition(String claw, String position) {
        if (claw.equals("upper") || claw.equals("up") || claw.equals("high")) {
            switch (position) {
                case "open":
                    servoClawUpper.setPosition(0);
                    telemetry.addData("UPPER", "OPEN");
                case "closed":
                    servoClawUpper.setPosition(0.22);
                    telemetry.addData("UPPER", "CLOSED");
            }
        } else if (claw.equals("lower") || claw.equals("low")) {
            switch (position) {
                case "open":
                    servoClawLower.setPosition(1);
                    telemetry.addData("LOWER", "OPEN");
                case "closed":
                    servoClawLower.setPosition(0.74);
                    telemetry.addData("LOWER", "CLOSED");
            }
        }
    }

    /**
     * Sets the robot's arm and claw to its intake position.
     */
    public void setIntakePosition() {
        servoArmLeft.setPosition(0.967);
        servoArmRight.setPosition(0.967);
        servoClawPitchLeft.setPosition(0.46);
        servoClawPitchRight.setPosition(0.46);
    }

    /**
     * Sets the robot's arm and claw to its scoring position.
     */
    public void setScoringPosition() {
        servoClawPitchLeft.setPositi///on(0.25);
        servoClawPitchRight.setPosition(0.25);
        servoArmLeft.setPosition(0.4);
        servoArmRight.setPosition(0.4);
    }

    /**
     * Resets the IMU's yaw value to 0.
     */
    public void resetIMUYaw() {
        imu.resetYaw();
    }
}