package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        this.telemetry = telemetry;
    }

    public void reset() {
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
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

    // Regular methods.
    /**
     * Sets slider positions.
     * @param position The target position, typically the backdrop's set line height. Integer value from 0-2.
     * @param power The speed of the motors to run on. Double value from 0-1.
     */
    public void setSliderPosition(int position, double power) {
        sliderPosition = position;
        switch (position) {
            case 0:  // Fully lowered.
                motorSliderLeft.setTargetPosition(0);
                motorSliderRight.setTargetPosition(0);
                break;
            case 1:  // Backdrop scoring position.
                motorSliderLeft.setTargetPosition(1120);
                motorSliderRight.setTargetPosition(1120);
                break;
            case 2:  // Rigging hold position.
                motorSliderLeft.setTargetPosition(310);
                motorSliderRight.setTargetPosition(310);
        }

        motorSliderLeft.setPower(power);
        motorSliderRight.setPower(power);
        motorSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Sets slider positions. Speed is set to 1. Pass a speed parameter for speed control.
     * @param position The target position, typically the backdrop's set line height. Integer value from 0-2.
     */
    public void setSliderPosition(int position) {setSliderPosition(position, 0.7);}

    public void setUpperClaw(int position) {
        switch (position) {
            case 0:
                servoClawUpper.setPosition(0);
            case 1:
                servoClawUpper.setPosition(0.22);
        }
    }

    public void setLowerClaw(int position) {
        switch (position) {
            case 0:
                servoClawLower.setPosition(1);
            case 1:
                servoClawLower.setPosition(0.74);
        }
    }

    /**
     * Sets the robot's arm and claw to its intake position.
     */
    public void setIntakePosition() {
        servoArmLeft.setPosition(0.963);
        servoArmRight.setPosition(0.963);
        servoClawPitchLeft.setPosition(0.45);
        servoClawPitchRight.setPosition(0.45);
    }

    /**
     * Sets the robot's arm and claw to its scoring position.
     */
    public void setScoringPosition() {
        servoClawPitchLeft.setPosition(0.16);
        servoClawPitchRight.setPosition(0.16);
        servoArmLeft.setPosition(0.45);
        servoArmRight.setPosition(0.45);
    }

    /**
     * Resets the IMU's yaw value to 0.
     */
    @Deprecated
    public void resetIMUYaw() {
        imu.resetYaw();
    }
}