package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotorEx motorSliderLeft;
    DcMotorEx motorSliderRight;
    Servo servoArmRight;
    Servo servoArmLeft;
    Servo servoClaw;
    IMU imu;

    public void init(HardwareMap hardwareMap) {
        // Chassis
        this.motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        this.motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        this.motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        this.motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        // Slider
        this.motorSliderLeft = hardwareMap.get(DcMotorEx.class, "motorSliderLeft");
        this.motorSliderRight = hardwareMap.get(DcMotorEx.class, "motorSliderRight");
        // Arm
        this.servoArmLeft = hardwareMap.get(Servo.class, "servoArmLeft");
        this.servoArmRight = hardwareMap.get(Servo.class, "servoArmRight");
        // Claw
        this.servoClaw = hardwareMap.get(Servo.class, "servoClawLeft");
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
        this.motorSliderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorSliderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void reset() {
        this.resetIMU();
        this.motorSliderLeft.getCurrentPosition();
    }

    public void resetIMU() {
        this.imu.resetYaw();
    }

    public void setSliderPosition(int position) {
        if (position == 0) {  // Retract slider
            this.motorSliderLeft.setTargetPosition(0);
            this.motorSliderRight.setTargetPosition(0);
        } else if (position == 1) {  // 1st set height
            this.motorSliderLeft.setTargetPosition(1583);
            this.motorSliderRight.setTargetPosition(-781);
        } else if (position == 2) {  // 2nd set height
            this.motorSliderLeft.setTargetPosition(2007);
            this.motorSliderRight.setTargetPosition(-1008);
        } else if (position == 3) {  // 3rd set height
            this.motorSliderLeft.setTargetPosition(2328);
            this.motorSliderRight.setTargetPosition(-1175);
        }
        this.motorSliderLeft.setPower(1);
        this.motorSliderRight.setPower(1);
        this.motorSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setArmPosition(int position) {
        if (position == 0) {
            this.servoArmLeft.setPosition(90);  // Intake position
            this.servoArmRight.setPosition(-90);
        } else if (position == 1) {
            this.servoArmLeft.setPosition(270);
            this.servoArmRight.setPosition(-270);
        }
    }

    public void openClaw(int side) {
        servoClaw.setPosition(1);
    }

    public void closeClaw(int side) {
        this.servoClaw.setPosition(0);
    }
}
