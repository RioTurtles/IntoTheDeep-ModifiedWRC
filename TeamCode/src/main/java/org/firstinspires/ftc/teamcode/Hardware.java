package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

public class Hardware {
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor motorHeightLeft;
    DcMotor motorHeightRight;
    Servo servoArmRight;
    Servo servoArmLeft;
    Servo servoIntakePitchLeft;
    Servo servoIntakePitchRight;
    Servo servoIntakeLeft;
    Servo servoIntakeRight;
    Servo servoDroneLauncher;
    DistanceSensor sensorDistance;
    IMU imu;

    public void init(HardwareMap hardwareMap) {
        this.motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        this.motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        this.motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        this.motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        this.motorHeightLeft = hardwareMap.get(DcMotor.class, "motorHeightLeft");
        this.motorHeightRight = hardwareMap.get(DcMotor.class, "motorHeightRight");
        this.servoArmLeft = hardwareMap.get(Servo.class, "servoArmLeft");
        this.servoArmRight = hardwareMap.get(Servo.class, "servoArmRight");
        this.servoIntakeLeft = hardwareMap.get(Servo.class, "servoIntakeLeft");
        this.servoIntakeRight = hardwareMap.get(Servo.class, "servoIntakeRight");
        this.servoDroneLauncher = hardwareMap.get(Servo.class, "servoDroneLauncher");
        this.sensorDistance = hardwareMap.get(DistanceSensor.class, "sensorDistance");
        this.imu = hardwareMap.get(IMU.class, "imu");
    }

    public void reset() {
        this.motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
    }

    public void resetIMU() {
        this.imu.resetYaw();
    }
}
