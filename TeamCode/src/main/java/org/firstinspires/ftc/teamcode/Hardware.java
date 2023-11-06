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
    DcMotor motorArmLeft;
    DcMotor motorArmRight;
    DcMotor motorRiggingLinearActuatorLeft;
    DcMotor motorRiggingLinearActuatorRight;
    CRServo servoTest;
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
        this.servoTest = hardwareMap.get(CRServo.class, "motorTest");
        /*
        this.motorArmLeft = hardwareMap.get(DcMotor.class, "motorArmLeft");
        this.motorArmRight = hardwareMap.get(DcMotor.class, "motorArmRight");
        this.motorRiggingLinearActuatorLeft = hardwareMap.get(DcMotor.class, "motorRiggingLinearActuatorLeft");
        this.motorRiggingLinearActuatorRight = hardwareMap.get(DcMotor.class, "motorRiggingLinearActuatorRight");
        this.servoIntakeLeft = hardwareMap.get(Servo.class, "servoIntakeLeft");
        this.servoIntakeRight = hardwareMap.get(Servo.class, "servoIntakeRight");
        this.servoDroneLauncher = hardwareMap.get(Servo.class, "servoDroneLauncher");
        this.sensorDistance = hardwareMap.get(DistanceSensor.class, "sensorDistance");
         */
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
