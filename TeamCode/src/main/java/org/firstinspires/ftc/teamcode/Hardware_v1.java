package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;

public class Hardware_v1 {
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor motorArmLeft;
    DcMotor motorArmRight;
    DcMotor motorRiggingLinearActuatorLeft;
    DcMotor motorRiggingLinearActuatorRight;
    Servo servoClawLeft;
    Servo servoClawRight;
    Servo servoDroneLauncher;
    DistanceSensor sensorDistance;
    IMU imu;

    public void init(HardwareMap hardwareMap) {
        this.motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        this.motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        this.motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        this.motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        this.motorArmLeft = hardwareMap.get(DcMotor.class, "motorArmLeft");
        this.motorArmRight = hardwareMap.get(DcMotor.class, "motorArmRight");
        this.motorRiggingLinearActuatorLeft = hardwareMap.get(DcMotor.class, "motorRiggingLinearActuatorLeft");
        this.motorRiggingLinearActuatorRight = hardwareMap.get(DcMotor.class, "motorRiggingLinearActuatorRight");
        this.servoClawLeft = hardwareMap.get(Servo.class, "servoClawLeft");
        this.servoClawRight = hardwareMap.get(Servo.class, "servoClawRight");
        this.servoDroneLauncher = hardwareMap.get(Servo.class, "servoDroneLauncher");
        this.sensorDistance = hardwareMap.get(DistanceSensor.class, "sensorDistance");
        this.imu = hardwareMap.get(IMU.class, "imu");
    }

    public void reset() {
        this.motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorArmLeft.setTargetPosition(Constants.armLeftRetracted);
        this.motorArmRight.setTargetPosition(Constants.armRightRetracted);
        this.imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
    }


}
