package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Hardware_v2 {
    DcMotor motorFL, motorFR;
    DcMotor motorBL, motorBR;
    DcMotorEx motorSliderLeft, motorSliderRight;
    Servo servoClawUpper, servoClawLower;
    DistanceSensor distanceSensor;
    WebcamName webcam1;

    int sliderPosition = 0;

    public Hardware_v2(HardwareMap hardwareMap) {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorSliderLeft = hardwareMap.get(DcMotorEx.class, "motorSliderLeft");
        motorSliderRight = hardwareMap.get(DcMotorEx.class, "motorSliderRight");
        servoClawUpper = hardwareMap.get(Servo.class, "servoClawUpper");
        servoClawLower = hardwareMap.get(Servo.class, "servoClawLower");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        webcam1 = hardwareMap.get(WebcamName.class, "webcam1");

        setMotorDirections();
        setMotorBrakes();
        resetSliderPositionValues();
    }

    // Init methods.
    private void setMotorDirections() {
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void setMotorBrakes() {
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void resetSliderPositionValues() {
        motorSliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setSliderPosition(int position, double power) {
        sliderPosition = position;
        switch (position) {
            case 0:
                motorSliderLeft.setTargetPosition(0);
                motorSliderRight.setTargetPosition(0);
                break;
            case 1:
                motorSliderLeft.setTargetPosition(600);
                motorSliderRight.setTargetPosition(600);
                break;
            case 2:
                motorSliderLeft.setTargetPosition(120);
                motorSliderRight.setTargetPosition(120);
                break;
            case 3:
                motorSliderLeft.setTargetPosition(180);
                motorSliderRight.setTargetPosition(180);
                break;
        }

        motorSliderLeft.setPower(power);
        motorSliderRight.setPower(power);
        motorSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setSliderPosition(int position) {setSliderPosition(position, 1);}

    public void resetSliderPosition(double power) {
        sliderPosition = 0;
        setSliderPosition(0, power);
    }

    public void setClawPosition(String claw, String position) {
        if (claw.equals("upper") || claw.equals("up") || claw.equals("high")) {
            switch (position) {
                case "open":
                    servoClawUpper.setPosition(0.7);
                case "closed":
                    servoClawUpper.setPosition(0);
            }
        } else if (claw.equals("lower") || claw.equals("low")) {
            switch (position) {
                case "open":
                    servoClawLower.setPosition(0.7);
                case "closed":
                    servoClawLower.setPosition(0);
            }
        }
    }

    public void resetSliderPosition() {resetSliderPosition(1);}

    public double getProximity(DistanceUnit unit) {
        return distanceSensor.getDistance(unit);
    }

    public double getProximity() {
        return getProximity(DistanceUnit.CM);
    }
}
