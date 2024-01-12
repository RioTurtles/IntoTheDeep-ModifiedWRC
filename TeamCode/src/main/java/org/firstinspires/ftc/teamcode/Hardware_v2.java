package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
   // DistanceSensor distanceSensor;
    //WebcamName webcam1;
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
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        //fwebcam1 = hardwareMap.get(WebcamName.class, "webcam1");
        this.telemetry = telemetry;

    }

    public void reset() {

        setMotorDirections();
        setMotorBrakes();
        //resetSliderPosition();
        resetClawPosition();
    }

    // Init methods.

    /**
     * Sets motor directions to its functional ones. Mostly used for mecanum drives.
     */
    private void setMotorDirections() {
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSliderRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSliderLeft.setDirection(DcMotorSimple.Direction.FORWARD);
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
                motorSliderLeft.setTargetPosition(1120);
                motorSliderRight.setTargetPosition(1120);
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
    public void setSliderPosition(int position) {setSliderPosition(position, 0.2);}

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
    public void resetSliderPosition() {resetSliderPosition(0.2);}

    /**
     * Claw control.
     * @param claw The claw to control, <code>upper</code>, or <code>lower</code>.
     * @param position The position of the claw. For an open claw, use <code>upper</code>; for a closed claw, use <code>closed</code>.
     */

    public void resetClawPosition() {
        setClawPosition("upper", "open");
        setClawPosition("lower", "open");

    }
    public void setClawPosition(String claw, String position) {
        if (claw.equals("upper") || claw.equals("up") || claw.equals("high")) {
            switch (position) {
                case "open":
                    servoClawUpper.setPosition(0.7);
                    telemetry.addData("UPPER", "OPEN");
                case "closed":
                    servoClawUpper.setPosition(0);
                    telemetry.addData("UPPER", "CLOSED");
            }
        } else if (claw.equals("lower") || claw.equals("low")) {
            switch (position) {
                case "open":
                    servoClawLower.setPosition(0);
                    telemetry.addData("LOWER", "OPEN");
                case "closed":
                    servoClawLower.setPosition(0.7);
                    telemetry.addData("LOWER", "CLOSED");
            }
        }
    }

    /**
     * Sets the robot's arm and claw to its scoring position.
     */
    public void setScoringPosition() {
        servoArmLeft.setPosition(0.7);
        servoArmRight.setPosition(0.7);
        servoClawPitchLeft.setPosition(0.7);
        servoClawPitchRight.setPosition(0.7);
    }

    /**
     * Sets the robot's arm and claw to its intake position.
     */
    public void setIntakePosition() {
        servoArmLeft.setPosition(0);
        servoArmRight.setPosition(0);
        servoClawPitchLeft.setPosition(0);
        servoClawPitchRight.setPosition(0);
    }

    /**
     * Resets the IMU's yaw value to 0.
     */
    public void resetIMUYaw() {
        imu.resetYaw();
    }

    /**
     * Gets the distance sensor's value.
     * @param unit The unit for the method to return.
     * @return A double value, the distance measured.
     */
    //public double getProximity(DistanceUnit unit) {
   //     return distanceSensor.getDistance(unit);
   // }

    /**
     * Gets the distance sensor's value. Unit is set to centimetres.
     * @return A double value, the distance measured.
     */
    //public double getProximity() {
      //  return getProximity(DistanceUnit.CM);
    //}
}