package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class    MecanumDrive{
    double max;
    double sin;
    double cos;
    double theta;
    double power;
    double vertical;
    double horizontal;
    double pivot;
    double heading;
    double FLPower;
    double FRPower;
    double BLPower;
    double BRPower;
    Project1Hardware robot;



    public MecanumDrive(Project1Hardware robot){
        this.robot = robot;
    }

    public void remote(double vertical, double horizontal, double pivot, double heading){
        this.vertical = vertical;
        this.horizontal = horizontal;
        this.pivot = pivot;
        this.heading = heading ;

        theta = 2 * Math.PI + Math.atan2(vertical,horizontal) - heading;
        power = Math.hypot(horizontal,vertical);

        sin = Math.sin(theta -Math.PI/4);
        cos = Math.cos(theta - Math.PI/4);
        max = Math.max(Math.abs(sin),Math.abs(cos));

        FLPower = power * (cos/max) + pivot;
        FRPower = power * sin/max - pivot;
        BLPower = power * -(sin/max) - pivot;
        BRPower = power * -(cos/max) + pivot;

        robot.motorFL.setPower(-FLPower);
        robot.motorFR.setPower(-FRPower);
        robot.motorBL.setPower(BLPower);
        robot.motorBR.setPower(BRPower);

        robot.motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //also mecanum drive but more organized - not tested yet
    public void remote2(double vertical, double horizontal, double pivot, double heading){
        robot.motorFL.setDirection(DcMotor.Direction.FORWARD);
        robot.motorFR.setDirection(DcMotor.Direction.REVERSE);
        robot.motorBL.setDirection(DcMotor.Direction.FORWARD);
        robot.motorBR.setDirection(DcMotor.Direction.REVERSE);

        this.vertical = vertical;
        this.horizontal = horizontal;
        this.pivot = pivot;
        this.heading = heading+(Math.PI/2);

        theta = 2 * Math.PI + Math.atan2(vertical,horizontal) - heading;
        power = Math.hypot(horizontal,vertical);

        sin = Math.sin(theta -Math.PI/4);
        cos = Math.cos(theta - Math.PI/4);
        max = Math.max(Math.abs(sin),Math.abs(cos));

        /*
        FLPower = power * (cos/max) + pivot;
        FRPower = power * (sin/max) - pivot;
        BLPower = power * (sin/max) + pivot;
        BRPower = power * (cos/max) - pivot;
        */
        FLPower = power * (cos/max) - pivot;
        FRPower = power * (sin/max) + pivot;
        BLPower = power * (sin/max) - pivot;
        BRPower = power * (cos/max) + pivot;

        robot.motorFL.setPower(FLPower);
        robot.motorFR.setPower(FRPower);
        robot.motorBL.setPower(BLPower);
        robot.motorBR.setPower(BRPower);

        robot.motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void part1(double theta, double pivot, double power){
        theta = 2 * Math.PI + (theta / 360 * 2 * Math.PI) - Math.PI / 2;

        sin = Math.sin(theta -Math.PI/4);
        cos = Math.cos(theta - Math.PI/4);
        max = Math.max(Math.abs(sin),Math.abs(cos));

        FLPower = power * (cos/max) + pivot;
        FRPower = power * sin/max - pivot;
        BLPower = power * -(sin/max) - pivot;
        BRPower = power * -(cos/max) + pivot;

        robot.motorFL.setPower(-FLPower);
        robot.motorFR.setPower(-FRPower);
        robot.motorBL.setPower(BLPower);
        robot.motorBR.setPower(BRPower);

        robot.motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive (double target, double power, double pivot, double distance){

        this.theta = Math.PI + (target * Math.PI/180);
        sin = Math.sin(theta -Math.PI/4);
        cos = Math.cos(theta - Math.PI/4);
        max = Math.max(Math.abs(sin),Math.abs(cos));

        int FL = robot.motorFL.getCurrentPosition();
        int FR = robot.motorFR.getCurrentPosition();
        int BL = robot.motorBL.getCurrentPosition();
        int BR = robot.motorBR.getCurrentPosition();

        double orig = FL;
        double cur = orig;

        while (Math.abs(cur-orig) <= distance){
            FL = robot.motorFL.getCurrentPosition();
            FR = robot.motorFR.getCurrentPosition();
            BL = robot.motorBL.getCurrentPosition();
            BR = robot.motorBR.getCurrentPosition();

            cur = FL;

            FLPower = power * -(cos/max) + pivot;
            FRPower = power * sin/max + pivot;
            BLPower = power * -(sin/max) + pivot;
            BRPower = power * cos/max + pivot;

            robot.motorFL.setPower(-FLPower);
            robot.motorFR.setPower(-FRPower);
            robot.motorBL.setPower(BLPower);
            robot.motorBR.setPower(BRPower);

            robot.motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
    }
}
