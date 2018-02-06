package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Colin_Zhu on 10/29/2017
 */

public class Robot
{
    private static final double drivePower = 0.75;
    private static final double K = 0.5;
    /* Public OpMode members. */
    public DcMotor motorLeftFront = null;
    public DcMotor motorRightFront = null;
    public DcMotor motorLeftBack = null;
    public DcMotor motorRightBack = null;
    public DcMotor motorLift;
    public DcMotor relicArm;
    public Servo leftClaw;
    public Servo rightClaw;
    public Servo jewel;
    public Servo relicWrist;
    public Servo relicClaw;
    /* local OpMode members. */
    private HardwareMap hardwareMap = null;
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;

        motorLeftFront = hardwareMap.get(DcMotor.class, "leftFront");
        motorLeftBack = hardwareMap.get(DcMotor.class,"leftBack");
        motorRightFront = hardwareMap.get(DcMotor.class,"rightFront");
        motorRightBack = hardwareMap.get(DcMotor.class,"rightBack");
        motorLift = hardwareMap.get(DcMotor.class, "lift");
        relicArm = hardwareMap.get(DcMotor.class, "relicArm");

        leftClaw  = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        jewel = hardwareMap.get(Servo.class,"jewel");
        relicWrist = hardwareMap.get(Servo.class, "relicWrist");
        relicClaw = hardwareMap.get(Servo.class, "relicClaw");


        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);
        motorLift.setDirection(DcMotor.Direction.FORWARD);
        relicArm.setDirection(DcMotor.Direction.FORWARD);

        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        motorLift.setPower(0);
        relicArm.setPower(0);
        //relicWrist.setPower(0);

    }
    public void mechanumDrive(double forward, double clockwise, double right){

        double front_left = forward  + right + K*clockwise;
        double front_right = forward - right - K*clockwise;
        double rear_left = forward - right + K*clockwise;
        double rear_right = forward + right - K*clockwise;

        double max = Math.abs(front_left);
        if (Math.abs(front_right)>max) max = Math.abs(front_right);
        if (Math.abs(rear_left)>max) max = Math.abs(rear_left);
        if (Math.abs(rear_right)>max) max = Math.abs(rear_right);
        if (max>1)
        {front_left/=max; front_right/=max; rear_left/=max; rear_right/=max;}

        motorLeftFront.setPower(front_left * drivePower);
        motorLeftBack.setPower(rear_left * drivePower);
        motorRightFront.setPower(front_right * drivePower);
        motorRightBack.setPower(rear_right * drivePower);

    }
}
