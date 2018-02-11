package org.firstinspires.ftc.teamcode.opmode;

import android.util.TypedValue;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.base.Color;
import org.firstinspires.ftc.teamcode.control.Mecanum;

import java.util.ArrayList;

/**
 * Hardware Abstraction Layer for Robot.
 * Provides common variables and functions for the hardware.
 * The robot configuration in the app should match enum names.
 * Per-robot customization configured in SharedCode/src/main/res/values/.
 */
public abstract class RobotHardware extends OpMode {
    // Get constant from resource file.
    int getResourceInt(int id) {
        return hardwareMap.appContext.getResources().getInteger(id);
    }

    // Get constant from resource file.
    double getResourceDouble(int id) {
        TypedValue outValue = new TypedValue();
        hardwareMap.appContext.getResources().getValue(id, outValue, true);
        return outValue.getFloat();
    }

    // The motors on the robot.
    protected enum MotorName {
        DRIVE_FRONT_LEFT,
        DRIVE_FRONT_RIGHT,
        DRIVE_BACK_LEFT,
        DRIVE_BACK_RIGHT,
    }

    /**
     * Sets the power of the motor.
     * @param motor The motor to modify.
     * @param power The power to set [-1, 1].
     */
    protected void setPower(MotorName motor, double power) {
        DcMotor m = allMotors.get(motor.ordinal());
        if (m == null) {
            telemetry.addData("Motor Missing", motor.name());
        } else {
            m.setPower(power);
        }
    }

    /**
     * Sets the drive chain power.
     * @param left The power for the left two motors.
     * @param right The power for the right two motors.
     */
    protected void setDriveForTank(double left, double right) {
        setPower(MotorName.DRIVE_FRONT_LEFT, left);
        setPower(MotorName.DRIVE_BACK_LEFT, left);
        setPower(MotorName.DRIVE_FRONT_RIGHT, right);
        setPower(MotorName.DRIVE_BACK_RIGHT, right);
    }

    /**
     * Sets the drive chain power from Mecanum motion.
     * Maintains relative speeds when changing angles.
     * @param motion The desired Mecanum motion.
     */
    protected void setDriveForMecanum(Mecanum.Motion motion) {
        Mecanum.Wheels wheels = Mecanum.motionToWheels(motion);
        setPower(MotorName.DRIVE_FRONT_LEFT, wheels.frontLeft);
        setPower(MotorName.DRIVE_FRONT_RIGHT, wheels.frontRight);
        setPower(MotorName.DRIVE_BACK_LEFT, wheels.backLeft);
        setPower(MotorName.DRIVE_BACK_RIGHT, wheels.backRight);
    }

    /**
     * Sets the drive chain power from Mecanum motion.
     * Uses max power output while changing speeds at angle motions.
     * @param motion The desired Mecanum motion.
     */
    protected void setDriveForMecanumForSpeed(Mecanum.Motion motion) {
        Mecanum.Wheels wheels = Mecanum.motionToWheels(motion).scaleWheelPower(
                Math.sqrt(2));
        setPower(MotorName.DRIVE_FRONT_LEFT, wheels.frontLeft);
        setPower(MotorName.DRIVE_FRONT_RIGHT, wheels.frontRight);
        setPower(MotorName.DRIVE_BACK_LEFT, wheels.backLeft);
        setPower(MotorName.DRIVE_BACK_RIGHT, wheels.backRight);
    }

    // The servos on the robot.
    protected enum ServoName {
        JEWEL_DROP,
        JEWEL_HIT,
    }

    /**
     * Sets the angle of the servo.
     * @param servo The servo to modify.
     * @param position The angle to set [0, 1].
     */
    protected void setAngle(ServoName servo, double position) {
        Servo s = allServos.get(servo.ordinal());
        if (s == null) {
            telemetry.addData("Servo Missing", servo.name());
        } else {
            s.setPosition(position);
        }
    }

    // Raises the jewel arm.
    protected void raiseJewelArm() {
        setAngle(ServoName.JEWEL_DROP, raisedJewelAngle);
    }

    // Lowers the jewel arm.
    protected void lowerJewelArm() {
        setAngle(ServoName.JEWEL_DROP, loweredJewelAngle);
    }

    // Centers the jewel arm.
    protected void centerJewelArm() {
        setAngle(ServoName.JEWEL_HIT, centerJewelAngle);
    }

    // Moves the jewel arm forward.
    protected void forwardJewelArm() {
        setAngle(ServoName.JEWEL_HIT, forwardJewelAngle);
    }

    // Moves the jewel arm backward.
    protected void backwardJewelArm() {
        setAngle(ServoName.JEWEL_HIT, backwardJewelAngle);
    }

    // The color sensors on the robot.
    protected enum ColorSensorName {
        JEWEL,
    }

    /**
     * Gets the color value on the sensor.
     * @param sensor The sensor to read.
     * @param color The color channel to read intensity.
     */
    protected int getColorSensor(ColorSensorName sensor, Color.Channel color) {
        ColorSensor s = allColorSensors.get(sensor.ordinal());
        if (s == null) {
            telemetry.addData("Color Sensor Missing", sensor.name());
            return 0;
        }

        switch (color) {
            case RED: return s.red();
            case GREEN: return s.green();
            case BLUE: return s.blue();
            case ALPHA: return s.alpha();
            default: return 0;
        }
    }

    /**
     * Sets the LED power for the color sensor.
     * @param sensor The sensor to set the LED power.
     * @param enabled Whether to turn the LED on.
     */
    protected void setColorSensorLedEnabled(ColorSensorName sensor,
                                         boolean enabled) {
        ColorSensor s = allColorSensors.get(sensor.ordinal());
        if (s == null) {
            telemetry.addData("Color Sensor Missing", sensor.name());
        } else {
            s.enableLed(enabled);
        }
    }

    // Possible starting positions.
    protected enum StartPosition {
        FIELD_CENTER,
        FIELD_CORNER,
    }

    // Returns a string representation of the starting position.
    String getStartPositionName(Color.Ftc c, StartPosition p) {
        return c.name() + "-" + p.name();
    }

    /**
     * Initialize the hardware handles.
     */
    public void init() {
        raisedJewelAngle = getResourceInt(
                R.integer.raised_jewel_angle_percent) / 100.0;
        loweredJewelAngle = getResourceInt(
                R.integer.lowered_jewel_angle_percent) / 100.0;
        centerJewelAngle = getResourceInt(
                R.integer.center_jewel_angle_percent) / 100.0;
        forwardJewelAngle = getResourceInt(
                R.integer.forward_jewel_angle_percent) / 100.0;
        backwardJewelAngle = getResourceInt(
                R.integer.backward_jewel_angle_percent) / 100.0;

        allMotors = new ArrayList<DcMotor>();
        for (MotorName m : MotorName.values()) {
            try {
                allMotors.add(hardwareMap.get(DcMotor.class, m.name()));
            } catch (Exception e) {
                telemetry.addData("Motor Missing", m.name());
                allMotors.add(null);
            }
        }

        allServos = new ArrayList<Servo>();
        for (ServoName s : ServoName.values()) {
            try {
                allServos.add(hardwareMap.get(Servo.class, s.name()));
            } catch (Exception e) {
                telemetry.addData("Servo Missing", s.name());
                allServos.add(null);
            }
        }

        allColorSensors = new ArrayList<ColorSensor>();
        for (ColorSensorName s : ColorSensorName.values()) {
            try {
                allColorSensors.add(hardwareMap.get(ColorSensor.class,
                                                    s.name()));
            } catch (Exception e) {
                telemetry.addData("Color Sensor Missing", s.name());
                allColorSensors.add(null);
            }
        }

        raiseJewelArm();
        centerJewelArm();
    }

    public void loop() {}

    /**
     * End of match, stop all actuators.
     */
    public void stop() {
        super.stop();

        for (MotorName m : MotorName.values()) {
            setPower(m, 0);
        }
        for (ColorSensorName s : ColorSensorName.values()) {
            setColorSensorLedEnabled(s, false);
        }
    }

    // All motors on the robot, in order of MotorName.
    private ArrayList<DcMotor> allMotors;
    // All servos on the robot, in order of ServoName.
    private ArrayList<Servo> allServos;
    // All color sensors on the robot, in order of ColorSensorName.
    private ArrayList<ColorSensor> allColorSensors;

    // Per robot tuning parameters.
    private double raisedJewelAngle;
    private double loweredJewelAngle;
    private double centerJewelAngle;
    private double forwardJewelAngle;
    private double backwardJewelAngle;
}
