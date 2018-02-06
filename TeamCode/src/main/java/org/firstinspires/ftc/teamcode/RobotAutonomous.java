package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Colin_Zhu on 10/29/2017
 */

public class RobotAutonomous
{
    private ElapsedTime runtime = new ElapsedTime();
    private static final double     COUNTS_PER_MOTOR_REV    = 560 ;    // Andymark
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    private static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    private static final double     P_TURN_COEFF            = 0.05;     // Larger is more responsive, but also less stable
    private static final double     P_DRIVE_COEFF           = 0.10;     // Larger is more responsive, but also less stable
    private static final double     DRIVE_SPEED             = 0.2;
    private static final double     TURN_SPEED              = 0.1;
    private static final double JEWEL_POSITION = 0.95;
    private static final double UP_RIGHT = 0.5;
    private static final double JEWEL_TURN = 3;
    private static final boolean bLedOn = true;
    private boolean jewelDone = true;
    /* Public OpMode members. */
    public DcMotor motorLeftFront = null;
    public DcMotor motorRightFront = null;
    public DcMotor motorLeftBack = null;
    public DcMotor motorRightBack = null;
    public DcMotor motorLift = null;
    public Servo leftClaw;
    public Servo rightClaw;
    public Servo jewel;
    public ModernRoboticsI2cColorSensor colorSensor = null;
    public ModernRoboticsI2cGyro gyro = null;
    /* local OpMode members. */
    private HardwareMap hardwareMap = null;
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;

        motorLeftFront = hardwareMap.get(DcMotor.class, "leftFront");
        motorLeftBack = hardwareMap.get(DcMotor.class, "leftBack");
        motorRightFront = hardwareMap.get(DcMotor.class, "rightFront");
        motorRightBack = hardwareMap.get(DcMotor.class, "rightBack");
        motorLift = hardwareMap.get(DcMotor.class, "lift");

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        jewel = hardwareMap.get(Servo.class, "jewel");

        colorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "colorSensor");
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);

        motorLift.setDirection(DcMotor.Direction.FORWARD);

        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        motorLift.setPower(0);

        colorSensor.enableLed(bLedOn);


        resetEncoders();

        gyro.resetZAxisIntegrator();
        gyro.calibrate();
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS){

            // Determine new target position, and pass to motor controller
            motorLeftFront.setTargetPosition(motorLeftFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH));
            motorLeftBack.setTargetPosition(motorLeftBack.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH));
            motorRightFront.setTargetPosition(motorRightFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH));
            motorRightBack.setTargetPosition(motorRightBack.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH));


            // Turn On RUN_TO_POSITION
            motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            motorLeftFront.setPower(Math.abs(speed));
            motorLeftBack.setPower(Math.abs(speed));
            motorRightFront.setPower(Math.abs(speed));
            motorRightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((runtime.seconds() < timeoutS) &&
                    (motorLeftFront.isBusy() && motorLeftBack.isBusy() && motorRightFront.isBusy() && motorRightBack.isBusy())) {
            }

            // Stop all motion;
            motorLeftFront.setPower(0);
            motorLeftBack.setPower(0);
            motorRightFront.setPower(0);
            motorRightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            jewel.setPosition(1);
    }
    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;


        // Determine new target position, and pass to motor controller
        moveCounts = (int)(distance * COUNTS_PER_INCH);

        // Set Target and Turn On RUN_TO_POSITION
        motorLeftFront.setTargetPosition(motorLeftFront.getCurrentPosition() + moveCounts);
        motorLeftBack.setTargetPosition(motorLeftBack.getCurrentPosition() + moveCounts);
        motorRightFront.setTargetPosition( motorRightFront.getCurrentPosition() + moveCounts);
        motorRightBack.setTargetPosition( motorRightBack.getCurrentPosition() + moveCounts);

        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        motorLeftFront.setPower(speed);
        motorLeftBack.setPower(speed);
        motorRightFront.setPower(speed);
        motorRightBack.setPower(speed);

        // keep looping while we are still active, and BOTH motors are running.
        while (motorLeftFront.isBusy() && motorLeftBack.isBusy() && motorRightFront.isBusy() && motorRightBack.isBusy()) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                motorLeftFront.setPower(leftSpeed);
                motorLeftBack.setPower(leftSpeed);
                motorRightFront.setPower(rightSpeed);
                motorRightBack.setPower(rightSpeed);
            }

        // Stop all motion;
        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);

        // Turn off RUN_TO_POSITION
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (!onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
        }
    }
    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    private boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        motorLeftFront.setPower(leftSpeed);
        motorLeftBack.setPower(leftSpeed);
        motorRightFront.setPower(rightSpeed);
        motorRightBack.setPower(rightSpeed);


        return onTarget;
    }
    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    public void resetEncoders(){
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void jewelRed(){
        jewel.setPosition(JEWEL_POSITION);
        leftClaw.setPosition(1);
        rightClaw.setPosition(0);
        while(jewelDone) {

            if (colorSensor.red() > colorSensor.blue()) {
                encoderDrive(TURN_SPEED, JEWEL_TURN, -JEWEL_TURN, 1);
                encoderDrive(TURN_SPEED, -JEWEL_TURN, JEWEL_TURN, 1);
                jewelDone = false;
            } else if (colorSensor.red() < colorSensor.blue()) {
                encoderDrive(TURN_SPEED, -JEWEL_TURN, JEWEL_TURN, 1);
                encoderDrive(TURN_SPEED, JEWEL_TURN, -JEWEL_TURN, 1);
                jewelDone = false;
            }
        }
        jewel.setPosition(UP_RIGHT);
    }
    public void jewelBlue(){
        jewel.setPosition(JEWEL_POSITION);
        leftClaw.setPosition(1);
        rightClaw.setPosition(0);
        while(jewelDone) {
            if (colorSensor.blue() > colorSensor.red()) {
                encoderDrive(TURN_SPEED, JEWEL_TURN, -JEWEL_TURN, 1);
                encoderDrive(TURN_SPEED, -JEWEL_TURN, JEWEL_TURN, 1);
                jewelDone = false;
            } else if (colorSensor.blue() < colorSensor.red()) {
                encoderDrive(TURN_SPEED, -JEWEL_TURN, JEWEL_TURN, 1);
                encoderDrive(TURN_SPEED, JEWEL_TURN, -JEWEL_TURN, 1);
                jewelDone = false;
            }
        }
        jewel.setPosition(UP_RIGHT);
    }
}
