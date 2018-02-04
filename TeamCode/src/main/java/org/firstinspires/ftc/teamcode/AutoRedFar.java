package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
Created By Colin Zhu
 */

@Autonomous(name="Red Far", group ="Red")
public class AutoRedFar extends LinearOpMode {

    private RobotAutonomous robot = new RobotAutonomous();

    private static final double     DRIVE_SPEED             = 0.2;
    private static final double     TURN_SPEED              = 0.1;

    private static final double OFF_BOARD = 18;
    private static final double LEFT_COLUMN = 24;
    private static final double CENTER_COLUMN = 16;
    private static final double RIGHT_COLUMN = 9;
    private static final double CRYPTOBOX_DISTANCE = 8;

    private boolean isDone = true;
    private VuforiaLocalizer vuforia;

    private ElapsedTime     runtime = new ElapsedTime();

    @Override public void runOpMode() {
        robot.init(hardwareMap);



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ATYxHnb/////AAAAGR2068HfR0szsfy8CfwNdH5MMFM3VFio5e8jf5P+hxM+6Vr0b0c3w1Glwuiw62sQdGzmd75H1osvxnOH53LlsaWOF0Y+F0zBteIa5fF78ozsQvDr/jeN9eUib5hbfRVXXExokFpa05xY1MlHb09i/PFv5gc84E0benqgASpyPjyciyMedwEFlO04AKUXfpSX9YupIbzSDHJyaFWuzbfzRE4ezsQfmVvIfCGabe2N/RWpaIxY3UR3Mhx+qP4tUSVUiVn2Be4oHGSdGCktlx6Xe0yCcdargDXYvM66SovpUhjeudwwsIY3pd0Ld56xao5h6GXArWwT06Nset/fsQ0QEtg/XsYN0xhPl35YaTFjZtcE\n";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        telemetry.addData(">", "Calibrating Gyro");
        telemetry.update();

        robot.gyro.calibrate();

        while (!isStopRequested() && robot.gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        runtime.reset();
        telemetry.addData("Blue",robot.colorSensor.blue());
        telemetry.addData("Red",robot.colorSensor.red());
        telemetry.addData(">", "Press Play to start");
        telemetry.addData(">", "Robot Heading = %d", robot.gyro.getIntegratedZValue());
        telemetry.update();

        waitForStart();

        relicTrackables.activate();

        robot.jewelRed();
        waitTime(1);
        while(isDone && opModeIsActive()){
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);
                telemetry.update();
                switch(vuMark){
                    case LEFT:
                        telemetry.addData("VuMark", "%s visible", vuMark);
                        telemetry.update();
                        robot.encoderDrive(DRIVE_SPEED,OFF_BOARD,OFF_BOARD,5);
                        waitTime(1);
                        robot.gyroTurn(TURN_SPEED,90);
                        waitTime(1);
                        robot.encoderDrive(DRIVE_SPEED,LEFT_COLUMN,LEFT_COLUMN,5);
                        waitTime(1);
                        robot.gyroTurn(TURN_SPEED,0);
                        isDone = false;
                        break;
                    case CENTER:
                        telemetry.addData("VuMark", "%s visible", vuMark);
                        telemetry.update();
                        robot.encoderDrive(DRIVE_SPEED,OFF_BOARD,OFF_BOARD,5);
                        waitTime(1);
                        robot.gyroTurn(TURN_SPEED,90);
                        waitTime(1);
                        robot.encoderDrive(DRIVE_SPEED,CENTER_COLUMN,CENTER_COLUMN,5);
                        waitTime(1);
                        robot.gyroTurn(TURN_SPEED,0);
                        isDone = false;
                        break;
                    case RIGHT:
                        telemetry.addData("VuMark", "%s visible", vuMark);
                        telemetry.update();
                        robot.encoderDrive(DRIVE_SPEED,OFF_BOARD,OFF_BOARD,5);
                        waitTime(1);
                        robot.gyroTurn(TURN_SPEED,90);
                        waitTime(1);
                        robot.encoderDrive(DRIVE_SPEED,RIGHT_COLUMN,RIGHT_COLUMN,5);
                        waitTime(1);
                        robot.gyroTurn(TURN_SPEED,0);
                        isDone = false;
                        break;
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }
            telemetry.update();
            idle();
        }

        waitTime(1);
        robot.encoderDrive(DRIVE_SPEED,CRYPTOBOX_DISTANCE,CRYPTOBOX_DISTANCE,2);
        waitTime(1);
        robot.leftClaw.setPosition(0);
        robot.rightClaw.setPosition(1);
        waitTime(1);
        robot.encoderDrive(DRIVE_SPEED,-4,-4,2);

    }
    public void waitTime(double time){
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
}
