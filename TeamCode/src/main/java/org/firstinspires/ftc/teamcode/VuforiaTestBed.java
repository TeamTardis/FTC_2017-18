package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//Imports

@Autonomous(name="VuforiaTestBed", group ="Teleop")
@Disabled

public class VuforiaTestBed extends LinearOpMode {

    public static final String TAG = "VuforiaTestBed";

    VuforiaLocalizer vuforia;

//////////////////////////////////////////////////////

    DcMotor m1; //Define dcMotor as m1
    DcMotor m2; //Define dcMotor as m2
    DcMotor m3; //Define dcMotor as m3
    DcMotor m4; //Define dcMotor as m4
    DcMotor m5; //Define dcMotor as m5

    Servo s1; //Define servo
    Servo s2;
    //Servo s3; //Define servo as s1

    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    ColorSensor c1;

    I2cDeviceSynch r2reader;
    ModernRoboticsI2cRangeSensor r2;

    I2cDeviceSynch r1reader;
    ModernRoboticsI2cRangeSensor r1;

    /////////////////////////////////////////////////////////

    @Override
    public void runOpMode() {

/////////////////////////////////////////////////////////////

        m1 = hardwareMap.dcMotor.get("m1"); //Sets m1 in the config
        m2 = hardwareMap.dcMotor.get("m2"); //Sets m2 in the config
        m3 = hardwareMap.dcMotor.get("m3"); //Sets m3 in the config
        m4 = hardwareMap.dcMotor.get("m4"); //Sets m4 in the config
        m5 = hardwareMap.dcMotor.get("m5"); //Sets m4 in the config

        s1 = hardwareMap.servo.get("s1"); //Sets s1 i the config
        s2 = hardwareMap.servo.get("s2"); //Sets s1 i the config
        //s3 = hardwareMap.servo.get("s3"); //Sets s1 i the config

        m2.setDirection(DcMotor.Direction.REVERSE);
        m4.setDirection(DcMotor.Direction.REVERSE);

        s1.setPosition(0);
        s2.setPosition(0);

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;

        c1 = hardwareMap.colorSensor.get("c1"); //Sets colorSensor to c1 in the config
        c1.enableLed(true); //Turns Color Sensor LED off

        modernRoboticsI2cGyro.calibrate();

        while(modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("", "Gyro Calibrating. Please wait...");
            telemetry.update();
        }
        telemetry.addData("", "Gyro Calibrated. Good luck!");
        telemetry.update();

        r1reader = hardwareMap.i2cDeviceSynch.get("r1");
        r1 = new ModernRoboticsI2cRangeSensor(r1reader);
//          r1 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "r1");
          r1.setI2cAddress(I2cAddr.create8bit(0x2a));

        r2reader = hardwareMap.i2cDeviceSynch.get("r2");
        r2 = new ModernRoboticsI2cRangeSensor(r2reader);
        r2.setI2cAddress(I2cAddr.create8bit(0x28));


//////////////////////////////////////////////////////////

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                "id", hardwareMap.appContext.getPackageName()); //Shows camera on robot controller phone

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId); //Shows camera on robot controller phone

        // Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "Aa9zuMz/////AAAAGTZYlyCroUG9ibBucmCtqD990SI/3cked3Q7Nnj4zhBoD72GU" +
                "WNlfPopl2pvrC3tFmeHYt/I1Rtxubiif5SptIs9SdCheiJZBLEIAOG4nizHWuIB5tU2yCjJoO7pNrn/+gn4y2LDg" +
                "bIQw7AVdp272CbVsp6E/e6zq7fA0Yelv6JN/nLROUo7FWJHz8G98/XL9d1poW9D5DlJ++j7ePbgPv+kSPqIhfiQ8" +
                "uEgtCTLUctdu2/n4M3J/fltvRe4ykRG1UczLleQJ5NMflog4rloogbngGNRTVg0DRV0YEQkbnJIcfBz9kDja0Miq" +
                "vcc6lwPfyBIo591XYi0hU7asnQ2boyWWQGXEdLMnxHQcW5YVmbG"; //License key

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        while (opModeIsActive()) {


            double rangeCM1 = r1.getDistance(DistanceUnit.CM);
            double rangeCM2 = r2.getDistance(DistanceUnit.CM);
            int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue(); //Gyro integratedZ value

            telemetry.addData("\nColorBlue: ",  c1.blue()
                    + "\nColorRed: " + c1.red() + "\nRange1: " + rangeCM1 + "\nRange2: " + rangeCM2); //Adds telemetry to debug
            telemetry.update(); //Updates telemetry with new information

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
 /*           if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                telemetry.addData("VuMark", "%s visible", vuMark);
            } else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
*/        }
    }
}

