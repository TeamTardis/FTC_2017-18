package org.firstinspires.ftc.teamcode;
/**
 * Created by Corning Robotics on 9/25/16.
 */
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.math.RoundingMode;
import java.text.DecimalFormat;

import static com.sun.tools.javac.util.Constants.format;
//Imports

@TeleOp(name="TestBed", group="TeleOp")
public class TestBed2018 extends OpMode {

    //
    // From Matt: PLEASE USE ToddBot instead!!
    //


    DcMotor m1; //Define dcMotor as m1
    DcMotor m2; //Define dcMotor as m2
    DcMotor m3; //Define dcMotor as m3
    DcMotor m4; //Define dcMotor as m4
    DcMotor m5; //Define dcMotor as m4

    Servo s1; //Claw grip servo
    Servo s2; //Claw rotation servo

    IntegratingGyroscope gyro; //Gyro
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    I2cDeviceSynch r1reader; //Right range sensor
    ModernRoboticsI2cRangeSensor r1;

    I2cDeviceSynch r2reader; //Front range sensor
    ModernRoboticsI2cRangeSensor r2;

    public void init() { //Start of the initiation for autonomous

        m1 = hardwareMap.dcMotor.get("m1"); //Sets m1 to m3 in the config
        m2 = hardwareMap.dcMotor.get("m2"); //Sets m2 to m1 in the config
        m3 = hardwareMap.dcMotor.get("m3"); //Sets m3 to m4 in the config
        m4 = hardwareMap.dcMotor.get("m4"); //Sets m4 to m2 in the config
        m5 = hardwareMap.dcMotor.get("m5"); //Sets m4 to m2 in the config

        s1 = hardwareMap.servo.get("s1"); //Sets s1 i the config
        s2 = hardwareMap.servo.get("s2"); //Sets s1 i the config

        m2.setDirection(DcMotor.Direction.REVERSE);
        m4.setDirection(DcMotor.Direction.REVERSE);

        s1.setPosition(0);
        s2.setPosition(1);
        r1reader = hardwareMap.i2cDeviceSynch.get("r1"); //Port 1 (Right side)
        r1 = new ModernRoboticsI2cRangeSensor(r1reader);
        r1.setI2cAddress(I2cAddr.create8bit(0x2a));

        r2reader = hardwareMap.i2cDeviceSynch.get("r2"); //Port 2 (Front side)
        r2 = new ModernRoboticsI2cRangeSensor(r2reader);
        r2.setI2cAddress(I2cAddr.create8bit(0x2c));

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;

        modernRoboticsI2cGyro.calibrate(); //Gyro calibration

        while (modernRoboticsI2cGyro.isCalibrating()) { //Adds telemetry for gyro calibration
            telemetry.addData("", "Gyro Calibrating. Please wait..."); //Adds telemetry
            telemetry.update(); //Updates telemetry
        }

        telemetry.addData("", "Gyro Calibrated. Initializing Vuforia..."); //Adds telemetry
        telemetry.update(); //Updates telemetry

        double rCM1Prev;
        double rCM2Prev;

    } //Ends initiation

    double turn = 0;
    double power;
    float encoderCurrent;
    double encoderDifference;
    float encoderTarget = 0;
    double wristRotation = 0;
    int gyroTarget = 0;
//
//    double rangeCM1 = r1.getDistance(DistanceUnit.CM); //Initializes rangeCM1 for range reading
//    double rangeCM2 = r2.getDistance(DistanceUnit.CM); //Initializes rangeCM2 for range reading
//
//    double rCM1Prev = r1.getDistance(DistanceUnit.CM); //Defining variable used in low pass filter
//    if (rCM1Prev > 255 || rCM1Prev < 0) { //Error check
//        rCM1Prev = 0;
//    }
//    double rCM2Prev = r2.getDistance(DistanceUnit.CM); //Defining variable used in low pass filter
//    if (rCM2Prev > 255 || rCM2Prev < 0) { //Error check
//        rCM2Prev = 0;
//    }

//    double rCM1Curr = 0; //Initializes variable to track current range 1 reading
//    double rCM2Curr = 0; //Initializes variable to track current range 2 reading
    @Override
    public void loop() {

//
//        rCM1Curr = r1.getDistance(DistanceUnit.CM); //Defining variable used in low pass filter
//        if (rCM1Curr > 255 || rCM1Curr < 0) {
//            rCM1Curr = rCM1Prev;
//        }
//        rCM2Curr = r2.getDistance(DistanceUnit.CM); //Defining variable used in low pass filter
//        if (rCM2Curr > 255 || rCM2Curr < 0) {
//            rCM2Curr = rCM2Prev;
//        }

//        rangeCM1 = r1.getDistance(DistanceUnit.CM); //Updates rangeCM1 variable with low pass filter
//        rCM1Prev = rangeCM1; //Updates rCM1Prev variable with info current rangeCM1 variable
//        rangeCM2 = r2.getDistance(DistanceUnit.CM); //Updates rangeCM1 variable with low pass filter
//        rCM2Prev = rangeCM2; //Updates rCM2Prev variable with info current rangeCM1 variable
        encoderCurrent = m5.getCurrentPosition();
        /*
        double currentPosition = s2.getPosition();

        if(modernRoboticsI2cGyro.getIntegratedZValue() > gyroTarget + 3 && currentPosition < 1) {
            currentPosition += .007;
        }
        if(modernRoboticsI2cGyro.getIntegratedZValue() < gyroTarget - 3 && currentPosition > 0) {
            currentPosition -= .007;
        }

        if(gamepad2.y && Math.abs(gyroTarget - modernRoboticsI2cGyro.getIntegratedZValue()) < 100) {
            gyroTarget++;
        }
        if(gamepad2.x && Math.abs(gyroTarget - modernRoboticsI2cGyro.getIntegratedZValue()) < 100) {
            gyroTarget--;
        }

        s2.setPosition(currentPosition);
        */

        telemetry.addData("Matt says use ToddBot instead!\nRange1: ",  r1.getDistance(DistanceUnit.CM) + "\nRange2: " + r2.getDistance(DistanceUnit.CM)); //Adds telemetry to debug
        telemetry.update(); //Updates telemetry with new informatio
        float LUD = gamepad1.left_stick_y; //Variable for left stick y axis
        float LRL = gamepad1.left_stick_x; //Variable for left stick x axis
        float RUD = gamepad1.right_stick_y; //Variable for right stick y axis
        float RLR = -gamepad1.right_stick_x; //Variable for right stick x axis

        if (gamepad1.left_trigger != 0) {
            s1.setPosition(0); //Set servo position to 0.8
        } else {
            s1.setPosition(1); //Set servo position to 1
        }

        if ((gamepad2.left_stick_y < -.1 || gamepad2.left_stick_y > .1)) {
            m5.setPower(gamepad2.left_stick_y / 2);
        }  else {
            m5.setPower(0);
        }

        if (gamepad1.right_trigger == 0) { //Controls for slow mode

            m1.setPower(((LRL + LUD) / 6) + (RLR / 4) - turn); //Turning for top left
            m2.setPower(((LUD - LRL) / 6) - (RLR / 4) + turn); //Steering for top right
            m3.setPower(((LUD - LRL) / 6) + (RLR / 4) - turn); //Steering for back left
            m4.setPower(((LRL + LUD) / 6) - (RLR / 4) + turn); //Steering for back right

        } else { //Controls for fast mode

            m1.setPower(((LRL + LUD) / 2) + RLR - turn); //Steering for top left
            m2.setPower(((LUD - LRL) / 2) - RLR + turn); //Steering for top right
            m3.setPower(((LUD - LRL) / 2) + RLR - turn); //Steering for back left
            m4.setPower(((LRL + LUD) / 2) - RLR + turn); //Steering for back right

        }
    }
}