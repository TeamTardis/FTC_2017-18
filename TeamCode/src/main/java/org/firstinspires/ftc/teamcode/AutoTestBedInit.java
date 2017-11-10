package org.firstinspires.ftc.teamcode; //Use the package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor; //Import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor; //Import com.qualcomm.robotcore.hardware.ColorSensor for the color sensors
import com.qualcomm.robotcore.hardware.DcMotor; //Import com.qualcomm.robotcore.hardware.DcMotor for motors
import com.qualcomm.robotcore.hardware.GyroSensor; //Import com.qualcomm.robotcore.hardware.GyroSensor for the gyro sensor
import com.qualcomm.robotcore.hardware.I2cAddr; //Import com.qualcomm.robotcore.hardware.I2cAddr to allow to change I2c addresses
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor; //Import com.qualcomm.robotcore.hardware.OpticalDistanceSensor for the optical distance sensor
import com.qualcomm.robotcore.hardware.Servo; //Import com.qualcomm.robotcore.hardware.Servo for servos
import com.qualcomm.robotcore.hardware.TouchSensor; //Import com.qualcomm.robotcore.hardware.TouchSensor for touch sensors
import com.qualcomm.robotcore.eventloop.opmode.Autonomous; //Imports com.qualcomm.robotcore.eventloop.opmode.Autonomous for autonomous additions
import com.qualcomm.robotcore.eventloop.opmode.OpMode; //Imports com.qualcomm.robotcore.eventloop.opmode.OpMode for opmode additions
import com.qualcomm.robotcore.util.ElapsedTime; //Imports com.qualcomm.robotcore.util.ElapsedTime for timed events
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit; //Imports org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

/**
 * Created by Corning Robotics on 12/21/16. *
 **/

public abstract class AutoTestBedInit extends OpMode { //Imports presets for initiation from OpMode
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

    public static final String TAG = "AutoTestBedInit";

    @Override //Method overrides parent class
    public void init() { //Start of the initiation for autonomous

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
        r1.setI2cAddress(I2cAddr.create8bit(0x2a));

        r2reader = hardwareMap.i2cDeviceSynch.get("r2");
        r2 = new ModernRoboticsI2cRangeSensor(r2reader);
        r2.setI2cAddress(I2cAddr.create8bit(0x28));

    } //Ends initiation
} //End of program

