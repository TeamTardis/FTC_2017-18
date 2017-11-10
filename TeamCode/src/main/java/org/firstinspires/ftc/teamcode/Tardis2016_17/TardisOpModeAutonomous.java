package org.firstinspires.ftc.teamcode; //Use the package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor; //Import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor; //Import com.qualcomm.robotcore.hardware.ColorSensor for the color sensors
import com.qualcomm.robotcore.hardware.DcMotor; //Import com.qualcomm.robotcore.hardware.DcMotor for motors
import com.qualcomm.robotcore.hardware.GyroSensor; //Import com.qualcomm.robotcore.hardware.GyroSensor for the gyro sensor
import com.qualcomm.robotcore.hardware.I2cAddr; //Import com.qualcomm.robotcore.hardware.I2cAddr to allow to change I2c addresses
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor; //Import com.qualcomm.robotcore.hardware.OpticalDistanceSensor for the optical distance sensor
import com.qualcomm.robotcore.hardware.Servo; //Import com.qualcomm.robotcore.hardware.Servo for servos
import com.qualcomm.robotcore.hardware.TouchSensor; //Import com.qualcomm.robotcore.hardware.TouchSensor for touch sensors
import com.qualcomm.robotcore.eventloop.opmode.Autonomous; //Imports com.qualcomm.robotcore.eventloop.opmode.Autonomous for autonomous additions
import com.qualcomm.robotcore.eventloop.opmode.OpMode; //Imports com.qualcomm.robotcore.eventloop.opmode.OpMode for opmode additions
import com.qualcomm.robotcore.util.ElapsedTime; //Imports com.qualcomm.robotcore.util.ElapsedTime for timed events
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit; //Imports org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

/**
 * Created by Corning Robotics on 12/21/16. *
 **/

public abstract class TardisOpModeAutonomous extends OpMode { //Imports presets for initiation from OpMode
    DcMotor m1; //Define dcMotor as m1
    DcMotor m2; //Define dcMotor as m2
    DcMotor m3; //Define dcMotor as m3
    DcMotor m4; //Define dcMotor as m4
    DcMotor m5; //Define dcMotor as m5
    DcMotor m6; //Define dcMotor as m6
    DcMotor m7; //Define dcMotor as m7
    DcMotor m8; //Define dcMotor as m8
    Servo s1; //Define servo as s1 (linear actuator)
    Servo s2; //Define servo as s2 (launcher servo)
    Servo s3; //Define servo as s3 (mast forks)
    Servo SFront; //Define servo as SFront (servos to open ball collector-extender)
    TouchSensor touchSensor1; //Define touch sensor as touchSensor1 (mast limit switch)
    TouchSensor touchSensor2; //Define touch sensor as touchSensor2 (mast limit switch)
    ColorSensor colorSensor; //Define color sensor as colorSensor
    ColorSensor colorSensor2; //Define color sensor as colorSensor2
    OpticalDistanceSensor odsSensor1; //Define optical distance sensor as odsSensor1
    OpticalDistanceSensor odsSensor2; //Define optical distance sensor as odsSensor2
    ModernRoboticsI2cGyro gyro;; //Define gyro sensor as gyro
    ModernRoboticsI2cRangeSensor range; //Define range sensor as range

    @Override //Method overrides parent class
    public void init() { //Start of the initiation for autonomous
        m1 = hardwareMap.dcMotor.get("m1"); //Sets m1 to m3 in the config
        m2 = hardwareMap.dcMotor.get("m2"); //Sets m2 to m1 in the config
        m3 = hardwareMap.dcMotor.get("m3"); //Sets m3 to m4 in the config
        m4 = hardwareMap.dcMotor.get("m4"); //Sets m4 to m2 in the config
        m5 = hardwareMap.dcMotor.get("m5"); //Sets m5 to m5 in the config
        m6 = hardwareMap.dcMotor.get("m6"); //Sets m6 to m6 in the config
        m7 = hardwareMap.dcMotor.get("m7"); //Sets m7 to m7 in the config
        m8 = hardwareMap.dcMotor.get("m8"); //Sets m8 to m8 in the config
        m1.setDirection(DcMotor.Direction.REVERSE); //Reverses direction of m1
        m3.setDirection(DcMotor.Direction.REVERSE); //Reverses direction of m3

        m5.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Sets mode of m5 to run with encoder
        m6.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Sets mode of m6 to run with encoder
        m5.setDirection(DcMotor.Direction.REVERSE);  //Reverses direction of m5

        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range"); //Sets range to range in the config

        s1 = hardwareMap.servo.get("s1"); //Sets s1 to s1 in the config
        s2 = hardwareMap.servo.get("s2"); //Sets s2 to s2 in the config
        s3 = hardwareMap.servo.get("s3"); //Sets s3 to s3 in the config
        SFront = hardwareMap.servo.get("SFront"); //Sets SFront to SFront in the config

        odsSensor1 = hardwareMap.opticalDistanceSensor.get("ods1"); //Sets odsSensor1 to ods1 in the config
        I2cAddr ods1 = I2cAddr.create8bit(0x61); //Changes I2c Address to 0x61

        odsSensor2 = hardwareMap.opticalDistanceSensor.get("ods2"); //Sets odsSensor2 to ods2 in the config
        I2cAddr ods2 = I2cAddr.create8bit(0x62); //Changes I2c Address to 0x62

        colorSensor = hardwareMap.colorSensor.get("c1"); //Sets colorSensor to c1 in the config
        colorSensor.enableLed(false); //Turns Color Sensor LED off

//        m5.setMaxSpeed(1600);  //Set max speed medium for shooter
//        m6.setMaxSpeed(1600);

        touchSensor1 = hardwareMap.touchSensor.get("t1"); //Sets touchSensor1 to t1 in the config
        I2cAddr t1 = I2cAddr.create8bit(0x60); //Changes I2c Address to 0x60

        touchSensor2 = hardwareMap.touchSensor.get("t2"); //Sets touchSensor2 to t2 in the config
        I2cAddr t2 = I2cAddr.create8bit(0x69); //Changes I2c Address to 0x69

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro"); //Sets gyro to gyro in the config

        s1.setPosition(0.5); //Raises the linear actuator
        s2.setPosition(.05); //Launching servo to lowest position
        s3.setPosition(0.45); //Fits the mast forks inside the 18 by 18 by 18 inch square

        gyro.calibrate(); //Calibrate the gyro sensor
        while (gyro.isCalibrating()) { //Adds telemetry for gyro calibration
            telemetry.addData("<", "Gyro calibrating..."); //Tells the user the gyro is calibrating
            telemetry.update(); //Updates telemetry
        } //End of while statement
        telemetry.addData("<", "Gyro calibrated, good luck!"); //Tells the user the gyro has finished calibrating
        telemetry.update(); //Updates telemetry
    } //Ends initiation
} //End of program

