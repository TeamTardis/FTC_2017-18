
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//Imports Files for robot parts

@TeleOp(name = "Sensor Test", group = "TeleOp")
public class SensorTest extends OpMode {

    DcMotor m1; //Front left motor
    DcMotor m2; //Front right motor
    DcMotor m3; //Back left motor
    DcMotor m4; //Back right motor
    DcMotor m5; //Arm raise motor
    DcMotor m6; //Arm base rotation motor
    DcMotor m7; //Arm crunch vertical motor

    Servo s1; //Color sensor arm servo
    Servo s2; //Relic Claw servo
    Servo s3; //Arm Crunch A
    Servo s4; //Arm Crunch B
    //    Servo s5; //Second claw grip
    Servo s6; //Arm extension

    IntegratingGyroscope gyro; //Gyro
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    ColorSensor c1; //Color sensor

    I2cDeviceSynch r1reader;
    ModernRoboticsI2cRangeSensor r1; //Right range sensor

    I2cDeviceSynch r2reader;
    ModernRoboticsI2cRangeSensor r2; //Front range sensor

    I2cDeviceSynch r3reader;
    ModernRoboticsI2cRangeSensor r3; //Left range sensor

    I2cDeviceSynch r4reader;
    ModernRoboticsI2cRangeSensor r4; //Back range sensor

    TouchSensor touchSensor1; //Define touch sensor as touchSensor1 (mast limit switch)

    ElapsedTime runtime; //Time variable
    float straight = 0; //Variable for straight motion
    double turn = 0; //Variable for turn
    double wristPosition = 0; //Variable for wristPosition
    double armPosition = 0.5;
    float encoderCurrent; //Variable for current encoder value

    public void init() { //Start of the initiation for autonomous

        m1 = hardwareMap.dcMotor.get("m1"); //Sets m1 to m1 in the config
        m2 = hardwareMap.dcMotor.get("m2"); //Sets m2 to m2 in the config
        m3 = hardwareMap.dcMotor.get("m3"); //Sets m3 to m3 in the config
        m4 = hardwareMap.dcMotor.get("m4"); //Sets m4 to m4 in the config
        m5 = hardwareMap.dcMotor.get("m5"); //Sets m5 to m5 in the config
        m6 = hardwareMap.dcMotor.get("m6"); //Sets m6 to m6 in the config
        m7 = hardwareMap.dcMotor.get("m7"); //Sets m7 to m7 in the config

        s1 = hardwareMap.servo.get("s1"); //Sets s1 in the config
        s2 = hardwareMap.servo.get("s2"); //Sets s2 in the config
        s3 = hardwareMap.servo.get("s3"); //Sets s3 in the config
        s4 = hardwareMap.servo.get("s4"); //Sets s4 in the config
//        s5 = hardwareMap.servo.get("s5"); //Sets s5 in the config
        s6 = hardwareMap.servo.get("s6"); //Sets s6 in the config

        m2.setDirection(DcMotor.Direction.REVERSE); //Sets m2 direction to REVERSE
        m4.setDirection(DcMotor.Direction.REVERSE); //Sets m4 direction to REVERSE

        touchSensor1 = hardwareMap.touchSensor.get("t1"); //Sets touchSensor1 to t1 in the config

        runtime = new ElapsedTime(); //Creates runtime variable for using time

        c1 = hardwareMap.colorSensor.get("c1");

        s1.setPosition(0); //Pulls jewel appendage against side of robot
        s2.setPosition(1); //Opens Relic Claw
        s3.setPosition(0.85); //Closes arm crunch
        s4.setPosition(0.45);
//        s5.setPosition(0.5); //Opens 2nd gripper *NOT USED*
        s6.setPosition(0.5); //Sets arm extension to not move

        r1reader = hardwareMap.i2cDeviceSynch.get("r1"); //Port 1 (Right side)
        r1 = new ModernRoboticsI2cRangeSensor(r1reader);
        r1.setI2cAddress(I2cAddr.create8bit(0x2a));

        r2reader = hardwareMap.i2cDeviceSynch.get("r2"); //Port 2 (Front side)
        r2 = new ModernRoboticsI2cRangeSensor(r2reader);
        r2.setI2cAddress(I2cAddr.create8bit(0x2c));

        r3reader = hardwareMap.i2cDeviceSynch.get("r3"); //Port 3 (Left side)
        r3 = new ModernRoboticsI2cRangeSensor(r3reader);
        r3.setI2cAddress(I2cAddr.create8bit(0x26));

        r4reader = hardwareMap.i2cDeviceSynch.get("r4"); //Port 4 (Back side)
        r4 = new ModernRoboticsI2cRangeSensor(r4reader);
        r4.setI2cAddress(I2cAddr.create8bit(0x28));

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;

    } //Ends initiation

    @Override
    public void loop() {  //Start of main loop

        telemetry.addData("Step", "ColorBlue: " + c1.blue()
                + "\nColorRed: " + c1.red() + "\nRange1: " + r1.getDistance(DistanceUnit.CM) + "\nRange2: " + r2.getDistance(DistanceUnit.CM)
                + "\nRange3: " + r3.getDistance(DistanceUnit.CM) + "\nRange4: " + r4.getDistance(DistanceUnit.CM) + "\nGyro: " + modernRoboticsI2cGyro.getIntegratedZValue()
                + "\nRuntime: " + runtime.seconds()); //Adds telemetry to debug
        telemetry.update(); //Updates telemetry with new information

    }
}