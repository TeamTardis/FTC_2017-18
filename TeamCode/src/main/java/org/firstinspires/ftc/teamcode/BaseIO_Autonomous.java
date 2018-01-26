package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by dobbinsms on 11/14/2017.
 */

public abstract class BaseIO_Autonomous extends LinearOpMode {

    public enum steps { //All steps for completing autonomous

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //Main steps - These steps run in sequence when the program is going perfect with no unexpected readings in sensor inputs.//
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        STARTCAMERA,
        SCANIMAGE,
        LOWERSERVO,
        SENSECOLOR,
        KNOCKFORWARDS,
        KNOCKBACK,
        RAISESERVO,
        DRIVETOCRYPTOBOX,
        ROTATE,
        MOVE_CLOSER,
        LEFTCOLUMN,
        CENTERCLOMUN,
        RIGHTCOLUMN,
        STOP //[Stop] The title says it all.

    } //End of steps for autonomous

    /**
     * Front Left Motor, gearbox 40
     */
    DcMotor m1;

    /**
     * Front Right Motor, gearbox 40
     */
    DcMotor m2;

    /**
     * Back Right Motor, gearbox 40
     */
    DcMotor m3;

    /**
     * Back Left Motor, gearbox 40
     */
    DcMotor m4;

    /**
     * Arm Raise Motor, gearbox 60
     */
    DcMotor m5;

    /**
     * Arm Rotating Base Motor, gearbox 60
     */
    DcMotor m6;

    /**
     * Jewel Arm Servo, 190 degrees
      */
    Servo s1; //Color sensor arm servo

    /**
     * 1st Claw Grip Servo, 190 degrees
     */
    Servo s2; //Claw grip servo

    /**
     * Wrist Rotation Servo, 190 degrees, extended mode
     */
    Servo s3; //Wrist rotation

    /**
     * Claw Vertical Servo, continuous
     */
    Servo s4; //Claw vertical

    /**
     * 2nd Claw Grip Servo, 190 degrees
     */
    Servo s5; //Second claw grip

    /**
     * Arm Extension Servo, continuous
     */
    Servo s6; //Arm extension

    /**
     * Time Variable, use runtime.seconds()
     */
    ElapsedTime runtime;

    VuforiaLocalizer vuforia;

    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    ColorSensor c1;

    I2cDeviceSynch r1reader;
    ModernRoboticsI2cRangeSensor r1;

    I2cDeviceSynch r2reader;
    ModernRoboticsI2cRangeSensor r2;

    I2cDeviceSynch r3reader;
    ModernRoboticsI2cRangeSensor r3;

    I2cDeviceSynch r4reader;
    ModernRoboticsI2cRangeSensor r4;

    public BaseIO_Autonomous()
    {

        m1 = hardwareMap.dcMotor.get("m1"); //Sets m1 to m1 in the config
        m2 = hardwareMap.dcMotor.get("m2"); //Sets m2 to m2 in the config
        m3 = hardwareMap.dcMotor.get("m3"); //Sets m3 to m3 in the config
        m4 = hardwareMap.dcMotor.get("m4"); //Sets m4 to m4 in the config
        m5 = hardwareMap.dcMotor.get("m5"); //Sets m5 to m5 in the config
        m6 = hardwareMap.dcMotor.get("m6"); //Sets m6 to m6 in the config

        s1 = hardwareMap.servo.get("s1"); //Sets s1 in the config
        s2 = hardwareMap.servo.get("s2"); //Sets s2 in the config
        s3 = hardwareMap.servo.get("s3"); //Sets s3 in the config
        s4 = hardwareMap.servo.get("s4"); //Sets s4 in the config
        s5 = hardwareMap.servo.get("s5"); //Sets s5 in the config
        s6 = hardwareMap.servo.get("s6"); //Sets s6 in the config

        m1.setDirection(DcMotor.Direction.REVERSE);
        m3.setDirection(DcMotor.Direction.REVERSE);

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;

        c1 = hardwareMap.colorSensor.get("c1"); //Sets colorSensor to c1 in the config, Port 5
        c1.enableLed(true); //Turns Color Sensor LED off


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

        runtime = new ElapsedTime(); //Creates runtime variable for using time

    }



}

