package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
/**
 * Created by dobbinsms on 11/14/2017.
 */

public abstract class BaseIO_TardisRelicRecovery extends OpMode {

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

    public BaseIO_TardisRelicRecovery()
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

        m2.setDirection(DcMotor.Direction.REVERSE); //Sets m2 direction to REVERSE
        m4.setDirection(DcMotor.Direction.REVERSE); //Sets m4 direction to REVERSE

        runtime = new ElapsedTime(); //Creates runtime variable for using time

    }



}

