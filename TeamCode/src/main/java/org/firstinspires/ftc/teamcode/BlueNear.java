//Blue Near
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class BlueNear extends LinearOpMode { //Extends LinearOpMode for autonomous

    public static final String TAG = "BlueNear";

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
     * Arm Crunch Vertical, gearbox 40
     */
    DcMotor m7;

    /**
     * Jewel Arm Servo, 190 degrees
     */
    Servo s1; //Color sensor arm servo

    /**
     * 1st Claw Grip Servo, 190 degrees
     */
    Servo s2; //Claw grip servo

    /**
     * Left Arm Crunch Servo, 190 degrees
     */
    Servo s3; //Left arm crunch servo

    /**
     * Right Arm Crunch Servo, 190 degrees
     */
    Servo s4; //Right arm crunch servo

    /**
     * 2nd Claw Grip Servo, 190 degrees
     */
    //Servo s5; //Second claw grip

    /**
     * Arm Extension Servo, continuous
     */
    Servo s6; //Arm extension

    /**
     * Time Variable, use runtime.seconds()
     */
    ElapsedTime runtime;
    ElapsedTime checkTime;

    IntegratingGyroscope gyro; //Gyro
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    ColorSensor c1; //Color sensor

    I2cDeviceSynch r1reader; //Right range sensor
    ModernRoboticsI2cRangeSensor r1;

    I2cDeviceSynch r2reader; //Front range sensor
    ModernRoboticsI2cRangeSensor r2;

    I2cDeviceSynch r3reader; //Left range sensor
    ModernRoboticsI2cRangeSensor r3;

    I2cDeviceSynch r4reader; //Back range sensor
    ModernRoboticsI2cRangeSensor r4;

    OpticalDistanceSensor ods1;
    OpticalDistanceSensor ods2;
    OpticalDistanceSensor ods3;

    VuforiaLocalizer vuforia; //Image recognition

    /**
     * Positive = Forward
     **/
    public void setDrivePower(double power, double turn) {

        m1.setPower(power - turn); //Drives robot forwards or backwards, availability for turn variable
        m2.setPower(power + turn);
        m3.setPower(power - turn);
        m4.setPower(power + turn);
    }

    /**
     * Positive = Right
     **/
    public void setStrafePower(double power, double turn) {

        m1.setPower(power - turn); //Drives robot to strafe left or right, availability for turn variable
        m2.setPower(-power + turn);
        m3.setPower(-power - turn);
        m4.setPower(power + turn);
    }

    /**
     * Positive = Forward
     **/
    public void setDriveEncoder(double encoder) {

        double power = -((2 / (1 + Math.pow(Math.E, -.01 * ((encoder - m2.getCurrentPosition())/10)))) - 1);
        m1.setPower(power); //Drives robot forwards or backwards, availability for turn variable
        m2.setPower(power);
        m3.setPower(power);
        m4.setPower(power);
    }

    /**
     * Positive = Clockwise
     **/
    public void setRotationPower(double power) {

        m1.setPower(power); //Drives robot to rotate
        m2.setPower(-power);
        m3.setPower(power);
        m4.setPower(-power);
    }

    /**
     * Positive = Clockwise
     **/
    public void setRotationTarget(double target) {//.02, .03

        double power = 0;
        double gyro = modernRoboticsI2cGyro.getIntegratedZValue();
        double shift = .2;
        if(gyro < 0) {
            power = (2/(1+Math.pow(Math.E, -(.006*(target - gyro) - shift)))) - 1;
        } else {
            power = (2/(1+Math.pow(Math.E, -(.006*(target - gyro) + shift)))) - 1;
        }
        m1.setPower(power); //Drives robot to rotate
        m2.setPower(-power);
        m3.setPower(power);
        m4.setPower(-power);
    }

    /**
     * Positive = Clockwise
     **/
    public double setRotationPercise(double target, double currPower) {//.02, .03

        double gyro = modernRoboticsI2cGyro.getIntegratedZValue();
        double newPower = 0;
        if(gyro > target) {
            newPower = currPower -= .015;
        }
        if(gyro < target) {
            newPower = currPower += .015;
        }
        return newPower;
    }

    public enum steps {
        SCANIMAGE, //Runs vuforia to scan pictograph
        LOWERSERVO, //Extends jewel appendage
        SENSECOLOR, //Determines whether one jewel is red or blue
        KNOCKFORWARDS, //Knocks in direction of relic recovery zone
        KNOCKBACK, //Knocks away from relic recovery zone
        RAISESERVO,
        OFF_STONE,
        CHECK_ROTATION,
        BACK_STONE,
        DRIVE_TO_CRYPTO,
        ROTATE,
        PRECISE_ROTATE,
        CHECK_DISTANCE,
        DROP_GLYPH,
        FIND_GLYPHS,
        SCAN_GLYPHS,
        FORWARD_GLYPH,
        GRAB_GLYPH,
        RESET,
        STOP
    }

    steps CURRENT_STEP = steps.SCANIMAGE; //Sets the variable CURRENT_STEP to the first step in the sequence

    public void changeStep() {

        setDrivePower(0,0);
        runtime.reset();
    }

    public void gripClose() {

        s3.setPosition(0.85); //Closes arm crunch
        s4.setPosition(0.45);
    }

    public void gripOpen() {

        s3.setPosition(0.35); //Sets Arm Crunch Servo A
        s4.setPosition(1); //Sets Arm Crunch Servo B
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}