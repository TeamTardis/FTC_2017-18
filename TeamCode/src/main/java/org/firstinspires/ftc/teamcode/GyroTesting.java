package org.firstinspires.ftc.teamcode;
/**
 * Created by Corning Robotics on 9/25/16.
 */
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static com.sun.tools.javac.util.Constants.format;
//Imports

@TeleOp(name="GyroTesting", group="TeleOp")
//@Disabled

public class GyroTesting extends OpMode {

    DcMotor m5;
    Servo s2;
    IntegratingGyroscope gyro; //Gyro
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    ElapsedTime runtime;

    public void init() { //Start of the initiation for autonomous

        m5 = hardwareMap.dcMotor.get("m5");

        s2 = hardwareMap.servo.get("s2"); //Sets s1 i the config
        s2.setPosition(.5);

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;

        modernRoboticsI2cGyro.calibrate(); //Gyro calibration

        while (modernRoboticsI2cGyro.isCalibrating()) { //Adds telemetry for gyro calibration
            telemetry.addData("", "Gyro Calibrating. Please wait..."); //Adds telemetry
            telemetry.update(); //Updates telemetry
        }

        telemetry.addData("", "Gyro Calibrated. Initializing Vuforia..."); //Adds telemetry
        telemetry.update(); //Updates telemetry

        runtime = new ElapsedTime(); //Creates runtime variable for using time

    } //Ends initiation

    boolean armMove = false;
    double armSpeed = 0;

    @Override
    public void loop() {

        double currentPosition = s2.getPosition();
        float LUD = gamepad1.left_stick_y;

        if((LUD > .1 || LUD < -.1)) {
            if(!armMove) {
                runtime.reset();
            }
            armMove = true;
        }
        if((LUD < .1 && LUD > -.1)) {
            armMove = false;
        }

        if(armMove) {
            armSpeed = (Math.pow(2.718281828, runtime.seconds())) / 20;
            if(LUD < -.1) {
                armSpeed = armSpeed*-1;
            }
        } else {
            armSpeed = 0;
        }

        if(armSpeed > .2) {
            armSpeed = .2;
        }
        if(armSpeed < -.3) {
            armSpeed = -.3;
        }

        m5.setPower(armSpeed);

        if(modernRoboticsI2cGyro.getIntegratedZValue() > 1 && currentPosition < 1) {
            currentPosition += .005;
        }
        if(modernRoboticsI2cGyro.getIntegratedZValue() < -1 && currentPosition > 0) {
            currentPosition -= .005;
        }

        s2.setPosition(currentPosition);

        telemetry.addData("","Arm speed " + armSpeed +"\nRuntime " + runtime.seconds() +"\nBoolean " + armMove); //Adds telemetry to debug
        telemetry.update(); //Updates telemetry with new information

    }
}

