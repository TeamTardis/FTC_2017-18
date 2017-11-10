package org.firstinspires.ftc.teamcode;
/**
 * Created by Corning Robotics on 9/25/16.
 */

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//Imports

@TeleOp(name="SensorDebug", group="TeleOp")
@Disabled
public class SensorDebugging extends OpMode {
    DcMotor m1; //Define dcMotor as m1
    DcMotor m2; //Define dcMotor as m2
    DcMotor m3; //Define dcMotor as m3
    DcMotor m4; //Define dcMotor as m4
    DcMotor m5; //Define dcMotor as m4

    Servo s1;
    Servo s2;

    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    ColorSensor c1;

    I2cDeviceSynch r2reader;
    ModernRoboticsI2cRangeSensor r2;

    I2cDeviceSynch r1reader;
    ModernRoboticsI2cRangeSensor r1;

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

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;
        modernRoboticsI2cGyro.calibrate();

        c1 = hardwareMap.colorSensor.get("c1"); //Sets colorSensor to c1 in the config
        c1.enableLed(true); //Turns Color Sensor LED off

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

    ElapsedTime runtime = new ElapsedTime();

    float straight = 0;
    double turn = 0;

    @Override
    public void loop() {

        int heading = modernRoboticsI2cGyro.getHeading(); //Gyro heading value
        int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue(); //Gyro integratedZ value

        float LUD = gamepad1.left_stick_y; //Variable for left stick y axis
        float LRL = -gamepad1.left_stick_x; //Variable for left stick x axis
        float RUD = gamepad1.right_stick_y; //Variable for right stick y axis
        float RLR = -gamepad1.right_stick_x; //Variable for right stick x axis

        double r1cm = r1.getDistance(DistanceUnit.CM);
        double r2cm = r2.getDistance(DistanceUnit.CM);

        if(gamepad2.y){
            s2.setPosition(0);
        }

        if(gamepad2.x){
            s2.setPosition(1);
        }

        if(gamepad2.left_stick_y != 0) {
            m5.setPower(gamepad2.left_stick_y / 4);
        } else {
            m5.setPower(0);
        }

        if (RLR != 0) { //Checks if the robot is rotating
            straight = integratedZ; //If rotating, then set the straight varible to the integratedZ value
        } //End of if statement

        if (integratedZ < straight && LUD != 0 && LRL != 0) { //Checks to see if the robot is moving and the value of straight is greater than the value integratedZ
            turn = .4; //Sets the turn value to .4
        } else if (integratedZ > straight && LUD != 0 && LRL != 0) { //Checks to see if the robot is moving and the value of straight is less than the value integratedZ
            turn = -.4; //Sets the turn value to -.4
        } else { //Default value (robot is not moving)
            turn = 0; //Sets the turn value to 0
        } //End of else statement

        //(Range_Rightcache[0] & 0xFF)

        telemetry.addData("", "Gyro ZValue: " + integratedZ + "\nGyro Straight:" + straight + "\nR1 CM: " + r1cm + "\nR2 CM: " + r2cm); //Adds telemetry for debugging
        telemetry.update(); //Updates telemetry

        if(gamepad1.right_trigger == 0) { //Controls for normal mode

            if (RLR == 0) {
                m1.setPower(((LRL + LUD) / 2) - turn); //Steering for top left
                m2.setPower(((LUD - LRL) / 2) + turn); //Steering for top right
                m3.setPower(((LUD - LRL) / 2) - turn); //Steering for back left
                m4.setPower(((LRL + LUD) / 2) + turn); //Steering for back right
            } else {
                m1.setPower(RLR); //Turning for top left motor
                m2.setPower(-RLR); //Turning for top right motor
                m3.setPower(RLR); //Turning for back left motor
                m4.setPower(-RLR); //Turning for back right motor
            }
        }

        else { //Controls for slow mode

            if (RLR == 0) {
                m1.setPower(((LRL + LUD) / 8) - turn); //Steering for top left
                m2.setPower(((LUD - LRL) / 8) + turn); //Steering for top right
                m3.setPower(((LUD - LRL) / 8) - turn); //Steering for back left
                m4.setPower(((LRL + LUD) / 8) + turn); //Steering for back right
            } else {
                m1.setPower(RLR / 4); //Turning for top left motor
                m2.setPower(-RLR / 4); //Turning for top right motor
                m3.setPower(RLR / 4); //Turning for back left motor
                m4.setPower(-RLR / 4); //Turning for back right motor
            }
        }

    }
}