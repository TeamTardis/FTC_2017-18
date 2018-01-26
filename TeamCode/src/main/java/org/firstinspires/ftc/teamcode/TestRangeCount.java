package org.firstinspires.ftc.teamcode;
/**
 * Created by Corning Robotics on 9/25/16.
 */

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.teamcode.RangeTestBedSteps.steps.FORWARD_UNTIL_COLUMN;
import static org.firstinspires.ftc.teamcode.RangeTestBedSteps.steps.RUNTIME_RESET;
//Imports

@Autonomous(name = "Range count", group = "Autonomous")
//@Disabled
public class TestRangeCount extends RangeTestBedSteps {
    DcMotor m1; //Define dcMotor as m1
    DcMotor m2; //Define dcMotor as m2
    DcMotor m3; //Define dcMotor as m3
    DcMotor m4; //Define dcMotor as m4
    DcMotor m5; //Define dcMotor as m4

    Servo s1; //Claw grip servo
    Servo s2; //Claw rotation servo

    IntegratingGyroscope gyro; //Gyro
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    ElapsedTime runtime;

    I2cDeviceSynch r1reader; //Right range sensor
    ModernRoboticsI2cRangeSensor r1;

    I2cDeviceSynch r2reader; //Right range sensor
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

        s1.setPosition(1);
        s2.setPosition(0);

        r1reader = hardwareMap.i2cDeviceSynch.get("r1"); //Port 1 (Right side)
        r1 = new ModernRoboticsI2cRangeSensor(r1reader);
        r1.setI2cAddress(I2cAddr.create8bit(0x2a));

        r2reader = hardwareMap.i2cDeviceSynch.get("r2"); //Port 1 (Right side)
        r2 = new ModernRoboticsI2cRangeSensor(r1reader);
        r2.setI2cAddress(I2cAddr.create8bit(0x2c));

        runtime = new ElapsedTime(); //Creates runtime variable for using time

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;

        modernRoboticsI2cGyro.calibrate(); //Gyro calibration

        while (modernRoboticsI2cGyro.isCalibrating()) { //Adds telemetry for gyro calibration
            telemetry.addData("", "Gyro Calibrating. Please wait..."); //Adds telemetry
            telemetry.update(); //Updates telemetry
        }

        telemetry.addData("", "Gyro Calibrated. Initializing Vuforia..."); //Adds telemetry
        telemetry.update(); //Updates telemetry

    } //Ends initiation

    int pillarCount = 0;
    double rangeCM1;
    double rangePrevCM1 = 0;
    double rangeCM2;
    double rangePrevCM2 = 0;

    steps CURRENT_STEP = RUNTIME_RESET; //Sets the variable CURRENT_STEP to the first step in the sequence

    @Override
    public void loop() {

        telemetry.addData("Range: ", r1.getDistance(DistanceUnit.CM) + "\nPillar count: " + pillarCount); //Adds telemetry to debug
        telemetry.update(); //Updates telemetry with new information

        rangeCM1 = r1.getDistance(DistanceUnit.CM);
        rangeCM2 = r2.getDistance(DistanceUnit.CM);

        switch (CURRENT_STEP) { //Beginning of the switch. this sets the current step to whatever CURRENT_STEP is set to

            ///////////////////////
            //START OF MAIN STEPS//
            ///////////////////////

            case RUNTIME_RESET:

                runtime.reset();
                CURRENT_STEP = steps.DRIVE_OFF_BALANCE;
                break;

            case DRIVE_OFF_BALANCE:

                if (runtime.seconds() > 2.5) {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    CURRENT_STEP = steps.STRAFE;
                    break;
                }
                m1.setPower(.1);
                m2.setPower(.1);
                m3.setPower(.1);
                m4.setPower(.1);
                break;

            case STRAFE:

                if (r2.getDistance(DistanceUnit.CM) <= 25) {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset();
                    CURRENT_STEP = steps.FORWARD_UNTIL_COLUMN;
                    break;
                }
                m1.setPower(.2);
                m2.setPower(-.2);
                m3.setPower(-.2);
                m4.setPower(.2);
                break;

            case FORWARD_UNTIL_COLUMN:

                rangeCM2 = r2.getDistance(DistanceUnit.CM);

                if (rangeCM2 < 10 || rangeCM2 > 40) {
                    rangeCM2 = rangePrevCM2;
                }
                if (rangeCM2 - rangePrevCM2 > 2 && runtime.seconds() > .5 && rangePrevCM2 != 0) {
                    pillarCount++;
                    runtime.reset();
                }
                if (pillarCount != 3) {
                    m1.setPower(.1);
                    m2.setPower(.1);
                    m3.setPower(.1);
                    m4.setPower(.1);
                } else {
                    CURRENT_STEP = steps.ROTATION;
                    break;
                }
                rangePrevCM2 = rangeCM2;
                break;

            case ROTATION:

                if(modernRoboticsI2cGyro.getIntegratedZValue() > 90) {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset();
                    CURRENT_STEP = steps.STOP;
                    break;
                }
                m1.setPower(.1);
                m2.setPower(-.1);
                m3.setPower(.1);
                m4.setPower(-.1);
                break;

            case STRAFE_RIGHT:

                rangeCM1 = r1.getDistance(DistanceUnit.CM);

                if (rangeCM1 < 10 || rangeCM1 > 40) {
                    rangeCM1 = rangePrevCM1;
                }
                if (rangeCM1 - rangePrevCM1 > 2 && rangePrevCM1 != 0) {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset();
                    CURRENT_STEP = steps.CENTER;
                    break;
                }
                m1.setPower(-.15);
                m2.setPower(.15);
                m3.setPower(.15);
                m4.setPower(-.15);
                rangePrevCM1 = rangeCM1;
                break;

            case CENTER:

                if (runtime.seconds() < .5) {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset();
                    CURRENT_STEP = steps.STOP;
                    break;
                }
                m1.setPower(.1);
                m2.setPower(-.1);
                m3.setPower(-.1);
                m4.setPower(.1);
                break;

            case STOP:

                m1.setPower(0);
                m2.setPower(0);
                m3.setPower(0);
                m4.setPower(0);
                break;

        }
    }
}