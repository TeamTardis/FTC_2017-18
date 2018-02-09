package org.firstinspires.ftc.teamcode;
/**
 * Created by Corning Robotics on 9/25/16.
 */

//import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.teamcode.RangeTestBedSteps.steps.RUNTIME_RESET;
//Imports

@TeleOp(name = "Driving Test", group = "TeleOp")
public class DriverTesting extends RangeTestBedSteps {
    DcMotor m1; //Define dcMotor as m1
    DcMotor m2; //Define dcMotor as m2
    DcMotor m3; //Define dcMotor as m3
    DcMotor m4; //Define dcMotor as m4
    DcMotor m5; //Define dcMotor as m4

    Servo s1; //Claw grip servo
    Servo s2; //Claw rotation servo

    TouchSensor touchSensor1; //Define touch sensor as touchSensor1 (mast limit switch)

    IntegratingGyroscope gyro; //Gyro
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    ElapsedTime runtime;

    OpticalDistanceSensor ods1;
    OpticalDistanceSensor ods2;

//    ModernRoboticsAnalogOpticalDistanceSensor ods1;
//    ModernRoboticsAnalogOpticalDistanceSensor ods2;

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

        ods1 = hardwareMap.get(OpticalDistanceSensor.class, "ods1");
        ods2 = hardwareMap.get(OpticalDistanceSensor.class, "ods2");
        ods1.enableLed(true);
        ods2.enableLed(true);

        m1.setDirection(DcMotor.Direction.REVERSE);
        m3.setDirection(DcMotor.Direction.REVERSE);

        touchSensor1 = hardwareMap.touchSensor.get("t1"); //Sets touchSensor1 to t1 in the config

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

//        telemetry.addData("", "Gyro Calibrated. Initializing Vuforia..."); //Adds telemetry
//        telemetry.update(); //Updates telemetry

        s1.setPosition(0);
    } //Ends initiation

    double rangeCM1;
    double rangePrevCM1 = 0;
    double rangeCM2;
    double rangePrevCM2 = 0;
    float encoderPause;
    int leftcolumn = 2000;
    int centercolumn = 2200;
    int rightcolumn = 2000;
    double speed = 0;
    double target = 0;

    steps CURRENT_STEP = RUNTIME_RESET; //Sets the variable CURRENT_STEP to the first step in the sequence

    @Override
    public void loop() {

        float LUD = gamepad1.left_stick_y; //Variable for left stick y axis
        float LRL = gamepad1.left_stick_x; //Variable for left stick x axis
        float RLR = -gamepad1.right_stick_x; //Variable for right stick x axis on gamepad 1 for driver control

        /*
        double m1m4 = -1.4279*Math.sin((modernRoboticsI2cGyro.getIntegratedZValue()*0.9984909/57.2)-0.783565);
        double m2m3 = -1.4279*Math.sin((modernRoboticsI2cGyro.getIntegratedZValue()*0.9984909/57.2)-2.357004);

        if(m1m4 > 1) {
            m1m4 = 1;
        } else if(m1m4 < -1) {
            m1m4 = -1;
        }

        if(m2m3 > 1) {
            m2m3 = 1;
        } else if(m2m3 < -1) {
            m2m3 = -1;
        }

        m1.setPower((m1m4*2 + 1.871));
        m2.setPower(-(m2m3*2) - 1.871);
        m3.setPower(-(m2m3*2) + 1.871);
        m4.setPower((m1m4*2 - 1.871));
        */
        telemetry.addData("Ods1 light detected: ", ods1.getLightDetected() + "\nOds2 light detected: " + ods2.getLightDetected() + "\nGyro: " + modernRoboticsI2cGyro.getIntegratedZValue()); //Adds telemetry to debug
        telemetry.update(); //Updates telemetry with new information

        rangeCM1 = r1.getDistance(DistanceUnit.CM);
        rangeCM2 = r2.getDistance(DistanceUnit.CM);
    }
}