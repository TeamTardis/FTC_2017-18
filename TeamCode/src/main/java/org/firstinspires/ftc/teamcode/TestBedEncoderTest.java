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
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.teamcode.RangeTestBedSteps.steps.EXPLORE;
import static org.firstinspires.ftc.teamcode.RangeTestBedSteps.steps.FORWARD_UNTIL_COLUMN;
import static org.firstinspires.ftc.teamcode.RangeTestBedSteps.steps.RUNTIME_RESET;
//Imports

@Autonomous(name = "TestBedEncoderTest", group = "Autonomous")
public class TestBedEncoderTest extends RangeTestBedSteps {
    DcMotor m1; //Define dcMotor as m1
    DcMotor m2; //Define dcMotor as m2
    DcMotor m3; //Define dcMotor as m3
    DcMotor m4; //Define dcMotor as m4
    DcMotor m5; //Define dcMotor as m4

    Servo s1; //Claw grip servo
    Servo s2; //Claw rotation servo
    Servo s3;

    OpticalDistanceSensor ods1;
    OpticalDistanceSensor ods2;
    OpticalDistanceSensor ods3;

    TouchSensor touchSensor1; //Define touch sensor as touchSensor1 (mast limit switch)

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
        s3 = hardwareMap.servo.get("s3");

        m1.setDirection(DcMotor.Direction.REVERSE);
        m3.setDirection(DcMotor.Direction.REVERSE);

        touchSensor1 = hardwareMap.touchSensor.get("t1"); //Sets touchSensor1 to t1 in the config

        ods1 = hardwareMap.get(OpticalDistanceSensor.class, "ods1");
        ods2 = hardwareMap.get(OpticalDistanceSensor.class, "ods2");
        ods3 = hardwareMap.get(OpticalDistanceSensor.class, "ods3");
        ods1.enableLed(true);
        ods2.enableLed(true);
        ods3.enableLed(true);

        s1.setPosition(0);
        s3.setPosition(0.51);

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
    double glyphODS = 0;
    int squaredirection = 0;

    steps CURRENT_STEP = RUNTIME_RESET; //Sets the variable CURRENT_STEP to the first step in the sequence

    @Override
    public void loop() {

        telemetry.addData("Step", CURRENT_STEP + "\nODS3: " + ods3.getLightDetected() + "\nGlyphODS: "
                + glyphODS + "\nEncoder Pause: " + encoderPause + "\nABS: " + Math.abs(ods1.getLightDetected() + ods2.getLightDetected())
        ); //Adds telemetry to debug
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
                    runtime.reset();
                    CURRENT_STEP = steps.BACKWARDS;
                    break;
                }
                m1.setPower(.1);
                m2.setPower(.1);
                m3.setPower(.1);
                m4.setPower(.1);
                s1.setPosition(0);
                break;

            case BACKWARDS:

                if (runtime.seconds() > 2) {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    CURRENT_STEP = steps.ENCODER_FORWARDS;
                    encoderPause = m1.getCurrentPosition();
                    break;
                }
                m1.setPower(-0.07);
                m2.setPower(-0.07);
                m3.setPower(-0.07);
                m4.setPower(-0.07);
                break;

            case ENCODER_FORWARDS:
                if ((m1.getCurrentPosition() - encoderPause) > centercolumn) {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset();
                    CURRENT_STEP = steps.ROTATION;
                    break;
                } else if (runtime.seconds() > 1) {
                    m1.setPower(.1);
                    m2.setPower(.1);
                    m3.setPower(.1);
                    m4.setPower(.1);
                    break;
                }
                break;

            case ROTATION:

                target = 90;
                speed = (2 / (1 + Math.pow(Math.E, -.015 * (target - modernRoboticsI2cGyro.getIntegratedZValue())))) - 1;
                if (modernRoboticsI2cGyro.getIntegratedZValue() > target - 2 && modernRoboticsI2cGyro.getIntegratedZValue() < target + 2) {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset();
                    CURRENT_STEP = steps.FORWARD;
                    break;
                }
                m1.setPower(-speed);
                m2.setPower(speed);
                m3.setPower(-speed);
                m4.setPower(speed);
                break;

            case FORWARD: //Beginning of the case statement FORWARD

                if (runtime.seconds() > 1.5) { //Runs for 2.5 seconds
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset(); //Resets the runtime
                    CURRENT_STEP = steps.DROP; //Changes step to DROP
                    break; //Exits switch statement
                }
                m1.setPower(0.1);
                m2.setPower(0.1);
                m3.setPower(0.1);
                m4.setPower(0.1); //Drive forward without using gyro
                break; //Exits switch statement

            case DROP: //Beginning of the case statement DROP

                s1.setPosition(1); //Opens crunch servo
                m1.setPower(0);
                m2.setPower(0);
                m3.setPower(0);
                m4.setPower(0);
                runtime.reset(); //Resets the runtime
                CURRENT_STEP = steps.BACK; //Changes step to BACK
                break; //Exits switch statement

            case BACK: //Beginning of the case statement BACK

                if (runtime.seconds() > 0.8) { //Runs for 0.5 seconds
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset(); //Resets the runtime
                    CURRENT_STEP = steps.ROTATION2; //Changes step to FORWARD2
                    break; //Exits switch statement
                }
                m1.setPower(-0.1);
                m2.setPower(-0.1);
                m3.setPower(-0.1);
                m4.setPower(-0.1); //Drives robot backwards without using gyro
                break; //Exits switch statement

            case ROTATION2:

                target = -90;
                speed = (2 / (1 + Math.pow(Math.E, -.015 * (target - modernRoboticsI2cGyro.getIntegratedZValue())))) - 1;
                if (modernRoboticsI2cGyro.getIntegratedZValue() > target - 2 && modernRoboticsI2cGyro.getIntegratedZValue() < target + 2) {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset();
                    CURRENT_STEP = steps.EXPLORE;
                    break;
                }
                m1.setPower(-speed);
                m2.setPower(speed);
                m3.setPower(-speed);
                m4.setPower(speed);
                break;

            case EXPLORE:

                /*
                if (ods1.getLightDetected() > 0.1 || ods2.getLightDetected() > 0.1 || touchSensor1.isPressed()) { //Runs for 1 second
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset(); //Resets the runtime
                    CURRENT_STEP = steps.EXPLOREHARDER; //Changes step to BACK2
                    break; //Exits switch statement
                }
                if (runtime.seconds() < 0.3) {
                    m1.setPower(0.3);
                    m2.setPower(0.0);
                    m3.setPower(0.3);
                    m4.setPower(0.0);
                } else if (runtime.seconds() > 0.3 && runtime.seconds() < 0.6) {
                    m1.setPower(0.0);
                    m2.setPower(0.3);
                    m3.setPower(0.0);
                    m4.setPower(0.3);
                } else if (runtime.seconds() > 0.6) {
                    runtime.reset();
                }
                */
                if (ods3.getLightDetected() > .008) {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    glyphODS = ods3.getLightDetected();
                    runtime.reset(); //Resets the runtime
                    CURRENT_STEP = steps.EXPLOREHARDER; //Changes step to BACK2
                    break; //Exits switch statement
                }
                if (ods1.getLightDetected() > .1 && ods2.getLightDetected() > .1) {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    glyphODS = ods3.getLightDetected();
                    runtime.reset(); //Resets the runtime
                    CURRENT_STEP = steps.FORWARDS; //Changes step to BACK2
                    break; //Exits switch statement
                }
                m1.setPower(.1);
                m2.setPower(.1);
                m3.setPower(.1);
                m4.setPower(.1);
                s1.setPosition(0.8);
                break; //Exits switch statement

            case EXPLOREHARDER:

                /*
                if ((ods1.getLightDetected() > 0.1 && ods2.getLightDetected() > 0.1) || touchSensor1.isPressed()) { // || touchSensor1.isPressed()) { //Runs for 1 second
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset(); //Resets the runtime
                    CURRENT_STEP = steps.GRAB; //Changes step to BACK2
                    break; //Exits switch statement
                } else if (ods1.getLightDetected() < 0.1 && ods2.getLightDetected() < 0.1) {
                    runtime.reset();
                    CURRENT_STEP = steps.EXPLORE;
                }
                if (ods1.getLightDetected() > ods2.getLightDetected()) {

                    if (runtime.seconds() < 0.5) {
                        m1.setPower(-0.2);
                        m2.setPower(-0.2);
                        m3.setPower(-0.2);
                        m4.setPower(-0.2);
                    } else if (runtime.seconds() > 0.5 && runtime.seconds() < 1.3) {
                        m1.setPower(0.2);
                        m2.setPower(-0.05);
                        m3.setPower(0.2);
                        m4.setPower(-0.05);
                    } else if (runtime.seconds() > 1.3) {
                        runtime.reset();
                    }
                } else if (ods1.getLightDetected() < ods2.getLightDetected()) {

                    if (runtime.seconds() < 0.5) {
                        m1.setPower(-0.2);
                        m2.setPower(-0.2);
                        m3.setPower(-0.2);
                        m4.setPower(-0.2);
                    } else if (runtime.seconds() > 0.5 && runtime.seconds() < 1.3) {
                        m1.setPower(-0.05);
                        m2.setPower(0.2);
                        m3.setPower(-0.05);
                        m4.setPower(0.2);
                    } else if (runtime.seconds() > 1.3) {
                        runtime.reset();
                    }
                }
                */
                if (ods3.getLightDetected() < glyphODS - .003) {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset(); //Resets the runtime
                    CURRENT_STEP = steps.SQUAREUP; //Changes step to BACK2
                    break; //Exits switch statement
                }
                m1.setPower(.2);
                m2.setPower(-.2);
                m3.setPower(-.2);
                m4.setPower(.2);
                break; //Exits switch statement

            case SQUAREUP:

                if ((Math.abs(ods1.getLightDetected() + ods2.getLightDetected()) < 0.04 && ods1.getLightDetected() > 0.13)  || (ods1.getLightDetected() > 0.13 && ods2.getLightDetected() > 0.13)) {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset(); //Resets the run
                    CURRENT_STEP = steps.FORWARDS; //
                    break; //Exits switch statement
                }


//                if ((ods1.getLightDetected() > 0.13 && ods2.getLightDetected() > 0.13)) { // || touchSensor1.isPressed()) { //Runs for 1 second
//                    m1.setPower(0);
//                    m2.setPower(0);
//                    m3.setPower(0);
//                    m4.setPower(0);
//                    runtime.reset(); //Resets the runtime
//                    CURRENT_STEP = steps.FORWARDS; //Changes step to BACK2
//                    break; //Exits switch statement
//                }
//
                if (runtime.seconds() < 0.5) { //Scans slope of glyph to determine how to position

                    if (ods1.getLightDetected() > ods2.getLightDetected()) { //Go left
                        squaredirection = 1;
                    } else if (ods1.getLightDetected() < ods2.getLightDetected()) { //Go right
                        squaredirection = 2;
                    }
                } else if (runtime.seconds() > 0.5) //Positions robot

                    if (squaredirection == 1) { //Go left and...
                        if (ods1.getLightDetected() > ods2.getLightDetected()) { //Rotate clockwise
                            m1.setPower(-0.2 + 0.15 + 0.05); //Possibly add forwards
                            m2.setPower(0.2 - 0.15 + 0.05);
                            m3.setPower(0.2 + 0.15 + 0.05);
                            m4.setPower(-0.2 - 0.15 + 0.05);
                        } else { //Rotate counterclockwise
                            m1.setPower(-0.2 - 0.15 + 0.05);
                            m2.setPower(0.2 + 0.15 + 0.05);
                            m3.setPower(0.2 - 0.15 + 0.05);
                            m4.setPower(-0.2 + 0.15 + 0.05);
                        }
                    } else if (squaredirection == 2) { //Go right and...
                        if (ods1.getLightDetected() > ods2.getLightDetected()) { //Rotate clockwise
                            m1.setPower(0.2 + 0.15 + 0.05);
                            m2.setPower(-0.2 - 0.15 + 0.05);
                            m3.setPower(-0.2 + 0.15 + 0.05);
                            m4.setPower(0.2 - 0.15 + 0.05);
                        } else { //Rotate counterclockwise
                            m1.setPower(0.2 - 0.15 + 0.05);
                            m2.setPower(-0.2 + 0.15 + 0.05);
                            m3.setPower(-0.2 - 0.15 + 0.05);
                            m4.setPower(0.2 + 0.15 + 0.05);
                        }
                    }

            case FORWARDS: //Beginning of the case statement FORWARD

                if (runtime.seconds() > 0.5) { //Runs for 2.5 seconds
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset(); //Resets the runtime
                    CURRENT_STEP = steps.GRAB; //Changes step to DROP
                    break; //Exits switch statement
                }
                m1.setPower(0.1);
                m2.setPower(0.1);
                m3.setPower(0.1);
                m4.setPower(0.1); //Drive forward without using gyro
                break; //Exits switch statement

            case GRAB:

                if (runtime.seconds() > .5) {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset(); //Resets the runtime
                    CURRENT_STEP = steps.RAISE; //Changes step to BACK2
                }
                s1.setPosition(0);
                break;

            case SLIGHTRAISE:

                if (runtime.seconds() > 2) {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset(); //Resets the runtime
                    s3.setPosition(0.51);
                    CURRENT_STEP = steps.SLIGHTBACK; //Changes step to BACK2
                }
                s3.setPosition(0);
                break;

            case SLIGHTBACK:

                if (runtime.seconds() > 0.5) {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset(); //Resets the runtime
                    CURRENT_STEP = steps.RAISE; //Changes step to BACK2
                }
                m1.setPower(-.2);
                m2.setPower(-.2);
                m3.setPower(-.2);
                m4.setPower(-.2); //Drives forward without using gyro
                s3.setPosition(0.51);
                break; //Exits switch statement

            case RAISE:

                if (runtime.seconds() > 7) {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset(); //Resets the runtime
                    s3.setPosition(0.51);
                    CURRENT_STEP = steps.RETURN; //Changes step to BACK2
                }
                s3.setPosition(0);
                break;

            case RETURN:

                if (runtime.seconds() > .75) {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset(); //Resets the runtime
                    CURRENT_STEP = steps.ROTATION3; //Changes step to BACK2
                }
                m1.setPower(-.2);
                m2.setPower(-.2);
                m3.setPower(-.2);
                m4.setPower(-.2); //Drives forward without using gyro
                s3.setPosition(0.51);
                break; //Exits switch statement

            case ROTATION3:

                target = 92;
                speed = (2 / (1 + Math.pow(Math.E, -.015 * (target - modernRoboticsI2cGyro.getIntegratedZValue())))) - 1;
                if (modernRoboticsI2cGyro.getIntegratedZValue() > target - 2 && modernRoboticsI2cGyro.getIntegratedZValue() < target + 2) {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset();
                    CURRENT_STEP = steps.FORWARD3;
                    break;
                }
                m1.setPower(-speed);
                m2.setPower(speed);
                m3.setPower(-speed);
                m4.setPower(speed);
                break;

            case FORWARD3:

                if (runtime.seconds() > 2.5) { //Runs for 2.5 seconds
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset(); //Resets the runtime
                    CURRENT_STEP = steps.DROP2; //Changes step to DROP
                    break; //Exits switch statement
                }
                m1.setPower(0.1);
                m2.setPower(0.1);
                m3.setPower(0.1);
                m4.setPower(0.1); //Drive forward without using gyro
                break; //Exits switch statement

            case DROP2:

                s1.setPosition(1); //Opens crunch servo
                m1.setPower(0);
                m2.setPower(0);
                m3.setPower(0);
                m4.setPower(0);
                runtime.reset(); //Resets the runtime
                CURRENT_STEP = steps.BACK3; //Changes step to BACK
                break; //Exits switch statement

            case BACK3:

                if (runtime.seconds() > .5) { //Runs for 2.5 seconds
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset(); //Resets the runtime
                    CURRENT_STEP = steps.STOP; //Changes step to DROP
                    break; //Exits switch statement
                }
                m1.setPower(-.1);
                m2.setPower(-.1);
                m3.setPower(-.1);
                m4.setPower(-.1); //Drive forward without using gyro
                break; //Exits switch statement

            case STOP:

                m1.setPower(0);
                m2.setPower(0);
                m3.setPower(0);
                m4.setPower(0);
                break;

        }
    }
}