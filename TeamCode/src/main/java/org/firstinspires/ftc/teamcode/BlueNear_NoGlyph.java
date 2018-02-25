//BlueNear
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
//Imports

@Autonomous(name = "BlueNear", group = "Autonomous") //Names program and declares it autonomous
//@Disabled
public class BlueNear_NoGlyph extends AutoSteps {

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
    Servo s5; //Second claw grip

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

    VuforiaLocalizer vuforia; //Image recognition

    @Override
    public void runOpMode() { //Starts initiate loop

        m1 = hardwareMap.dcMotor.get("m1"); //Sets motors in the config
        m2 = hardwareMap.dcMotor.get("m2");
        m3 = hardwareMap.dcMotor.get("m3");
        m4 = hardwareMap.dcMotor.get("m4");
        m5 = hardwareMap.dcMotor.get("m5");
        m6 = hardwareMap.dcMotor.get("m6");
        m7 = hardwareMap.dcMotor.get("m7");

        s1 = hardwareMap.servo.get("s1"); //Sets servos in the config
        s2 = hardwareMap.servo.get("s2");
        s3 = hardwareMap.servo.get("s3");
        s4 = hardwareMap.servo.get("s4");
        s5 = hardwareMap.servo.get("s5");
        s6 = hardwareMap.servo.get("s6");

        m1.setDirection(DcMotor.Direction.REVERSE); //Sets m1 to reverse mode
        m3.setDirection(DcMotor.Direction.REVERSE); //Sets m3 to reverse mode

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro"); //Configures gyro, port 6
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;

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

        s1.setPosition(0); //Pulls jewel appendage against side of robot
        s2.setPosition(1); //Closes relic claw
        s3.setPosition(0); //Sets arm crunch servo to open
        s4.setPosition(1); //Sets Arm Crunch servo to open
        s6.setPosition(0.5); //Sets arm extension to not move

        modernRoboticsI2cGyro.calibrate(); //Gyro calibration

        while (modernRoboticsI2cGyro.isCalibrating()) { //Adds telemetry for gyro calibration
            telemetry.addData("", "Gyro Calibrating. Please wait..."); //Adds telemetry
            telemetry.update(); //Updates telemetry
        }

        telemetry.addData("Vuforia Initialized. Press play.", ""); //Adds telemetry
        telemetry.update(); //Updates telemetry

        AutoSteps.steps CURRENT_STEP = AutoSteps.steps.LOWERSERVO; //Sets the variable CURRENT_STEP to the first step in the sequence

        float image = 0; //Initializes image variable to track scanned pictograph
        float straight = 0; //Initializes straight variable to set what is straight forward for the robot using gyro
        double turn = 0; //Initializes turn variable for gyro driving correction
        double speed = 0; //Initializes speed variable for exponential regression

        double rangeCM1 = r1.getDistance(DistanceUnit.CM); //Initializes rangeCM1 for range reading
        double rangeCM2 = r2.getDistance(DistanceUnit.CM); //Initializes rangeCM2 for range reading
        double rangeCM3 = r3.getDistance(DistanceUnit.CM); //Initializes rangeCM3 for range reading
        double rangeCM4 = r4.getDistance(DistanceUnit.CM); //Initializes rangeCM4 for range reading
        double rangePrevCM3 = 0;
        int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue(); //Gyro integratedZ value

        int pillarCount = 0;
        int correctCount = 0;

        waitForStart(); //Ends initialization

        double rCM4Prev = r4.getDistance(DistanceUnit.CM); //Defining variable used in low pass filter
        if (rCM4Prev > 255 || rCM4Prev < 0) {
            rCM4Prev = 0;
        }

        while (opModeIsActive()) { //Starts play loop

            ///////////////////////////
            //Telemetry for debugging//
            ///////////////////////////

            telemetry.addData("Step", CURRENT_STEP + "\nRuntime: " + runtime.seconds()); //Adds telemetry to debug
            telemetry.update(); //Updates telemetry with new information

            /////////////////////////////
            //Start of switch statement//
            /////////////////////////////

            switch (CURRENT_STEP) { //Beginning of the switch- this sets the current step to whatever CURRENT_STEP is set to

                ///////////////////////
                //START OF MAIN STEPS//
                ///////////////////////

                case LOWERSERVO: //Beginning of the case statement

                    if (runtime.seconds() > 0.3 && runtime.seconds() < 0.8) { //Activates motor to raise glyph
                        m7.setPower(-0.5);
                    } else {
                        m7.setPower(0);
                    }

                    if(runtime.seconds() > 0.8) {
                        runtime.reset(); //Resets the runtime
                        CURRENT_STEP = steps.SENSECOLOR; //Changes step to SENSECOLOR
                    }

                    s1.setPosition(0.55); //Sets servo 1 position to 0.55 (lowers jewel arm)
                    break; //Exits switch statement

                case SENSECOLOR: //Beginning of the case statement

                    if (c1.blue() > c1.red() && runtime.seconds() > 1) {
                        CURRENT_STEP = steps.KNOCKFORWARDS; //Changes step to KNOCKFORWARDS
                        runtime.reset(); //Resets the runtime
                        break; //Exits switch statement
                    }

                    if (c1.red() > c1.blue() && runtime.seconds() > 1) {
                        CURRENT_STEP = steps.KNOCKBACK   ; //Changes step to KNOCKBACK
                        runtime.reset(); //Resets the runtime
                        break; //Exits switch statement
                    }

                    if (c1.red() == c1.blue() && c1.red() == 0 && runtime.seconds() > 1) {
                        s1.setPosition(0); //Raises jewel arm
                        runtime.reset(); //Resets the runtime
                        CURRENT_STEP = steps.RAISESERVO; //Changes step to RAISESERVO
                    }

                    if (runtime.seconds() > .5) {
                        s1.setPosition(0.65); //Continues to lower jewel arm
                    }
                    break; //Exits switch statement

                case KNOCKBACK: //Beginning of the case statement

                    if(runtime.seconds() > 1){
                        m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                        runtime.reset();
                        CURRENT_STEP = steps.RAISESERVO; //Changes step to KNOCKFORWARDS
                        break; //Exits switch statement
                    }
                    if(runtime.seconds() < 0.3) {
                        m1.setPower(-0.15);
                        m2.setPower(0.15);
                        m3.setPower(-0.15);
                        m4.setPower(0.15);
                        break;
                    }
                    if(runtime.seconds() > 0.5 && runtime.seconds() < 0.9) {
                        s1.setPosition(0);
                        m1.setPower(0.15);
                        m2.setPower(-0.15);
                        m3.setPower(0.15);
                        m4.setPower(-0.15);
                        break;
                    }
                    break;

                case KNOCKFORWARDS: //Beginning of the case statement

                    if(runtime.seconds() > 1){
                        m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                        runtime.reset();
                        CURRENT_STEP = steps.RAISESERVO; //Changes step to KNOCKFORWARDS
                        break; //Exits switch statement
                    }
                    if(runtime.seconds() < 0.3) {
                        m1.setPower(0.15);
                        m2.setPower(-0.15);
                        m3.setPower(0.15);
                        m4.setPower(-0.15);
                        break;
                    }
                    if(runtime.seconds() > 0.5 && runtime.seconds() < 0.9) {
                        s1.setPosition(0);
                        m1.setPower(-0.15);
                        m2.setPower(0.15);
                        m3.setPower(-0.15);
                        m4.setPower(0.15);
                        break;
                    }
                    break;

                case RAISESERVO: //Beginning of the case statement

                    m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                    m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                    m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                    m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving

                    s1.setPosition(0); //Brings jewel appendage servo in
                    runtime.reset(); //Resets runtime
                    CURRENT_STEP = steps.PARK; //Changes steps to PARK
                    break; //Exits switch statement

                case PARK: //Beginning of the case statement

                    if (runtime.seconds() > 2.5) { //If more than 2.5 seconds
                        m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                        runtime.reset();
                        CURRENT_STEP = steps.STOP; //Changes steps to STOP
                    }
                    m1.setPower(.1); //Sets motor 1 power to -.1 to drive backwards
                    m2.setPower(.1); //Sets motor 2 power to -.1 to drive backwards
                    m3.setPower(.1); //Sets motor 3 power to -.1 to drive backwards
                    m4.setPower(.1); //Sets motor 4 power to -.1 to drive backwards
                    s1.setPosition(0);
                    break; //Exits switch statement

                case STOP: //Beginning of the case statement STOP

                    m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                    m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                    m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                    m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                    break; //Exits switch statement
            }
        }
    }
}

