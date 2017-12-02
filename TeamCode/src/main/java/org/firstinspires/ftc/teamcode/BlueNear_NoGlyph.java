//BlueNear_NoGlyph
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
//Imports

@Autonomous(name = "BlueNear_NoGlyph", group = "Autonomous") //Names program and declares it autonomous
public class BlueNear_NoGlyph extends AutoSteps {

    public static final String TAG = "BlueNear_NoGlyph";

    /** Front Left Motor, gearbox 40 */
    DcMotor m1;

    /** Front Right Motor, gearbox 40 */
    DcMotor m2;

    /** Back Right Motor, gearbox 40 */
    DcMotor m3;

    /** Back Left Motor, gearbox 40 */
    DcMotor m4;

    /** Arm Raise Motor, gearbox 60 */
    DcMotor m5;

    /** Arm Rotating Base Motor, gearbox 60 */
    DcMotor m6;

    /** Jewel Arm Servo, 190 degrees*/
    Servo s1; //Color sensor arm servo

    /** 1st Claw Grip Servo, 190 degrees */
    Servo s2; //Claw grip servo

    /** Wrist Rotation Servo, 190 degrees, extended mode */
    Servo s3; //Wrist rotation

    /** Claw Vertical Servo, continuous */
    Servo s4; //Claw vertical

    /** 2nd Claw Grip Servo, 190 degrees */
    Servo s5; //Second claw grip

    /** Arm Extension Servo, continuous */
    Servo s6; //Arm extension

    /** Time Variable, use runtime.seconds() */
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

    @Override
    public void runOpMode() { //Starts initiate loop

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

        m1.setDirection(DcMotor.Direction.REVERSE); //Sets m1 to reverse mode
        m3.setDirection(DcMotor.Direction.REVERSE); //Sets m3 to reverse mode

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
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
        checkTime = new ElapsedTime(); //Creates runtime variable for using time

        s1.setPosition(0); //Pulls jewel appendage against side of robot
        s2.setPosition(0); //Opens 1st gripper
        s3.setPosition(0.45); //Sets wrist rotation to be perpendicular to robot *NOT USED*
        s4.setPosition(0.52); //Sets wrist vertical to not move
        s5.setPosition(0); //Opens 2nd gripper *NOT USED*
        s6.setPosition(0.5); //Sets arm extension to not move

        telemetry.addData("", "Press Play"); //Displays start message
        telemetry.update();

        steps CURRENT_STEP = steps.LOWERSERVO; //Sets the variable CURRENT_STEP to the LOWERSERVO

        double rangeCM4 = r4.getDistance(DistanceUnit.CM);

        double rCM1Curr = 0;
        double rCM4Curr = 0;

        waitForStart(); //Ends initialization

        double rCM4Prev = r4.getDistance(DistanceUnit.CM); //Defining variable used in low pass filter
        if (rCM4Prev > 255 || rCM4Prev < 0) {
            rCM4Prev = 0;
        }

        while (opModeIsActive()) { //Starts play loop

            rCM4Curr = r4.getDistance(DistanceUnit.CM); //Defining variable used in low pass filter
            if (rCM4Curr > 255 || rCM4Curr < 0) {
                rCM4Curr = rCM4Prev;
            }

            rangeCM4 = (rCM4Curr * 0.2) + (rCM4Prev * 0.8);
            rCM4Prev = rangeCM4;

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

                    m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                    m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                    m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                    m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving

                    s1.setPosition(0.55); //Sets jewel arm to raise outwards partially
                    runtime.reset(); //Resets runtime
                    CURRENT_STEP = steps.SENSECOLOR; //Changes steps to SENSECOLOR

                    break; //Exits switch statement

                case SENSECOLOR: //Beginning of the case statement

                    if (runtime.seconds() < .7) { //If less than .7 seconds
                        s4.setPosition(1); //Sets claw vertical to raise upwards
                    } else {
                        s4.setPosition(.52); //Sets claw vertical to stop
                    }

                    if (runtime.seconds() > .5) { //If less than .5 seconds
                        s1.setPosition(0.65); //Sets jewel arm to raise outwards fully
                    }

                    if (c1.blue() > 2 && runtime.seconds() > 1) { //If c1 sees blue
                        CURRENT_STEP = steps.KNOCKFORWARDS; //Changes steps to KNOCKFORWARDS
                        runtime.reset(); //Resets runtime
                        break; //Exits switch statement
                    }

                    if (c1.red() > 2 && runtime.seconds() > 1) { //If c1 sees red
                        CURRENT_STEP = steps.KNOCKBACK; //Changes steps to KNOCKFORWARDS
                        runtime.reset(); //Resets runtime
                        break; //Exits switch statement
                    }
                    break; //Exits switch statement

                case KNOCKBACK: //Beginning of the case statement

                    if (runtime.seconds() > 0.4) {
                        m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                        CURRENT_STEP = steps.KNOCKFORWARDS; //Changes steps to KNOCKFORWARDS
                        break; //Exits switch statement
                    }

                    m1.setPower(-0.1); //Sets motor 1 power to -.1 to drive backwards
                    m2.setPower(-0.1); //Sets motor 2 power to -.1 to drive backwards
                    m3.setPower(-0.1); //Sets motor 3 power to -.1 to drive backwards
                    m4.setPower(-0.1); //Sets motor 4 power to -.1 to drive backwards
                    s4.setPosition(.52); //Sets claw vertical to stop
                    break; //Exits switch statement

                case KNOCKFORWARDS: //Beginning of the case statement

                    if (runtime.seconds() > 0.5) {
                        m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                        CURRENT_STEP = steps.RAISESERVO; //Changes steps to RAISESERVO
                        break; //Exits switch statement
                    }
                    m1.setPower(0.1); //Sets motor 1 power to .1 to drive forwards
                    m2.setPower(0.1); //Sets motor 2 power to .1 to drive forwards
                    m3.setPower(0.1); //Sets motor 3 power to .1 to drive forwards
                    m4.setPower(0.1); //Sets motor 4 power to .1 to drive forwards
                    break; //Exits switch statement

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

                    if (rangeCM4 > 105) { //If more than 2.5 seconds
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

