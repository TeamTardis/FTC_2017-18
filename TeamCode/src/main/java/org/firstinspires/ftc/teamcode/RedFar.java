package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//Imports

@Autonomous(name = "RedFar", group = "Autonomous")
public class RedFar extends AutoSteps {

    public static final String TAG = "RedFar";


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
    ElapsedTime checkTime;

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

    @Override
    public void runOpMode() {


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

        s1.setPosition(0);
        //s2.setPosition(1);
        s3.setPosition(0.45);
        s4.setPosition(0.52);
        s5.setPosition(0);
        s6.setPosition(0.5);


        modernRoboticsI2cGyro.calibrate();

        while (modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("", "Gyro Calibrating. Please wait...");
            telemetry.update();
        }
        telemetry.addData("", "Gyro Calibrated. Initializing Vuforia...");
        telemetry.update();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                "id", hardwareMap.appContext.getPackageName()); //Shows camera on robot controller phone

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId); //Shows camera on robot controller phone

        // Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "Aa9zuMz/////AAAAGTZYlyCroUG9ibBucmCtqD990SI/3cked3Q7Nnj4zhBoD72GU" +
                "WNlfPopl2pvrC3tFmeHYt/I1Rtxubiif5SptIs9SdCheiJZBLEIAOG4nizHWuIB5tU2yCjJoO7pNrn/+gn4y2LDg" +
                "bIQw7AVdp272CbVsp6E/e6zq7fA0Yelv6JN/nLROUo7FWJHz8G98/XL9d1poW9D5DlJ++j7ePbgPv+kSPqIhfiQ8" +
                "uEgtCTLUctdu2/n4M3J/fltvRe4ykRG1UczLleQJ5NMflog4rloogbngGNRTVg0DRV0YEQkbnJIcfBz9kDja0Miq" +
                "vcc6lwPfyBIo591XYi0hU7asnQ2boyWWQGXEdLMnxHQcW5YVmbG"; //License key

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();

        telemetry.addData(">", "Vuforia Initialized. Press play.");
        telemetry.update();

        steps CURRENT_STEP = steps.SCANIMAGE; //Sets the variable CURRENT_STEP to the first step in the sequence

        float image = 0;
        float straight = 0;
        double turn = 0;
        double speed = 0;

        double rangeCM1 = r1.getDistance(DistanceUnit.CM);
        double rangeCM2 = r2.getDistance(DistanceUnit.CM);
        double rangeCM3 = r3.getDistance(DistanceUnit.CM);
        double rangeCM4 = r4.getDistance(DistanceUnit.CM);
        int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue(); //Gyro integratedZ value

        waitForStart();

        boolean checkPosition = false;
        boolean inPosition = false;

        double rCM1Prev = r1.getDistance(DistanceUnit.CM); //Defining variable used in low pass filter
        if (rCM1Prev > 255 || rCM1Prev < 0) {
            rCM1Prev = 0;
        }
        double rCM4Prev = r4.getDistance(DistanceUnit.CM); //Defining variable used in low pass filter
        if (rCM4Prev > 255 || rCM4Prev < 0) {
            rCM4Prev = 0;
        }

        double rCM1Curr = 0;
        double rCM4Curr = 0;

        while (opModeIsActive()) {

            rCM1Curr = r1.getDistance(DistanceUnit.CM); //Defining variable used in low pass filter
            if (rCM1Curr > 255 || rCM1Curr < 0) {
                rCM1Curr = rCM1Prev;
            }
            rCM1Curr = r4.getDistance(DistanceUnit.CM); //Defining variable used in low pass filter
            if (rCM4Curr > 255 || rCM4Curr < 0) {
                rCM4Curr = rCM4Prev;
            }
            rangeCM1 = (rCM1Curr * 0.2) + (rCM1Prev * 0.8);
            rCM1Prev = rangeCM1;
            rangeCM4 = (rCM4Curr * 0.2) + (rCM4Prev * 0.8);
            rCM4Prev = rangeCM4;
            rangeCM2 = r2.getDistance(DistanceUnit.CM);
            rangeCM3 = r3.getDistance(DistanceUnit.CM);
            rangeCM4 = r4.getDistance(DistanceUnit.CM);
            integratedZ = modernRoboticsI2cGyro.getIntegratedZValue(); //Gyro integratedZ value


            ///////////////////////////
            //Telemetry for debugging//
            ///////////////////////////

            telemetry.addData("Step", CURRENT_STEP + "\nImage" + image + "\nColorBlue: " + c1.blue()
                    + "\nColorRed: " + c1.red() + "\nRange1: " + rangeCM1 + "\nRange2: " + rangeCM2
                    + "\nRange3: " + rangeCM3 + "\nRange4: " + rangeCM4 + "\nGyro: " + integratedZ + "\nRuntime: " + runtime.seconds() ); //Adds telemetry to debug
            telemetry.update(); //Updates telemetry with new information

            /////////////////////////////
            //Start of switch statement//
            /////////////////////////////

            switch (CURRENT_STEP) { //Beginning of the switch- this sets the current step to whatever CURRENT_STEP is set to

                ///////////////////////
                //START OF MAIN STEPS//
                ///////////////////////


                case SCANIMAGE: //Beginning of case statement START_RESET

                    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

                    s2.setPosition(0);

                    if (vuMark == RelicRecoveryVuMark.LEFT) {
                        image = 1;
                        CURRENT_STEP = steps.LOWERSERVO;
                        break; //Exits switch statement
                    }

                    if (vuMark == RelicRecoveryVuMark.CENTER) {
                        image = 2;
                        CURRENT_STEP = steps.LOWERSERVO;
                        break; //Exits switch statement
                    }

                    if (vuMark == RelicRecoveryVuMark.RIGHT) {
                        image = 3;
                        CURRENT_STEP = steps.LOWERSERVO;
                        break; //Exits switch statement
                    }
                    break;

                case LOWERSERVO: //Beginning of the case statement

                    m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                    m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                    m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                    m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving

                    s1.setPosition(0.55);
                    runtime.reset();
                    CURRENT_STEP = steps.SENSECOLOR;

                    break; //Exits switch statement

                case SENSECOLOR: //Beginning of case statement START_RESET

                    if (runtime.seconds() < .7) {
                        s4.setPosition(1);
                    } else {
                        s4.setPosition(.52);
                    }

                    if (runtime.seconds() > .5) {
                        s1.setPosition(0.65);
                    }

                    if (c1.blue() > 2 && runtime.seconds() > 1) {
                        CURRENT_STEP = steps.KNOCKFORWARDS;
                        runtime.reset();
                        break; //Exits switch statement
                    }

                    if (c1.red() > 2 && runtime.seconds() > 1) {
                        CURRENT_STEP = steps.KNOCKBACK;
                        runtime.reset();
                        break; //Exits switch statement
                    }
                    break;

                case KNOCKBACK: //Beginning of case statement START_RESET

                    if (runtime.seconds() > 0.4) {
                        m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                        CURRENT_STEP = steps.KNOCKFORWARDS;
                        break; //Exits switch statement
                    }

                    m1.setPower(0.1);
                    m2.setPower(0.1);
                    m3.setPower(0.1);
                    m4.setPower(0.1);
                    s4.setPosition(.52);
                    break;

                case KNOCKFORWARDS: //Beginning of case statement START_RESET

                    if (runtime.seconds() > 0.5) {
                        m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                        CURRENT_STEP = steps.RAISESERVO;
                        break; //Exits switch statement
                    }
                    m1.setPower(-0.1);
                    m2.setPower(-0.1);
                    m3.setPower(-0.1);
                    m4.setPower(-0.1);
                    break;

                case RAISESERVO: //Beginning of the case statement

                    m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                    m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                    m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                    m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving

                    s1.setPosition(0);
                    runtime.reset();
                    CURRENT_STEP = steps.DRIVETOCRYPTOBOX;

                    break; //Exits switch statement

                case DRIVETOCRYPTOBOX: //Beginning of the case statement

                    if (rangeCM4 < 80) {
                        m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                        runtime.reset();
                        CURRENT_STEP = steps.ROTATE;
                    }
                    m1.setPower(-.1); //Sets motor 1 power to 0 to make sure it is not moving
                    m2.setPower(-.1); //Sets motor 2 power to 0 to make sure it is not moving
                    m3.setPower(-.1); //Sets motor 3 power to 0 to make sure it is not moving
                    m4.setPower(-.1); //Sets motor 4 power to 0 to make sure it is not moving
                    s1.setPosition(0);
                    break; //Exits switch statement

                case ROTATE:

                    if (integratedZ < -140) {
                        m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                        if (image == 1) {
                            CURRENT_STEP = steps.LEFTCOLUMN;
                            runtime.reset();
                        }
                        if (image == 2) {
                            CURRENT_STEP = steps.CENTERCLOMUN;
                            runtime.reset();
                        }
                        if (image == 3) {
                            CURRENT_STEP = steps.RIGHTCOLUMN;
                            runtime.reset();
                        }
                        break;
                    }
                    m1.setPower(-0.2); //Sets motor 1 power to 0 to make sure it is not moving
                    m2.setPower(0.2); //Sets motor 2 power to 0 to make sure it is not moving
                    m3.setPower(-0.2); //Sets motor 3 power to 0 to make sure it is not moving
                    m4.setPower(0.2); //Sets motor 4 power to 0 to make sure it is not moving
                    s1.setPosition(0);
                    break; //Exits switch statement

                case LEFTCOLUMN: //Beginning of the case statement

                    speed = (Math.pow(0.9841381234, rangeCM1)) * 0.9637778146;  //Exponential regression equation to decrease speed as we approach target position

                    straight = -180; //Sets gyro variable to -180

                    if (integratedZ > straight) { //Checks to see if the robot is moving and the value of straight is greater than the value integratedZ
                        turn = .05; //Sets the turn value to .05
                    } else if (integratedZ < straight) { //Checks to see if the robot is moving and the value of straight is less than the value integratedZ
                        turn = -.05; //Sets the turn value to -.05
                    } else { //Default value (robot is not moving)
                        turn = 0; //Sets the turn value to 0
                    } //End of else statement

                    if (rangeCM1 == 86) { //STOP
                        m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                        runtime.reset();
                        CURRENT_STEP = steps.FORWARD;
                        break;
                    }

                    if (rangeCM1 < 86) { //LESS THAN (left)
                        m1.setPower(-speed - turn); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(speed + turn); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(speed - turn); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(-speed + turn); //Sets motor 4 power to 0 to make sure it is not moving
                        break; //Exits switch statement
                    }

                    if (rangeCM1 > 86) {
                        m1.setPower(0.2 - turn); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(-0.2 + turn); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(-0.2 - turn); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0.2 + turn); //Sets motor 4 power to 0 to make sure it is not moving
                        break; //Exits switch statement
                    }
                    break;

                case CENTERCLOMUN: //Beginning of the case statement STOP

                    speed = (Math.pow(0.9799308653, rangeCM1)) * .9; //Exponential regression equation to decrease speed as we approach target position

                    straight = -180; //Sets gyro variable to -180

                    if (integratedZ > straight) { //Checks to see if the robot is moving and the value of straight is greater than the value integratedZ
                        turn = .05; //Sets the turn value to .05
                    } else if (integratedZ < straight) { //Checks to see if the robot is moving and the value of straight is less than the value integratedZ
                        turn = -.05; //Sets the turn value to -.05
                    } else { //Default value (robot is not moving)
                        turn = 0; //Sets the turn value to 0
                    } //End of else statement

                    if (rangeCM1 == 70) { //STOP
                        m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                        runtime.reset();
                        CURRENT_STEP = steps.FORWARD;
                        break;
                    }

                    if (rangeCM1 < 70) { //LESS THAN (left)
                        m1.setPower(-speed - turn); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(speed + turn); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(speed - turn); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(-speed + turn); //Sets motor 4 power to 0 to make sure it is not moving
                        break; //Exits switch statement
                    }

                    if (rangeCM1 > 70) {
                        m1.setPower(0.2 - turn); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(-0.2 + turn); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(-0.2 - turn); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0.2 + turn); //Sets motor 4 power to 0 to make sure it is not moving
                        break; //Exits switch statement
                    }
                    break; //Exits switch statement

                case RIGHTCOLUMN: //Beginning of the case statement STOP

                    speed = (Math.pow(0.9852338678, rangeCM1) * 0.4); //Exponential regression equation to decrease speed as we approach target position

                    straight = -180; //Sets gyro variable to -180

                    if (integratedZ > straight) { //Checks to see if the robot is moving and the value of straight is greater than the value integratedZ
                        turn = .05; //Sets the turn value to .05
                    } else if (integratedZ < straight) { //Checks to see if the robot is moving and the value of straight is less than the value integratedZ
                        turn = -.05; //Sets the turn value to -.05
                    } else { //Default value (robot is not moving)
                        turn = 0; //Sets the turn value to 0
                    } //End of else statement

                    if (runtime.seconds() > 1 && rangeCM1 >= 48 && rangeCM1 <= 50 && integratedZ <= -178
                            && integratedZ >= -182) { //If checkPosition runtime is past 1 second
                        m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                        m6.setPower(0.05); //Sets arm base motor to hold against battery mount, turns left
                        CURRENT_STEP = steps.FORWARD;
                        break;
                    }
                    else if (rangeCM1 >= 48 && rangeCM1 <= 50) { //If in range
//                        if (checkPosition) { //If check position is true
//                            runtime.reset();
//                            checkPosition = false;
                          if (integratedZ <= -178 && integratedZ >= -182) { //If in range and in angle
                            m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                            m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                            m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                            m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                            m6.setPower(0.05); //Sets arm base motor to hold against battery mount, turns left
                            break;
                        } else { //If in range but outside angle
                            m1.setPower(-turn * 3); //Sets motor 1 power to 0 to make sure it is not moving
                            m2.setPower(turn * 3); //Sets motor 2 power to 0 to make sure it is not moving
                            m3.setPower(-turn * 3); //Sets motor 3 power to 0 to make sure it is not moving
                            m4.setPower(turn * 3); //Sets motor 4 power to 0 to make sure it is not moving
                            m6.setPower(0.05); //Sets arm base motor to hold against battery mount, turns left

                              runtime.reset();
                            break; //Exits switch statement
                        }
                    } else { //If outside range
                        if (rangeCM1 < 48) { //If too close to wall
                            m1.setPower(-speed - turn); //Sets motor 1 power to 0 to make sure it is not moving
                            m2.setPower(speed + turn); //Sets motor 2 power to 0 to make sure it is not moving
                            m3.setPower(speed - turn); //Sets motor 3 power to 0 to make sure it is not moving
                            m4.setPower(-speed + turn); //Sets motor 4 power to 0 to make sure it is not moving
                            m6.setPower(0.05); //Sets arm base motor to hold against battery mount, turns left
                            runtime.reset();
                            break; //Exits switch statement
                        } else { //If too far from wall
                            m1.setPower(0.18 - turn); //Sets motor 1 power to 0 to make sure it is not moving
                            m2.setPower(-0.18 + turn); //Sets motor 2 power to 0 to make sure it is not moving
                            m3.setPower(-0.18 - turn); //Sets motor 3 power to 0 to make sure it is not moving
                            m4.setPower(0.18 + turn); //Sets motor 4 power to 0 to make sure it is not moving
                            m6.setPower(0.05); //Sets arm base motor to hold against battery mount, turns left
                            runtime.reset();
                            break; //Exits switch statement
                        }
                    }
//                    break; //Exits switch statement

                case FORWARD:

                    if (runtime.seconds() > 2) {
                        m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                        runtime.reset();
                        CURRENT_STEP = steps.DROP;
                        break; //Exits switch statement
                    }
                    m1.setPower(0.1);
                    m2.setPower(0.1);
                    m3.setPower(0.1);
                    m4.setPower(0.1);
                    break;

                case DROP:

                    s2.setPosition(1);
                    m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                    m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                    m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                    m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                    runtime.reset();
                    CURRENT_STEP = steps.BACK;
                    break;

                case BACK:

                    if (runtime.seconds() > 0.3) {
                        m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                        runtime.reset();
                        CURRENT_STEP = steps.FORWARD2;
                        break; //Exits switch statement
                    }
                    m1.setPower(-0.1);
                    m2.setPower(-0.1);
                    m3.setPower(-0.1);
                    m4.setPower(-0.1);
                    break;

                case FORWARD2:

                    if (runtime.seconds() > 0.5) {
                        m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                        runtime.reset();
                        CURRENT_STEP = steps.BACK2;
                        break; //Exits switch statement
                    }
                    m1.setPower(0.2);
                    m2.setPower(0.2);
                    m3.setPower(0.2);
                    m4.setPower(0.2);
                    break;

                case BACK2:

                    if (runtime.seconds() > 0.3) {
                        m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                        runtime.reset();
                        CURRENT_STEP = steps.STOP;
                        break; //Exits switch statement
                    }
                    m1.setPower(-0.1);
                    m2.setPower(-0.1);
                    m3.setPower(-0.1);
                    m4.setPower(-0.1);
                    break;

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

