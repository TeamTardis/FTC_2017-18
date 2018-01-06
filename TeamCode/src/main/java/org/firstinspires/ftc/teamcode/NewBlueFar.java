//RedFar
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
//Imports

@Autonomous(name = "NewBlueFar", group = "Autonomous") //Names program
public class NewBlueFar extends AutoSteps { //Creates class and extends program wih steps

    public static final String TAG = "NewBlueFar";

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

    double minPowerPos = 0.1; //Variables for minimum powers
    double minPowerNeg = -0.1;

    /**
     * Positive = Forward
     **/
    public void setDrivePower(double power, double turn) {

        //Unless power is 0, set power to at least some minimum
        if (power < minPowerPos && power > 0) {
            power = minPowerPos;
        } else if (power > minPowerNeg && power < 0) {
            power = minPowerNeg;
        }

        m1.setPower(power - turn); //Drives robot forwards or backwards, availability for turn variable
        m2.setPower(power + turn);
        m3.setPower(power - turn);
        m4.setPower(power + turn);
    }

    /**
     * Positive = Clockwise
     **/
    public void setRotationPower(double power) {

        //Unless power is 0, set power to at least some minimum
        if (power < minPowerPos && power > 0) {
            power = minPowerPos;
        } else if (power > minPowerNeg && power < 0) {
            power = minPowerNeg;
        }

        m1.setPower(power); //Drives robot to rotate
        m2.setPower(-power);
        m3.setPower(power);
        m4.setPower(-power);
    }

    /**
     * Positive = Right
     **/
    public void setStrafePower(double power, double turn) {

        //Unless power is 0, set power to at least some minimum
        if (power < minPowerPos && power > 0) {
            power = minPowerPos;
        } else if (power > minPowerNeg && power < 0) {
            power = minPowerNeg;
        }

        m1.setPower(power - turn); //Drives robot to strafe left or right, availability for turn variable
        m2.setPower(-power + turn);
        m3.setPower(-power - turn);
        m4.setPower(power + turn);
    }

    double rangeCheckClose = 25; //Variables for range check
    double rangeCheckFar = 40;

    public boolean columnRangeCheckNeeded(double rangeCM2) { //Checks to see if robot is too close or far from cryptobox
        return (rangeCM2 <= rangeCheckClose || rangeCM2 >= rangeCheckFar);
    }

    float leftposition = 58; //Variable for left column positioning
    float centerposition = 75; //Variable for center column positioning
    float rightposition = 93; //Variable for right column positioning

    double lefttolerance = 1.1; //Variables for column tolerances
    double centertolerance = 1.1;
    double righttolerance = 1.1;

    @Override
    public void runOpMode() { //Beginning of main loop

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

        telemetry.addData("", "Gyro Calibrated. Initializing Vuforia..."); //Adds telemetry
        telemetry.update(); //Updates telemetry

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                "id", hardwareMap.appContext.getPackageName()); //Shows camera on robot controller phone

        //Shows camera on robot controller phone
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "Aa9zuMz/////AAAAGTZYlyCroUG9ibBucmCtqD990SI/3cked3Q7Nnj4zhBoD72GU" +
                "WNlfPopl2pvrC3tFmeHYt/I1Rtxubiif5SptIs9SdCheiJZBLEIAOG4nizHWuIB5tU2yCjJoO7pNrn/+gn4y2LDg" +
                "bIQw7AVdp272CbVsp6E/e6zq7fA0Yelv6JN/nLROUo7FWJHz8G98/XL9d1poW9D5DlJ++j7ePbgPv+kSPqIhfiQ8" +
                "uEgtCTLUctdu2/n4M3J/fltvRe4ykRG1UczLleQJ5NMflog4rloogbngGNRTVg0DRV0YEQkbnJIcfBz9kDja0Miq" +
                "vcc6lwPfyBIo591XYi0hU7asnQ2boyWWQGXEdLMnxHQcW5YVmbG"; //License key

        //Sets parameters for vuforia
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; //Sets vuforia camera to back
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        relicTrackables.activate();

        telemetry.addData("Vuforia Initialized. Press play.", ""); //Adds telemetry
        telemetry.update(); //Updates telemetry

        steps CURRENT_STEP = steps.SCANIMAGE; //Sets the variable CURRENT_STEP to the first step in the sequence

        float image = 0; //Initializes image variable to track scanned pictograph
        float straight = 0; //Initializes straight variable to set what is straight forward for the robot using gyro
        double turn = 0; //Initializes turn variable for gyro driving correction
        double speed = 0; //Initializes speed variable for exponential regression

        double rangeCM1 = r1.getDistance(DistanceUnit.CM); //Initializes rangeCM1 for range reading
        double rangeCM2 = r2.getDistance(DistanceUnit.CM); //Initializes rangeCM2 for range reading
        double rangeCM3 = r3.getDistance(DistanceUnit.CM); //Initializes rangeCM3 for range reading
        double rangeCM4 = r4.getDistance(DistanceUnit.CM); //Initializes rangeCM4 for range reading
        int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue(); //Gyro integratedZ value

        waitForStart(); //Waits for start

        double rCM3Prev = r3.getDistance(DistanceUnit.CM); //Defining variable used in low pass filter
        if (rCM3Prev > 255 || rCM3Prev < 0) { //Error check
            rCM3Prev = 0;
        }
        double rCM2Prev = r2.getDistance(DistanceUnit.CM); //Defining variable used in low pass filter
        if (rCM2Prev > 255 || rCM2Prev < 0) { //Error check
            rCM2Prev = 0;
        }
        double rCM4Prev = r4.getDistance(DistanceUnit.CM); //Defining variable used in low pass filter
        if (rCM4Prev > 255 || rCM4Prev < 0) { //Error check
            rCM4Prev = 0;
        }

        double rCM3Curr = 0; //Initializes variable to track current range 1 reading
        double rCM2Curr = 0; //Initializes variable to track current range 2 reading
        double rCM4Curr = 0; //Initializes variable to track current range 4 reading

        runtime.reset();

        while (opModeIsActive()) { //Loop for op mode

            rCM3Curr = r3.getDistance(DistanceUnit.CM); //Defining variable used in low pass filter
            if (rCM3Curr > 255 || rCM3Curr < 0) {
                rCM3Curr = rCM3Prev;
            }
            rCM2Curr = r2.getDistance(DistanceUnit.CM); //Defining variable used in low pass filter
            if (rCM2Curr > 255 || rCM2Curr < 0) {
                rCM2Curr = rCM2Prev;
            }
            rCM4Curr = r4.getDistance(DistanceUnit.CM); //Defining variable used in low pass filter
            if (rCM4Curr > 255 || rCM4Curr < 0) {
                rCM4Curr = rCM4Prev;
            }
            rangeCM3 = (rCM3Curr * 0.2) + (rCM3Prev * 0.8); //Updates rangeCM3 variable with low pass filter
            rCM3Prev = rangeCM3; //Updates rCM3Prev variable with info current rangeCM3 variable
            rangeCM2 = (rCM2Curr * 0.2) + (rCM2Prev * 0.8); //Updates rangeCM2 variable with low pass filter
            rCM2Prev = rangeCM2; //Updates rCM2Prev variable with info current rangeCM2 variable
            rangeCM4 = (rCM4Curr * 0.2) + (rCM4Prev * 0.8); //Updates rangeCM4 variable with low pass filter
            rCM4Prev = rangeCM4; //Updates rCM4Prev variable with info current rangeCM4 variable
            integratedZ = modernRoboticsI2cGyro.getIntegratedZValue(); //Gyro integratedZ value

            ///////////////////////////
            //Telemetry for debugging//
            ///////////////////////////

            telemetry.addData("Step", CURRENT_STEP + "\nImage" + image + "\nColorBlue: " + c1.blue()
                    + "\nColorRed: " + c1.red() + "\nRange1: " + rangeCM1 + "\nRange2: " + rangeCM2
                    + "\nRange3: " + rangeCM3 + "\nRange4: " + rangeCM4 + "\nGyro: " + integratedZ
                    + "\nRuntime: " + runtime.seconds()); //Adds telemetry to debug
            telemetry.update(); //Updates telemetry with new information

            /////////////////////////////
            //Start of switch statement//
            /////////////////////////////

            switch (CURRENT_STEP) { //Beginning of the switch. this sets the current step to whatever CURRENT_STEP is set to

                ///////////////////////
                //START OF MAIN STEPS//
                ///////////////////////

                case SCANIMAGE: //Beginning of case statement SCANIMAGE

                    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate); //Image scanning

                    s3.setPosition(0.35); //Closes arm crunch
                    s4.setPosition(0.7);

                    if (runtime.seconds() < 0.5) { //Activates motor to raise glyph
                        m7.setPower(-0.5);
                    } else {
                        m7.setPower(0);
                    }

                    if (vuMark == RelicRecoveryVuMark.LEFT && runtime.seconds() > 1.5) { //Vuforia for left pictograph
                        image = 1;
                        CURRENT_STEP = steps.LOWERSERVO; //Changes step to LOWERSERVO
                        break; //Exits switch statement
                    }

                    if (vuMark == RelicRecoveryVuMark.CENTER && runtime.seconds() > 1.5) { //Vuforia for center pictograph
                        image = 2;
                        CURRENT_STEP = steps.LOWERSERVO; //Changes step to LOWERSERVO
                        break; //Exits switch statement
                    }

                    if (vuMark == RelicRecoveryVuMark.RIGHT && runtime.seconds() > 1.5) { //Vuforia for right pictograph
                        image = 3;
                        CURRENT_STEP = steps.LOWERSERVO; //Changes step to LOWERSERVO
                        break; //Exits switch statement
                    } else if (image == 0 && runtime.seconds() > 3) { //If we don't scan the image
                        image = 2;
                        CURRENT_STEP = steps.LOWERSERVO; //Changes step to LOWERSERVO
                    }
                    break; //Exits switch statement

                case LOWERSERVO: //Beginning of the case statement LOWERSERVO

                    setDrivePower(0, 0); //Stop robot

                    s1.setPosition(0.55); //Sets servo 1 position to 0.55 (lowers jewel arm)
                    runtime.reset(); //Resets the runtime
                    CURRENT_STEP = steps.SENSECOLOR; //Changes step to SENSECOLOR
                    break; //Exits switch statement

                case SENSECOLOR: //Beginning of case statement SENSECOLOR

                    if (c1.blue() > c1.red() && runtime.seconds() > 1) {
                        CURRENT_STEP = steps.KNOCKFORWARDS; //Changes step to KNOCKFORWARDS
                        runtime.reset(); //Resets the runtime
                        break; //Exits switch statement
                    }

                    if (c1.red() > c1.blue() && runtime.seconds() > 1) {
                        CURRENT_STEP = steps.KNOCKBACK; //Changes step to KNOCKBACK
                        runtime.reset(); //Resets the runtime
                        break; //Exits switch statement
                    }

                    if (c1.red() == c1.blue() && c1.red() == 0 && runtime.seconds() > 1) {
                        s1.setPosition(0); //Raises jewel arm
                        CURRENT_STEP = steps.RAISESERVO; //Changes step to RAISESERVO
                    }

                    if (runtime.seconds() > .5) {
                        s1.setPosition(0.65); //Continues to lower jewel arm
                    }
                    break; //Exits switch statement

                case KNOCKBACK: //Beginning of case statement KNOCKBACK

                    if (runtime.seconds() > 0.4) {
                        setDrivePower(0, 0);
                        CURRENT_STEP = steps.KNOCKFORWARDS; //Changes step to KNOCKFORWARDS
                        break; //Exits switch statement
                    }
                    setDrivePower(-0.1, 0); //Drive backward without using gyro
                    break; //Exits switch statement

                case KNOCKFORWARDS: //Beginning of case statement KNOCKFORWARDS

                    if (runtime.seconds() > 0.5) {
                        setDrivePower(0, 0); //Stops robot
                        CURRENT_STEP = steps.RAISESERVO; //Changes step to RAISESERVO
                        break; //Exits switch statement
                    }
                    setDrivePower(0.1, 0); //Drive forward without using gyro
                    break; //Exits switch statement

                case RAISESERVO: //Beginning of the case statement RAISESERVO

                    setDrivePower(0, 0); //Stops robot
                    s1.setPosition(0); //Sets servo 1 position to 0 (raises jewel arm)
                    runtime.reset(); //Resets the runtime
                    CURRENT_STEP = steps.DRIVETOCRYPTOBOX; //Changes step to DRIVETOCRYPTOBOX
                    break; //Exits switch statement

                case DRIVETOCRYPTOBOX: //Beginning of the case statement DRIVETOCRYPTOBOX

                    if (rangeCM2 < 60) { //Moves forward until wall
                        setDrivePower(0, 0); //Stops robot
                        runtime.reset(); //Resets the runtime
                        CURRENT_STEP = steps.BACKUP; //Changes step to ROTATE
                    }
                    setDrivePower(0.12, 0); //Drives backward without using gyro
                    s1.setPosition(0); //Sets servo 1 position to 0 (raises jewel arm)

                    break; //Exits switch statement

                case BACKUP: //Beginning of the case statement BACKUP

                    straight = 0; //Sets gyro variable to 0

                    if (integratedZ > straight) { //Changes turn variable
                        turn = .1;
                    } else if (integratedZ < straight) {
                        turn = -.1;
                    } else {
                        turn = 0;
                    }

                    if (rangeCM2 > 40) { //If at desired range
                        setDrivePower(0, turn); //Stop, adjust orientation
                        if (image == 1) {
                            CURRENT_STEP = steps.LEFTCOLUMN; //Changes step to LEFTCOLUMN
                            runtime.reset(); //Resets the runtime
                        }
                        if (image == 2) {
                            CURRENT_STEP = steps.CENTERCLOMUN; //Changes step to CENTERCLOMUN
                            runtime.reset(); //Resets the runtime
                        }
                        if (image == 3) {
                            CURRENT_STEP = steps.RIGHTCOLUMN; //Changes step to RIGHTCOLUMN
                            runtime.reset(); //Resets the runtime
                        }
                        break; //Exits switch statement
                    }
                    setDrivePower(-0.1, turn); //Drives backwards with gyro
                    break; //Exits switch statement

                case LEFTCOLUMN: //Beginning of the case statement LEFTCOLUMN

                    //Exponential regression equation to decrease speed as we approach target position
                    speed = ((Math.pow(rangeCM3 - leftposition, 2)) / 5000) + .18;

                    straight = 0; //Sets gyro variable to 0

                    if (integratedZ > straight) { //Changes turn variable
                        turn = .08;
                    } else if (integratedZ < straight) {
                        turn = -.08;
                    } else {
                        turn = 0;
                    }

                    if (columnRangeCheckNeeded(rangeCM2)) { //Checks range from cryptobox
                        CURRENT_STEP = steps.POSITIONCHECK;
                        break;
                    } else { //If in range

                        if (runtime.seconds() > 1 && rangeCM3 >= (leftposition - lefttolerance) && rangeCM3 <= (leftposition + lefttolerance) && integratedZ <= 2 && integratedZ >= -2) { //If in range and runtime is past 1 second
                            setDrivePower(0, 0); //Stops robot
                            CURRENT_STEP = steps.FORWARD; //Changes step to FORWARD
                            break; //Exits switch statement
                        } else if (rangeCM3 >= (leftposition - lefttolerance) && rangeCM3 <= (leftposition + lefttolerance)) { //If in range
                            if (integratedZ <= 2 && integratedZ >= -2) { //If in range and in angle
                                setDrivePower(0, 0); //Stops robot
                                break; //Exits switch statement
                            } else { //If in range but outside angle
                                setRotationPower(-turn); //Rotates robot
                                runtime.reset(); //Resets the runtime
                                break; //Exits switch statement
                            }
                        } else { //If outside range
                            if (rangeCM3 < (leftposition - lefttolerance)) { //If too close to wall
                                setStrafePower(speed, turn); //Strafes right and rotates
                                runtime.reset(); //Resets the runtime
                                break; //Exits switch statement
                            } else { //If too far from wall
                                setStrafePower(-0.23, turn); //Strafes left and rotates
                                runtime.reset(); //Resets the runtime
                                break; //Exits switch statement
                            }
                        }
                    }

                case CENTERCLOMUN: //Beginning of the case statement CENTERCLOMUN

                    //Exponential regression equation to decrease speed as we approach target position
                    speed = ((Math.pow(rangeCM3 - (centerposition + 15), 2)) / 5000) + .10;

                    straight = 0; //Sets gyro variable to 0

                    if (integratedZ > straight) { //Changes turn variable
                        turn = .08;
                    } else if (integratedZ < straight) {
                        turn = -.08;
                    } else {
                        turn = 0;
                    }

                    if (columnRangeCheckNeeded(rangeCM2)) { //Checks range from cryptobox
                        CURRENT_STEP = steps.POSITIONCHECK;
                        break;
                    } else { //If in range

                        if (runtime.seconds() > 1 && rangeCM3 >= (centerposition - centertolerance) && rangeCM3 <= (centerposition + centertolerance) && integratedZ <= 2
                                && integratedZ >= -2) { //If checkPosition runtime is past 1 second
                            setDrivePower(0, 0); //Stops robot
                            CURRENT_STEP = steps.FORWARD; //Changes step to FORWARD
                            break; //Exits switch statement
                        } else if (rangeCM3 >= (centerposition - centertolerance) && rangeCM3 <= (centerposition + centertolerance)) { //If in range
                            if (integratedZ <= 2 && integratedZ >= -2) { //If in range and in angle
                                setDrivePower(0, 0); //Stops robot
                                break; //Exits switch statement
                            } else { //If in range but outside angle
                                setRotationPower(-turn); //Rotates robot
                                runtime.reset(); //Resets the runtime
                                break; //Exits switch statement
                            }
                        } else { //If outside range
                            if (rangeCM3 < (centerposition - centertolerance)) { //If too close to wall
                                setStrafePower(speed, turn); //Strafes right and rotates
                                runtime.reset(); //Resets the runtime
                                break; //Exits switch statement
                            } else { //If too far from wall
                                setStrafePower(-0.24, turn); //Strafes left and rotates
                                runtime.reset(); //Resets the runtime
                                break; //Exits switch statement
                            }
                        }
                    }
                case RIGHTCOLUMN: //Beginning of the case statement RIGHTCOLUMN

                    //Exponential regression equation to decrease speed as we approach target position
                    speed = ((Math.pow(rangeCM3 - (rightposition + 5), 2)) / 5000) + .1;

                    straight = 0; //Sets gyro variable to 0

                    if (integratedZ > straight) { //Changes turn variable
                        turn = .08;
                    } else if (integratedZ < straight) {
                        turn = -.08;
                    } else {
                        turn = 0;
                    }

                    if (columnRangeCheckNeeded(rangeCM2)) { //Checks range from cryptobox
                        CURRENT_STEP = steps.POSITIONCHECK;
                        break;
                    } else { //If in range

                        if (runtime.seconds() > 1 && rangeCM3 >= (rightposition - righttolerance) && rangeCM3 <= (rightposition + righttolerance) && integratedZ <= 2
                                && integratedZ >= -2) { //If checkPosition runtime is past 1 second
                            setDrivePower(0, 0); //Stops robot
                            CURRENT_STEP = steps.FORWARD; //Changes step to FORWARD
                            break; //Exits switch statement
                        } else if (rangeCM3 >= (rightposition - righttolerance) && rangeCM3 <= (rightposition + righttolerance)) { //If in range
                            if (integratedZ <= 2 && integratedZ >= -2) { //If in range and in angle
                                setDrivePower(0, 0); //Stops robot
                                break; //Exits switch statement
                            } else { //If in range but outside angle
                                setRotationPower(-turn); //Rotates robot
                                runtime.reset(); //Resets the runtime
                                break; //Exits switch statement
                            }
                        } else { //If outside range
                            if (rangeCM3 < (rightposition - righttolerance)) { //If too close to wall
                                setStrafePower(speed, turn); //Strafes right and rotates
                                runtime.reset(); //Resets the runtime
                                break; //Exits switch statement
                            } else { //If too far from wall
                                setStrafePower(-0.23, turn); //Strafes left and rotates
                                runtime.reset(); //Resets the runtime
                                break; //Exits switch statement
                            }
                        }
                    }

                case POSITIONCHECK: //Beginning of the case statement POSITIONCHECK

                    if (!columnRangeCheckNeeded(rangeCM2)) { //Checks if robot is too close or far from cryptobox
                        setDrivePower(0, 0); //Stops robot
                        if (image == 1) {
                            CURRENT_STEP = steps.LEFTCOLUMN; //Changes step to LEFTCOLUMN
                            runtime.reset(); //Resets the runtime
                        }
                        if (image == 2) {
                            CURRENT_STEP = steps.CENTERCLOMUN; //Changes step to CENTERCLOMUN
                            runtime.reset(); //Resets the runtime
                        }
                        if (image == 3) {
                            CURRENT_STEP = steps.RIGHTCOLUMN; //Changes step to RIGHTCOLUMN
                            runtime.reset(); //Resets the runtime
                        }
                        break; //Exits switch statement
                    }

                    if (rangeCM2 < rangeCheckClose) { //If robot is too close
                        setDrivePower(-0.1, 0); //Drive backward without using gyro
                    } else { //If robot is too far
                        setDrivePower(0.1, 0); //Drive forward without using gyro
                    }

                    break; //Exits switch statement

                case FORWARD: //Beginning of the case statement FORWARD

                    if (runtime.seconds() > 2.5) { //Runs for 2.5 seconds
                        setDrivePower(0, 0); //Stops robot
                        runtime.reset(); //Resets the runtime
                        CURRENT_STEP = steps.DROP; //Changes step to DROP
                        break; //Exits switch statement
                    }
                    setDrivePower(0.1, 0); //Drive forward without using gyro
                    break; //Exits switch statement

                case DROP: //Beginning of the case statement DROP

                    s3.setPosition(0); //Opens left arm crunch servo
                    s4.setPosition(1); //Opens right arm crunch servo
                    setDrivePower(0, 0); //Stops robot
                    runtime.reset(); //Resets the runtime
                    CURRENT_STEP = steps.BACK; //Changes step to BACK
                    break; //Exits switch statement

                case BACK: //Beginning of the case statement BACK

                    if (runtime.seconds() > 0.5) { //Runs for 0.5 seconds
                        setDrivePower(0, 0); //Stops robot
                        runtime.reset(); //Resets the runtime
                        CURRENT_STEP = steps.FORWARD2; //Changes step to FORWARD2
                        break; //Exits switch statement
                    }
                    setDrivePower(-0.1, 0); //Drives robot backwards without using gyro
                    break; //Exits switch statement

                case FORWARD2: //Beginning of the case statement FORWARD2

                    if (runtime.seconds() > 1) { //Runs for 1 second
                        setDrivePower(0, 0); //Stops robot
                        runtime.reset(); //Resets the runtime
                        CURRENT_STEP = steps.BACK2; //Changes step to BACK2
                        break; //Exits switch statement
                    }
                    setDrivePower(0.2, 0); //Drives forward without using gyro
                    break; //Exits switch statement

                case BACK2: //Beginning of the case statement BACK2

                    if (runtime.seconds() > 0.5) { //Runs for 0.5 seconds
                        setDrivePower(0, 0); //Stops robot
                        runtime.reset(); //Resets the runtime
                        CURRENT_STEP = steps.STOP; //Changes step to STOP
                        break; //Exits switch statement
                    }
                    setDrivePower(-0.1, 0); //Drives backward without using gyro
                    break; //Exits switch statement

                case STOP: //Beginning of the case statement STOP
                    setDrivePower(0, 0); //Stops robot
                    break; //Exits switch statement
            }
        }
    }
} //Ends all loops