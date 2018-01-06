//RedFar
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

@Autonomous(name = "NewRedFar", group = "Autonomous")
public class SensorTest extends AutoSteps {

    public static final String TAG = "NewRedFar";

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

    /**
     * Positive = Forward
     **/
    public void setDrivePower(double power, double turn) {
        m1.setPower(power - turn); //Sets motor 1 power to -0.1 to make the robot move backward
        m2.setPower(power + turn); //Sets motor 2 power to -0.1 to make the robot move backward
        m3.setPower(power - turn); //Sets motor 3 power to -0.1 to make the robot move backward
        m4.setPower(power + turn); //Sets motor 4 power to -0.1 to make the robot move backward
    }

    /**
     * Positive = Clockwise
     **/
    public void setRotationPower(double power) {
        m1.setPower(power); //Sets motor 1 power to -0.1 to make the robot move backward
        m2.setPower(-power); //Sets motor 2 power to -0.1 to make the robot move backward
        m3.setPower(power); //Sets motor 3 power to -0.1 to make the robot move backward
        m4.setPower(-power); //Sets motor 4 power to -0.1 to make the robot move backward
    }

    /**
     * Positive = Right
     **/
    public void setStrafePower(double power, double turn) {
        m1.setPower(power - turn); //Sets motor 1 power to -0.1 to make the robot move backward
        m2.setPower(-power + turn); //Sets motor 2 power to -0.1 to make the robot move backward
        m3.setPower(-power - turn); //Sets motor 3 power to -0.1 to make the robot move backward
        m4.setPower(power + turn); //Sets motor 4 power to -0.1 to make the robot move backward
    }

    double rangeCheckClose = 25;
    double rangeCheckFar = 35;

    public boolean columnRangeCheckNeeded(double rangeCM2) {
        return (rangeCM2 <= rangeCheckClose || rangeCM2 >= rangeCheckFar);
    }

    float leftposition = 85; //Variable for left column positioning
    float ceterposition = 71; //Variable for center column positioning
    float rightposition = 52; //Variable for right column positioning

    double lefttolerance = 1.1; //Variable for left column tolerance
    double centertolerance = 1.1; //Variable for center column tolerance
    double righttolerance = 1.1; //Variable for right column tolerance

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
        s2.setPosition(0);
        s3.setPosition(1); //Closes left arm crunch servo
        s4.setPosition(1); //Closes right arm crunch servo
        s5.setPosition(0);
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

        telemetry.addData(">", "Vuforia Initialized. Press play."); //Adds telemetry
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

        double rCM1Prev = r1.getDistance(DistanceUnit.CM); //Defining variable used in low pass filter
        if (rCM1Prev > 255 || rCM1Prev < 0) { //Error check
            rCM1Prev = 0;
        }
        double rCM2Prev = r2.getDistance(DistanceUnit.CM); //Defining variable used in low pass filter
        if (rCM2Prev > 255 || rCM2Prev < 0) { //Error check
            rCM2Prev = 0;
        }
        double rCM4Prev = r4.getDistance(DistanceUnit.CM); //Defining variable used in low pass filter
        if (rCM4Prev > 255 || rCM4Prev < 0) { //Error check
            rCM4Prev = 0;
        }

        double rCM1Curr = 0; //Initializes variable to track current range 1 reading
        double rCM2Curr = 0; //Initializes variable to track current range 2 reading
        double rCM4Curr = 0; //Initializes variable to track current range 4 reading

        while (opModeIsActive()) { //Loop for op mode

            rCM1Curr = r1.getDistance(DistanceUnit.CM); //Defining variable used in low pass filter
            if (rCM1Curr > 255 || rCM1Curr < 0) {
                rCM1Curr = rCM1Prev;
            }
            rCM2Curr = r2.getDistance(DistanceUnit.CM); //Defining variable used in low pass filter
            if (rCM2Curr > 255 || rCM2Curr < 0) {
                rCM2Curr = rCM2Prev;
            }
            rCM4Curr = r4.getDistance(DistanceUnit.CM); //Defining variable used in low pass filter
            if (rCM4Curr > 255 || rCM4Curr < 0) {
                rCM4Curr = rCM4Prev;
            }
            rangeCM1 = (rCM1Curr * 0.2) + (rCM1Prev * 0.8); //Updates rangeCM1 variable with low pass filter
            rCM1Prev = rangeCM1; //Updates rCM1Prev variable with info current rangeCM1 variable
            rangeCM2 = (rCM2Curr * 0.2) + (rCM2Prev * 0.8); //Updates rangeCM1 variable with low pass filter
            rCM2Prev = rangeCM2; //Updates rCM2Prev variable with info current rangeCM1 variable
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

            switch (CURRENT_STEP) { //Beginning of the switch- this sets the current step to whatever CURRENT_STEP is set to

                ///////////////////////
                //START OF MAIN STEPS//
                ///////////////////////

                case SCANIMAGE: //Beginning of case statement SCANIMAGE

                    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate); //Image scanning

                    s3.setPosition(1); //Closes left arm crunch servo
                    s4.setPosition(1); //Closes right arm crunch servo

                    if (vuMark == RelicRecoveryVuMark.LEFT) { //Vuforia for left pictograph
                        image = 1;
                        CURRENT_STEP = steps.LOWERSERVO; //Changes step to LOWERSERVO
                        break; //Exits switch statement
                    }

                    if (vuMark == RelicRecoveryVuMark.CENTER) { //Vuforia for center pictograph
                        image = 2;
                        CURRENT_STEP = steps.LOWERSERVO; //Changes step to LOWERSERVO
                        break; //Exits switch statement
                    }

                    if (vuMark == RelicRecoveryVuMark.RIGHT) { //Vuforia for right pictograph
                        image = 3;
                        CURRENT_STEP = steps.LOWERSERVO; //Changes step to LOWERSERVO
                        break; //Exits switch statement
                    } else if (image == 0 && runtime.seconds() > 3) { //If we don't scan the image
                        image = 2;
                        CURRENT_STEP = steps.LOWERSERVO; //Changes step to LOWERSERVO
                    }
                    break; //Exits switch statement

                case LOWERSERVO: //Beginning of the case statement LOWERSERVO

                    setDrivePower(0, 0);

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
                        s1.setPosition(0); //Sets servo 1 position to 0 (raises jewel arm)
                        CURRENT_STEP = steps.RAISESERVO;
                    }

                    if (runtime.seconds() < .4) { //Activates servo to raise glyph
                        m7.setPower(0.2);
                    } else {
                        m7.setPower(0);
                    }

                    if (runtime.seconds() > .5) {
                        s1.setPosition(0.65); //Sets servo 1 position to 0.65 (continues to lower jewel arm)
                    }
                    break; //Exits switch statement

                case KNOCKBACK: //Beginning of case statement KNOCKBACK

                    if (runtime.seconds() > 0.4) {
                        setDrivePower(0, 0);
                        CURRENT_STEP = steps.KNOCKFORWARDS; //Changes step to KNOCKFORWARDS
                        break; //Exits switch statement
                    }
                    setDrivePower(0.1, 0);
                    s4.setPosition(0.52); //Sets servo 4 position to 0.52 (stops glyph from lifting)
                    break; //Exits switch statement

                case KNOCKFORWARDS: //Beginning of case statement KNOCKFORWARDS

                    if (runtime.seconds() > 0.5) {
                        setDrivePower(0, 0);
                        CURRENT_STEP = steps.RAISESERVO; //Changes step to RAISESERVO
                        break; //Exits switch statement
                    }
                    setDrivePower(-0.1, 0);
                    break; //Exits switch statement

                case RAISESERVO: //Beginning of the case statement RAISESERVO

                    setDrivePower(0, 0);
                    s1.setPosition(0); //Sets servo 1 position to 0 (raises jewel arm)
                    runtime.reset(); //Resets the runtime
                    CURRENT_STEP = steps.DRIVETOCRYPTOBOX; //Changes step to DRIVETOCRYPTOBOX
                    break; //Exits switch statement

                case DRIVETOCRYPTOBOX: //Beginning of the case statement DRIVETOCRYPTOBOX

                    if (rangeCM4 < 55) { //Moves backward until wall
                        setDrivePower(0, 0);
                        runtime.reset(); //Resets the runtime
                        CURRENT_STEP = steps.ROTATE; //Changes step to ROTATE
                    }
                    setDrivePower(-0.1, 0);
                    s1.setPosition(0); //Sets servo 1 position to 0 (raises jewel arm)
                    break; //Exits switch statement

                case ROTATE: //Beginning of the case statement ROTATE

                    if (integratedZ < -140) { //Rotates robot to the left about 140 degrees
                        setDrivePower(0, 0);
                        runtime.reset(); //Resets the runtime
                        CURRENT_STEP = steps.BACKUP; //Changes step to BACKUP
                    }
                    setRotationPower(-0.18);
                    s1.setPosition(0); //Sets servo 1 position to 0 (raises jewel arm)
                    break; //Exits switch statement

                case BACKUP: //Beginning of the case statement BACKUP

                    straight = -180; //Sets gyro variable to -180

                    turn = (integratedZ - straight) / 50;

                    if (rangeCM2 > 30) { //Moves the robot backward for 0.6 seconds
                        setDrivePower(0, turn);
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
                    setDrivePower(-0.1, turn);
                    break; //Exits switch statement

                case LEFTCOLUMN: //Beginning of the case statement LEFTCOLUMN (Target position: 86 cm)

                    //Exponential regression equation to decrease speed as we approach target position
                    speed = (Math.pow(0.9841381234, rangeCM1)) * 0.7;

                    straight = -180; //Sets gyro variable to -180

                    turn = (integratedZ - straight) / 50;

                    if (columnRangeCheckNeeded(rangeCM2)) {
                        CURRENT_STEP = steps.POSITIONCHECK;
                        break;
                    } else {

                        if (runtime.seconds() > 1 && rangeCM1 >= (leftposition - lefttolerance) && rangeCM1 <= (leftposition + lefttolerance) && integratedZ <= -178
                                && integratedZ >= -182) { //If in range and runtime is past 1 second
                            setDrivePower(0, 0);
                            CURRENT_STEP = steps.STOP; //Changes step to FORWARD
                            break; //Exits switch statement
                        } else if (rangeCM1 >= (leftposition - lefttolerance) && rangeCM1 <= (leftposition + lefttolerance)) { //If in range
                            if (integratedZ <= -178 && integratedZ >= -182) { //If in range and in angle
                                setDrivePower(0, 0);
                                break; //Exits switch statement
                            } else { //If in range but outside angle
                                setRotationPower(-turn * 2);
                                runtime.reset(); //Resets the runtime
                                break; //Exits switch statement
                            }
                        } else { //If outside range
                            if (rangeCM1 < (leftposition - lefttolerance)) { //If too close to wall
                                setStrafePower(-speed, turn);
                                runtime.reset(); //Resets the runtime
                                break; //Exits switch statement
                            } else { //If too far from wall
                                setStrafePower(0.18, turn);
                                runtime.reset(); //Resets the runtime
                                break; //Exits switch statement
                            }
                        }
                    }

                case CENTERCLOMUN: //Beginning of the case statement CENTERCLOMUN (Target position: 70 cm)

                    //Exponential regression equation to decrease speed as we approach target position
                    speed = (Math.pow(0.9799308653, rangeCM1)) * .7;

                    straight = -180; //Sets gyro variable to -180

                    turn = (integratedZ - straight) / 50;

                    if (columnRangeCheckNeeded(rangeCM2)) {
                        CURRENT_STEP = steps.POSITIONCHECK;
                        break;
                    } else {

                        if (runtime.seconds() > 1 && rangeCM1 >= (ceterposition - centertolerance) && rangeCM1 <= (ceterposition + centertolerance) && integratedZ <= -178
                                && integratedZ >= -182) { //If checkPosition runtime is past 1 second
                            setDrivePower(0, 0);
                            CURRENT_STEP = steps.STOP; //Changes step to FORWARD
                            break; //Exits switch statement
                        } else if (rangeCM1 >= (ceterposition - centertolerance) && rangeCM1 <= (ceterposition + centertolerance)) { //If in range
                            if (integratedZ <= -178 && integratedZ >= -182) { //If in range and in angle
                                setDrivePower(0, 0);
                                break; //Exits switch statement
                            } else { //If in range but outside angle
                                setRotationPower(-turn * 2);
                                runtime.reset(); //Resets the runtime
                                break; //Exits switch statement
                            }
                        } else { //If outside range
                            if (rangeCM1 < (ceterposition - centertolerance)) { //If too close to wall
                                setStrafePower(-speed, turn);
                                runtime.reset(); //Resets the runtime
                                break; //Exits switch statement
                            } else { //If too far from wall
                                setStrafePower(0.18, turn);
                                runtime.reset(); //Resets the runtime
                                break; //Exits switch statement
                            }
                        }
                    }

                case RIGHTCOLUMN: //Beginning of the case statement RIGHTCOLUMN (Target position: 54 cm)

                    //Exponential regression equation to decrease speed as we approach target position
                    speed = (Math.pow(0.9852338678, rangeCM1) * 0.4);

                    straight = -180; //Sets gyro variable to -180

                    turn = (integratedZ - straight) / 50;

                    if (columnRangeCheckNeeded(rangeCM2)) {
                        CURRENT_STEP = steps.POSITIONCHECK;
                        break;
                    } else {

                        if (runtime.seconds() > 1 && rangeCM1 >= (rightposition - righttolerance) && rangeCM1 <= (rightposition + righttolerance) && integratedZ <= -178
                                && integratedZ >= -182) { //If checkPosition runtime is past 1 second
                            setDrivePower(0, 0);
                            CURRENT_STEP = steps.STOP; //Changes step to FORWARD
                            break; //Exits switch statement
                        } else if (rangeCM1 >= (rightposition - righttolerance) && rangeCM1 <= (rightposition + righttolerance)) { //If in range
                            if (integratedZ <= -178 && integratedZ >= -182) { //If in range and in angle
                                setDrivePower(0, 0);
                                break; //Exits switch statement
                            } else { //If in range but outside angle
                                setRotationPower(-turn * 2);
                                runtime.reset(); //Resets the runtime
                                break; //Exits switch statement
                            }
                        } else { //If outside range
                            if (rangeCM1 < (rightposition - righttolerance)) { //If too close to wall
                                setStrafePower(-speed, turn);
                                runtime.reset(); //Resets the runtime
                                break; //Exits switch statement
                            } else { //If too far from wall
                                setStrafePower(0.18, turn);
                                runtime.reset(); //Resets the runtime
                                break; //Exits switch statement
                            }
                        }
                    }

                case POSITIONCHECK: //Beginning of the case statement BACKUP

                    if (!columnRangeCheckNeeded(rangeCM2)) { //Moves the robot backward for 0.6 seconds
                        setDrivePower(0, 0);
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

                    if (rangeCM2 < 30) {
                        setDrivePower(-0.1, 0);
                    } else {
                        setDrivePower(0.1, 0);
                    }

                    break; //Exits switch statement

                case FORWARD: //Beginning of the case statement FORWARD

                    if (runtime.seconds() > 2) { //Moves the robot forward for 2 seconds
                        setDrivePower(0, 0);
                        runtime.reset(); //Resets the runtime
                        CURRENT_STEP = steps.DROP; //Changes step to DROP
                        break; //Exits switch statement
                    }
                    setDrivePower(0.1, 0);
                    break; //Exits switch statement

                case DROP: //Beginning of the case statement DROP

                    s3.setPosition(0.5); //Opens left arm crunch servo
                    s4.setPosition(0.5); //Opens right arm crunch servo
                    setDrivePower(0, 0);
                    runtime.reset(); //Resets the runtime
                    CURRENT_STEP = steps.BACK; //Changes step to BACK
                    break; //Exits switch statement

                case BACK: //Beginning of the case statement BACK

                    if (runtime.seconds() > 0.5) { //Moves the robot backward for 0.5 seconds
                        setDrivePower(0, 0);
                        runtime.reset(); //Resets the runtime
                        CURRENT_STEP = steps.FORWARD2; //Changes step to FORWARD2
                        break; //Exits switch statement
                    }
                    setDrivePower(-0.1, 0);
                    break; //Exits switch statement

                case FORWARD2: //Beginning of the case statement FORWARD2

                    if (runtime.seconds() > 0.5) { //Moves the robot forward for 0.5 seconds
                        setDrivePower(0, 0);
                        runtime.reset(); //Resets the runtime
                        CURRENT_STEP = steps.BACK2; //Changes step to BACK2
                        break; //Exits switch statement
                    }
                    setDrivePower(0.2, 0);
                    break; //Exits switch statement

                case BACK2: //Beginning of the case statement BACK2

                    if (runtime.seconds() > 0.3) { //Moves the robot backward for 0.3 seconds
                        setDrivePower(0, 0);
                        runtime.reset(); //Resets the runtime
                        CURRENT_STEP = steps.STOP; //Changes step to STOP
                        break; //Exits switch statement
                    }
                    setDrivePower(-0.1, 0);
                    break; //Exits switch statement

                case STOP: //Beginning of the case statement STOP
                    setDrivePower(0, 0);
                    break; //Exits switch statement
            }
        }
    }
}