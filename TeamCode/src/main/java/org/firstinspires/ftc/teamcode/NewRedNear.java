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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.teamcode.RangeTestBedSteps.steps.FORWARD;
import static org.firstinspires.ftc.teamcode.RangeTestBedSteps.steps.GRAB;
import static org.firstinspires.ftc.teamcode.RangeTestBedSteps.steps.RUNTIME_RESET;
//Imports

@Autonomous(name = "NewRedNear", group = "Autonomous")
public class NewRedNear extends BlueNear {

    @Override
    public void runOpMode() { //Start of the initiation for autonomous

        m1 = hardwareMap.dcMotor.get("m1"); //Sets motors in the config
        m2 = hardwareMap.dcMotor.get("m2");
        m3 = hardwareMap.dcMotor.get("m3");
        m4 = hardwareMap.dcMotor.get("m4");
        m5 = hardwareMap.dcMotor.get("m5");
        m6 = hardwareMap.dcMotor.get("m6");
        m7 = hardwareMap.dcMotor.get("m7");

        s1 = hardwareMap.servo.get("s1"); //Sets s1 i the config
        s2 = hardwareMap.servo.get("s2"); //Sets s1 i the config
        s3 = hardwareMap.servo.get("s3"); //Sets s1 i the config
        s4 = hardwareMap.servo.get("s4"); //Sets s1 i the config
        //s5 = hardwareMap.servo.get("s5"); //Sets s1 i the config
        s6 = hardwareMap.servo.get("s6"); //Sets s1 i the config
        s7 = hardwareMap.servo.get("s7"); //Sets s1 i the config

        m1.setDirection(DcMotor.Direction.REVERSE);
        m3.setDirection(DcMotor.Direction.REVERSE);

        runtime = new ElapsedTime(); //Creates runtime variable for using time
        checkTime = new ElapsedTime(); //Creates runtime variable for using time
        matchTime = new ElapsedTime(); //Creates runtime variable for using time

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

        ods1 = hardwareMap.opticalDistanceSensor.get("ods1");
        ods2 = hardwareMap.opticalDistanceSensor.get("ods2");
        ods3 = hardwareMap.opticalDistanceSensor.get("ods3");

        c1 = hardwareMap.colorSensor.get("c1"); //Sets colorSensor to c1 in the config, Port 5
        c1.enableLed(true); //Turns Color Sensor LED off

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;

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

        s1.setPosition(0); //Pulls jewel appendage against side of robot
        s2.setPosition(.3); //Opens Relic Claw
        s3.setPosition(0.2); //Sets Arm Crunch Servo A
        s4.setPosition(1); //Sets Arm Crunch Servo B
//        s5.setPosition(0.5); //Opens 2nd gripper *NOT USED*
        s6.setPosition(0.5); //Sets arm extension to not move
        s7.setPosition(0.9);

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
        int image = 0;
        int cryptoDistance = -620;
        double glyphODS = 0;
        boolean backupRan = false;
        double rotationMin = 0;
        double rotationPrev = 0;
        double power = 0;
        boolean firstCheck = true;
        int m2Track = 0;

        double odsLight1 = ods1.getRawLightDetected(); //Initializes rangeCM1 for range reading
        double odsLight2 = ods2.getRawLightDetected(); //Initializes rangeCM2 for range reading
        double odsLight3 = ods3.getRawLightDetected(); //Initializes rangeCM3 for range reading

        waitForStart(); //Waits for start

        double ods1Curr = 0; //Initializes variable to track current range 1 reading
        double ods2Curr = 0; //Initializes variable to track current range 2 reading
        double ods3Curr = 0; //Initializes variable to track current range 4 reading

        double ods1Prev = ods1.getRawLightDetected(); //Defining variable used in low pass filter
        if (ods1Prev > 1 || ods1Prev < 0) { //Error check
            ods1Prev = 0;
        }
        double ods2Prev = ods2.getRawLightDetected(); //Defining variable used in low pass filter
        if (ods2Prev > 1 || ods2Prev < 0) { //Error check
            ods2Prev = 0;
        }
        double ods3Prev = ods3.getRawLightDetected(); //Defining variable used in low pass filter
        if (ods3Prev > 1 || ods3Prev < 0) { //Error check
            ods3Prev = 0;
        }

        while (opModeIsActive()) {

            ods1Curr = odsCheck(ods1.getLightDetected());
            ods2Curr = odsCheck(ods2.getLightDetected());
            ods3Curr = odsCheck(ods3.getLightDetected());
            odsLight1 = (ods1Curr * 0.2) + (ods1Prev * 0.8); //Updates rangeCM1 variable with low pass filter
            ods1Prev = odsLight1; //Updates rCM1Prev variable with info current rangeCM1 variable

            odsLight2 = (ods2Curr * 0.2) + (ods2Prev * 0.8); //Updates rangeCM1 variable with low pass filter
            ods2Prev = odsLight2; //Updates rCM1Prev variable with info current rangeCM1 variable

            odsLight3 = (ods3Curr * 0.2) + (ods3Prev * 0.8); //Updates rangeCM1 variable with low pass filter
            ods3Prev = odsLight3; //Updates rCM1Prev variable with info current rangeCM1 variable

            odsLight1 = odsCheck(odsLight1);
            odsLight2 = odsCheck(odsLight2);
            odsLight3 = odsCheck(odsLight3);

            telemetry.addData("Step", CURRENT_STEP);
            telemetry.addData("Gyro", modernRoboticsI2cGyro.getIntegratedZValue());
            telemetry.addData("power", power);
            telemetry.addData("Glyph ODS", glyphODS);
            telemetry.addData("m2 correct check", m2Track);
            telemetry.addData("ods3", odsLight3);
            telemetry.update();

            switch (CURRENT_STEP) { //Beginning of the switch- this sets the current step to whatever CURRENT_STEP is set to

                ///////////////////////
                //START OF MAIN STEPS//
                ///////////////////////

                case SCANIMAGE: //Beginning of case statement SCANIMAGE

                    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate); //Image scanning

                    gripClose();

                    if (vuMark == RelicRecoveryVuMark.LEFT) { //Vuforia for left pictograph
                        image = 1;
                        cryptoDistance = -1150;
                        changeStep();
                        matchTime.reset();
                        CURRENT_STEP = steps.LOWERSERVO; //Changes step to LOWERSERVO
                        break; //Exits switch statement
                    }

                    if (vuMark == RelicRecoveryVuMark.CENTER) { //Vuforia for center pictograph
                        image = 2;
                        cryptoDistance = -620;
                        changeStep();
                        matchTime.reset();
                        CURRENT_STEP = steps.LOWERSERVO; //Changes step to LOWERSERVO
                        break; //Exits switch statement
                    }

                    if (vuMark == RelicRecoveryVuMark.RIGHT) { //Vuforia for right pictograph
                        image = 3;
                        cryptoDistance = -230;
                        changeStep();
                        matchTime.reset();
                        CURRENT_STEP = steps.LOWERSERVO; //Changes step to LOWERSERVO
                        break; //Exits switch statement
                    }

                    if (image == 0 && runtime.seconds() > 3) { //If we don't scan the image
                        image = 2;
                        cryptoDistance = -620;
                        changeStep();
                        matchTime.reset();
                        CURRENT_STEP = steps.LOWERSERVO; //Changes step to LOWERSERVO
                        break;
                    }
                    break; //Exits switch statement

                case LOWERSERVO: //Beginning of the case statement

                    s1.setPosition(0.55); //Sets servo 1 position to 0.55 (lowers jewel arm)
                    gripClose();
                    changeStep();
                    CURRENT_STEP = steps.SENSECOLOR; //Changes step to SENSECOLOR
                    break; //Exits switch statement

                case SENSECOLOR: //Beginning of the case statement

                    if (runtime.seconds() > 0.3 && runtime.seconds() < 1) { //Activates motor to raise glyph
                        m7.setPower(-0.7);
                    } else {
                        m7.setPower(0);
                    }
                    if (c1.blue() > c1.red() && runtime.seconds() > 1) {
                        changeStep();
                        CURRENT_STEP = steps.KNOCKBACK; //Changes step to KNOCKBACK
                        break; //Exits switch statement
                    }

                    if (c1.red() > c1.blue() && runtime.seconds() > 1) {
                        changeStep();
                        CURRENT_STEP = steps.KNOCKFORWARDS; //Changes step to KNOCKFORWARDS
                        break; //Exits switch statement
                    }

                    if (c1.red() == c1.blue() && c1.red() == 0 && runtime.seconds() > 1) {
                        changeStep();
                        CURRENT_STEP = steps.RAISESERVO; //Changes step to RAISESERVO
                        break;
                    }

                    if (runtime.seconds() > .5) {
                        s1.setPosition(0.75); //Continues to lower jewel arm
                    }
                    break; //Exits switch statement

                case KNOCKBACK: //Beginning of the case statement

                    if (runtime.seconds() > .3) {
                        changeStep();
                        CURRENT_STEP = steps.RAISESERVO; //Changes step to KNOCKFORWARDS
                        break; //Exits switch statement
                    }
                    setDrivePower(-.2, 0);
                    break;

                case KNOCKFORWARDS: //Beginning of the case statement

                    if(runtime.seconds() > .9) {
                        changeStep();
                        CURRENT_STEP = steps.RAISESERVO; //Changes step to KNOCKFORWARDS
                        break; //Exits switch statement
                    }
                    if(runtime.seconds() < 0.3) {
                        setRotationPower(.15);
                        break;
                    }
                    if((runtime.seconds() > 0.5 && runtime.seconds() < 0.9) && modernRoboticsI2cGyro.getIntegratedZValue() > 0) {
                        s1.setPosition(0);
                        setRotationPower(-.15);
                        break;
                    }
                    break;

                case RAISESERVO:

                    s1.setPosition(0); //Sets servo 1 position to 0 (raises jewel arm)
                    changeStep();
                    CURRENT_STEP = steps.OFF_STONE; //Changes step to DRIVETOCRYPTOBOX
                    break; //Exits switch statement

                case OFF_STONE:

                    if(runtime.seconds() > 1 && r3.getDistance(DistanceUnit.CM) > 38) {
                        changeStep();
                        firstCheck = false;
                        checkTime.reset();
                        CURRENT_STEP = steps.CHECK_ROTATION;
                        break;
                    }
                    if(runtime.seconds() < 1) {
                        setDrivePower(-.12, 0);
                        break;
                    } else {
                        setStrafePower(.4, 0);
                        break;
                    }

                case CHECK_ROTATION:

                    target = 0;
                    if ((modernRoboticsI2cGyro.getIntegratedZValue() > target - 2 && modernRoboticsI2cGyro.getIntegratedZValue() < target + 2) || checkTime.seconds() > 2) {//modernRoboticsI2cGyro.getIntegratedZValue() < target + 2 && modernRoboticsI2cGyro.getIntegratedZValue() > target - 2) {
                        changeStep();
                        CURRENT_STEP = steps.BACK_STONE;
                        break;
                    }
                    if(!firstCheck && runtime.seconds() > .2) {
                        rotationPrev = modernRoboticsI2cGyro.getIntegratedZValue();
                        if(modernRoboticsI2cGyro.getIntegratedZValue() > target) {
                            power = -.12;
                        }
                        if(modernRoboticsI2cGyro.getIntegratedZValue() < target) {
                            power = .12;
                        }
                        firstCheck = true;
                        runtime.reset();
                    } else {
                        if(runtime.seconds() > .5) {
                            if(power > 0 && modernRoboticsI2cGyro.getIntegratedZValue() > target) {
                                power = -.12;
                            }
                            if(power < 0 && modernRoboticsI2cGyro.getIntegratedZValue() < target) {
                                power = .12;
                            }
                            if(modernRoboticsI2cGyro.getIntegratedZValue() == rotationPrev) {
                                power = setRotationPercise(target, power);
                            }
                            rotationPrev = modernRoboticsI2cGyro.getIntegratedZValue();
                            runtime.reset();
                        }
                    }
                    setRotationPower(power);
                    break;

                case BACK_STONE:

                    if (runtime.seconds() > 2.3) {
                        cryptoDistance += m2.getCurrentPosition();
                        changeStep();
                        CURRENT_STEP = steps.DRIVE_TO_CRYPTO;
                        break;
                    }
                    if (runtime.seconds() > .2) {
                        setDrivePower(.1, 0);
                    }
                    break;

                case DRIVE_TO_CRYPTO:

                    if (m2.getCurrentPosition() < cryptoDistance + 50 && m2.getCurrentPosition() > cryptoDistance - 50) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE;
                        break;
                    }
                    //setDriveEncoder(1000);
                    setDrivePower(-.1, 0);
                    break;

                case CHECK_DISTANCE:

                    if(r3.getDistance(DistanceUnit.CM) > 38) {
                        changeStep();
                        CURRENT_STEP = steps.ROTATE;
                        break;
                    }
                    if(r3.getDistance(DistanceUnit.CM) < 38 && runtime.seconds() > .5) {
                        setStrafePower(.4, 0);
                    }
                    break;

                case ROTATE: //Rotation counter clockwise = -, clockwise = +

                    target = -90;
                    if(modernRoboticsI2cGyro.getIntegratedZValue() > target - 20 && modernRoboticsI2cGyro.getIntegratedZValue() < target + 20) {
                        changeStep();
                        firstCheck = false;
                        checkTime.reset();
                        CURRENT_STEP = steps.PRECISE_ROTATE;
                        break;
                    }
                    setRotationTarget(target);
                    break;

                case PRECISE_ROTATE:

                    target = -90;
                    if (modernRoboticsI2cGyro.getIntegratedZValue() == target || checkTime.seconds() > 4) {//modernRoboticsI2cGyro.getIntegratedZValue() < target + 2 && modernRoboticsI2cGyro.getIntegratedZValue() > target - 2) {
                        changeStep();
                        CURRENT_STEP = steps.DROP_GLYPH;
                        break;
                    }
                    if(!firstCheck && runtime.seconds() > .2) {
                        rotationPrev = modernRoboticsI2cGyro.getIntegratedZValue();
                        if(modernRoboticsI2cGyro.getIntegratedZValue() > target) {
                            power = -.12;
                        }
                        if(modernRoboticsI2cGyro.getIntegratedZValue() < target) {
                            power = .12;
                        }
                        firstCheck = true;
                        runtime.reset();
                    } else {
                        if(runtime.seconds() > .5) {
                            if(power > 0 && modernRoboticsI2cGyro.getIntegratedZValue() > target) {
                                power = -.12;
                            }
                            if(power < 0 && modernRoboticsI2cGyro.getIntegratedZValue() < target) {
                                power = .12;
                            }
                            if(modernRoboticsI2cGyro.getIntegratedZValue() == rotationPrev) {
                                power = setRotationPercise(target, power);
                            }
                            rotationPrev = modernRoboticsI2cGyro.getIntegratedZValue();
                            runtime.reset();
                        }
                    }
                    setRotationPower(power);
                    /*
                    else if (modernRoboticsI2cGyro.getIntegratedZValue() < target) {//modernRoboticsI2cGyro.getIntegratedZValue() < target + 2 && modernRoboticsI2cGyro.getIntegratedZValue() > target - 2) {
                        changeStep();
                        firstCheck = true;
                        CURRENT_STEP = steps.PRECISE_ROTATE_C;
                        break;
                    }
                    //Scans for 1st rotationPrev
                    if (firstCheck) {
                        if (runtime.seconds() < 0.2) {
                            rotationMin = -0.1;
                            rotationPrev = modernRoboticsI2cGyro.getIntegratedZValue();
                            firstCheck = false;
                            runtime.reset();
                        }
                    } else {
                        //Scans for rotationPrev
                        if(runtime.seconds() > 0.7) {
                            rotationPrev = modernRoboticsI2cGyro.getIntegratedZValue();
                            runtime.reset();
                        }
                        //Changes rotationMin if robot is not moving
                        if (runtime.seconds() > 0.5 && runtime.seconds() < 0.7 && rotationPrev == modernRoboticsI2cGyro.getIntegratedZValue()) {
                            rotationMin -= 0.01;
                        }

                        setRotationPower(rotationMin);
                    }
                    break;
                    */
                    break;
                /*
                case PRECISE_ROTATE_C:

                    target = -90;

                    if (modernRoboticsI2cGyro.getIntegratedZValue() == target) {//modernRoboticsI2cGyro.getIntegratedZValue() < target + 2 && modernRoboticsI2cGyro.getIntegratedZValue() > target - 2) {
                        changeStep();
                        CURRENT_STEP = steps.DROP_GLYPH;
                        break;
                    } else if (modernRoboticsI2cGyro.getIntegratedZValue() > target) {//modernRoboticsI2cGyro.getIntegratedZValue() < target + 2 && modernRoboticsI2cGyro.getIntegratedZValue() > target - 2) {
                        changeStep();
                        firstCheck = true;
                        CURRENT_STEP = steps.PRECISE_ROTATE_CC;
                        break;
                    }
                    //Scans for 1st rotationPrev
                    if (firstCheck) {
                        if (runtime.seconds() < 0.2) {
                            rotationMin = 0.1;
                            rotationPrev = modernRoboticsI2cGyro.getIntegratedZValue();
                            firstCheck = false;
                            runtime.reset();
                        }
                    } else {
                        //Scans for rotationPrev
                        if(runtime.seconds() > 0.7) {
                            rotationPrev = modernRoboticsI2cGyro.getIntegratedZValue();
                            runtime.reset();
                        }
                        //Changes rotationMin if robot is not moving
                        if (runtime.seconds() > 0.5 && runtime.seconds() < 0.7 && rotationPrev == modernRoboticsI2cGyro.getIntegratedZValue()) {
                            rotationMin += 0.01;
                        }

                        setRotationPower(rotationMin);
                    }
                    break;
                */
                case DROP_GLYPH:

                    if (runtime.seconds() > 4) {
                        changeStep();
                        CURRENT_STEP = steps.FACE_PILE;
                        break;
                    } else {
                        if (runtime.seconds() < 1.5) {
                            setDrivePower(.1, 0);
                            break;
                        }
                        if (runtime.seconds() > 1.5 && runtime.seconds() < 2) {
                            gripOpen();
                            setDrivePower(-.1, 0);
                            break;
                        }
                        if (runtime.seconds() > 2 && runtime.seconds() < 3) {
                            setDrivePower(.1, 0);
                            break;
                        }
                        if (runtime.seconds() > 3 && runtime.seconds() < 4) {
                            setDrivePower(-.1, 0);
                            break;
                        }
                        break;
                    }

                case FACE_PILE:

                    target = 90; //MUST CHANGE
                    if(modernRoboticsI2cGyro.getIntegratedZValue() > target - 10 && modernRoboticsI2cGyro.getIntegratedZValue() < target + 10) {
                        changeStep();
                        firstCheck = false;
                        CURRENT_STEP = steps.RESET;
                        if(matchTime.seconds() > 20) {
                            CURRENT_STEP = steps.STOP;
                            break;
                        }
                        break;
                    }
                    setRotationTarget(target);
                    break;

                case RESET:

                    changeStep();
                    CURRENT_STEP = steps.FIND_GLYPHS; //Changes step to BACK2
                    break; //Exits switch statement

                case FIND_GLYPHS:

                    if(odsLight3 > .025) {
                        changeStep();
                        glyphODS = odsLight3;
                        m2Track = m2.getCurrentPosition();
                        CURRENT_STEP = steps.SCAN_GLYPHS; //Changes step to BACK2
                        break; //Exits switch statement
                    }
                    if(odsLight1 > .025 || odsLight2 > .025) {
                        changeStep();
                        m2Track = m2.getCurrentPosition();
                        CURRENT_STEP = steps.FORWARD_GLYPH; //Changes step to BACK2
                        break; //Exits switch statement
                    }
                    m7.setPower(.3);
                    gripScan();
                    if(runtime.seconds() < 3) {
                        setDrivePower(.1, 0);
                    } else {
                        if(runtime.seconds() > 4) {
                            changeStep();
                            glyphODS = odsLight3;
                            m2Track = m2.getCurrentPosition();
                            CURRENT_STEP = steps.SCAN_GLYPHS; //Changes step to BACK2
                            break; //Exits switch statement
                        }
                        setDrivePower(-.08, 0);
                    }

                    break;

                case SCAN_GLYPHS:

                    if((odsLight3 == 0 && runtime.seconds() > .5) || odsLight1 > .025 || odsLight2 > .025) {
                        changeStep();
                        m2Track = Math.abs(m2.getCurrentPosition() - m2Track);
                        CURRENT_STEP = steps.FORWARD_GLYPH; //Changes step to BACK2
                        break; //Exits switch statement
                    }
                    if(runtime.seconds() > .5 && runtime.seconds() < .6) {
                        setDrivePower(-.1, 0);
                    }
                    if(runtime.seconds() > .7) {
                        setStrafePower(.45,0);
                        gripClose();
                    }
                    m7.setPower(0);
                    break;

                case FORWARD_GLYPH:

                    if (runtime.seconds() > .8) {
                        changeStep();
                        CURRENT_STEP = steps.GRAB_GLYPH; //Changes step to BACK2
                        break; //Exits switch statement
                    }
                    gripOpen();
                    setDrivePower(.2, 0);
                    break;

                case GRAB_GLYPH:

                    if (runtime.seconds() > .6) {
                        changeStep();
                        CURRENT_STEP = steps.BACK_GLYPH; //Changes step to BACK2
                        break; //Exits switch statement
                    }
                    gripClose();
                    setDrivePower(0,0);
                    break;

                case BACK_GLYPH:

                    if(runtime.seconds() > .8) {
                        changeStep();
                        CURRENT_STEP = steps.FACE_CRYPTO; //Changes step to BACK2
                        break; //Exits switch statement
                    }
                    if(runtime.seconds() < .6) {
                        m7.setPower(-.8);
                    } else {
                        m7.setPower(0);
                    }
                    setDrivePower(-.2,0);
                    break;

                case FACE_CRYPTO:

                    target = -90; //MUST CHANGE
                    if(modernRoboticsI2cGyro.getIntegratedZValue() > target - 60 && modernRoboticsI2cGyro.getIntegratedZValue() < target + 60) {
                        changeStep();
                        firstCheck = false;
                        CURRENT_STEP = steps.DROP_GLYPH_2;
                        break;
                    }
                    setRotationTarget(target);
                    break;

                case PRECISE_ROTATE_2:

                    target = -72;
                    if (modernRoboticsI2cGyro.getIntegratedZValue() == target) {//modernRoboticsI2cGyro.getIntegratedZValue() < target + 2 && modernRoboticsI2cGyro.getIntegratedZValue() > target - 2) {
                        changeStep();
                        CURRENT_STEP = steps.DROP_GLYPH_2;
                        break;
                    }
                    if(!firstCheck && runtime.seconds() > .2) {
                        rotationPrev = modernRoboticsI2cGyro.getIntegratedZValue();
                        if(modernRoboticsI2cGyro.getIntegratedZValue() > target) {
                            power = -.12;
                        }
                        if(modernRoboticsI2cGyro.getIntegratedZValue() < target) {
                            power = .12;
                        }
                        firstCheck = true;
                        runtime.reset();
                    } else {
                        if(runtime.seconds() > .5) {
                            if(power > 0 && modernRoboticsI2cGyro.getIntegratedZValue() > target) {
                                power = -.12;
                            }
                            if(power < 0 && modernRoboticsI2cGyro.getIntegratedZValue() < target) {
                                power = .12;
                            }
                            if(modernRoboticsI2cGyro.getIntegratedZValue() == rotationPrev) {
                                power = setRotationPercise(target, power);
                            }
                            rotationPrev = modernRoboticsI2cGyro.getIntegratedZValue();
                            runtime.reset();
                        }
                    }
                    setRotationPower(power);
                    break;

                case DROP_GLYPH_2:

                    if (runtime.seconds() > 3.5) {
                        changeStep();
                        CURRENT_STEP = steps.STOP;
                        break;
                    } else {
                        if (runtime.seconds() < 1.5) {
                            m2.setPower(.7);
                            m3.setPower(.7);
                            break;
                        }
                        if (runtime.seconds() > 1.5 && runtime.seconds() < 2) {
                            gripOpen();
                            m2.setPower(-.4);
                            m3.setPower(-.4);
                            break;
                        }
                        if (runtime.seconds() > 2 && runtime.seconds() < 3) {
                            m2.setPower(.7);
                            m3.setPower(.7);
                            break;
                        }
                        if (runtime.seconds() > 3 && runtime.seconds() < 3.5) {
                            m2.setPower(-.4);
                            m3.setPower(-.4);
                            break;
                        }
                        break;
                    }

                case BACKUP_ROTATE:

                case STOP: //Beginning of the case statement

                    changeStep();
                    break; //Exits switch statement

            }
        }
    }
}