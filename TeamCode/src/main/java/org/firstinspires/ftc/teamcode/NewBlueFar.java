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

@Autonomous(name = "BLUE_far", group = "Autonomous") //Names program
public class NewBlueFar extends TardisAutonomous { //Creates class and extends program wih steps

    double rangeCheckClose = 35; //Variables for range check
    double rangeCheckFar = 45;

    public boolean columnRangeCheckNeeded(double rangeCM2) { //Checks to see if robot is too close or far from cryptobox
        return (rangeCM2 <= rangeCheckClose || rangeCM2 >= rangeCheckFar);
    }

    float leftposition = 54; //Variable for left column positioning
    float centerposition = 74; //Variable for center column positioning
    float rightposition = 90; //Variable for right column positioning

    @Override
    public void runOpMode() { //Beginning of main loop

        //Sets motors from config
        m1 = hardwareMap.dcMotor.get("m1");
        m2 = hardwareMap.dcMotor.get("m2");
        m3 = hardwareMap.dcMotor.get("m3");
        m4 = hardwareMap.dcMotor.get("m4");
        m5 = hardwareMap.dcMotor.get("m5");
        m6 = hardwareMap.dcMotor.get("m6");
        m7 = hardwareMap.dcMotor.get("m7");

        //Sets servos from config
        s1 = hardwareMap.servo.get("s1");
        s2 = hardwareMap.servo.get("s2");
        s3 = hardwareMap.servo.get("s3");
        s4 = hardwareMap.servo.get("s4");
        s6 = hardwareMap.servo.get("s6");
        s7 = hardwareMap.servo.get("s7");

        //Sets optical distance sensors from config
        ods1 = hardwareMap.opticalDistanceSensor.get("ods1");
        ods2 = hardwareMap.opticalDistanceSensor.get("ods2");
        ods3 = hardwareMap.opticalDistanceSensor.get("ods3");

        m1.setDirection(DcMotor.Direction.REVERSE); //Sets m1 direction to REVERSE
        m3.setDirection(DcMotor.Direction.REVERSE); //Sets m3 direction to REVERSE

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro"); //Configures gyro, port 6
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;

        c1 = hardwareMap.colorSensor.get("c1"); //Sets color sensor to c1 in the config, port 5
        c1.enableLed(true); //Turns Color Sensor LED on

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
        checkTime = new ElapsedTime(); //Creates checkTime variable for using time
        matchTime = new ElapsedTime(); //Creates matchTime variable to track match time

        s1.setPosition(0); //Pulls jewel appendage against side of robot
        s2.setPosition(0.3); //Opens Relic Claw
        s3.setPosition(0.2); //Sets Arm Crunch Servo A
        s4.setPosition(1); //Sets Arm Crunch Servo B
        s6.setPosition(0.5); //Sets arm extension to not move
        s7.setPosition(0.9); //Sets back up relic gripper out of the way

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

        double rangeCM1 = r1.getDistance(DistanceUnit.CM); //Initializes rangeCM1 for range reading
        double rangeCM2 = r2.getDistance(DistanceUnit.CM); //Initializes rangeCM2 for range reading
        double rangeCM3 = r3.getDistance(DistanceUnit.CM); //Initializes rangeCM3 for range reading
        double rangeCM4 = r4.getDistance(DistanceUnit.CM); //Initializes rangeCM4 for range reading
        int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue(); //Gyro integratedZ value

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

        double target; //Initializes variable to help with rotations using sigmoid functions
        double targetPosition = 0; //Initializes variable to help with strafing using sigmoid functions
        boolean firstCheck = false; //Variable for precise movement checks
        double readingPrev = 0; //Variable for precise movement checks
        double power = 0; //Variable for precise movement checks

        runtime.reset();
        checkTime.reset();

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

            ods1Curr = odsCheck(ods1.getLightDetected());
            ods2Curr = odsCheck(ods2.getLightDetected());
            ods3Curr = odsCheck(ods3.getLightDetected());
            odsLight1 = (ods1Curr * 0.2) + (ods1Prev * 0.8); //Updates rangeCM1 variable with low pass filter
            ods1Prev = odsLight1; //Updates rCM1Prev variable with info current rangeCM1 variable

            odsLight2 = (ods2Curr * 0.2) + (ods2Prev * 0.8); //Updates rangeCM1 variable with low pass filter
            ods2Prev = odsLight2; //Updates rCM1Prev variable with info current rangeCM1 variable

            odsLight3 = (ods3Curr * 0.2) + (ods3Prev * 0.8); //Updates rangeCM1 variable with low pass filter
            ods3Prev = odsLight3; //Updates rCM1Prev variable with info current rangeCM1 variable

            //Confirms readings are reasonable
            odsLight1 = odsCheck(odsLight1);
            odsLight2 = odsCheck(odsLight2);
            odsLight3 = odsCheck(odsLight3);

            ///////////////////////////
            //Telemetry for debugging//
            ///////////////////////////

            telemetry.addData("Image", image);
            telemetry.addData("Blue", c1.blue());
            telemetry.addData("Red", c1.red());
            telemetry.addData("Range 1 CM", rangeCM1);
            telemetry.addData("Range 2 CM", rangeCM2);
            telemetry.addData("Range 3 CM", rangeCM3);
            telemetry.addData("Range 4 CM", rangeCM4);
            telemetry.addData("Gyro Int. Z", integratedZ);
            telemetry.addData("Runtime", runtime.seconds());
            telemetry.update(); //Updates telemetry with new information

            /////////////////////////////
            //Start of switch statement//
            /////////////////////////////

            switch (CURRENT_STEP) { //Beginning of the switch. This sets the current step to whatever CURRENT_STEP is set to

                ///////////////////////
                //START OF MAIN STEPS//
                ///////////////////////

                case SCANIMAGE: //Uses Vuforia on the phone to detect pictograph

                    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate); //Image scanning
                    matchTime.reset();
                    gripClose();

                    if (vuMark == RelicRecoveryVuMark.LEFT) { //Vuforia for left pictograph
                        image = 1;
                        targetPosition = leftposition;
                        changeStep();
                        CURRENT_STEP = steps.LOWERSERVO;
                        break;
                    }
                    if (vuMark == RelicRecoveryVuMark.CENTER) { //Vuforia for center pictograph
                        image = 2;
                        targetPosition = centerposition;
                        changeStep();
                        CURRENT_STEP = steps.LOWERSERVO;
                        break;
                    }
                    if (vuMark == RelicRecoveryVuMark.RIGHT) { //Vuforia for right pictograph
                        image = 3;
                        targetPosition = rightposition;
                        changeStep();
                        CURRENT_STEP = steps.LOWERSERVO;
                        break;
                    }
                    if (image == 0 && runtime.seconds() > 3) { //If phone doesn't scan the image, default to center column
                        image = 2;
                        targetPosition = centerposition;
                        changeStep();
                        CURRENT_STEP = steps.LOWERSERVO;
                    }
                    break;

                case LOWERSERVO: //Lowers the arm containing the color sensor

                    s1.setPosition(0.55); //Sets servo 1 position to 0.55 (lowers jewel arm)
                    changeStep();
                    CURRENT_STEP = steps.SENSECOLOR;
                    break;

                case SENSECOLOR: //Uses the color sensor on extended arm to detect color of jewel

                    if (runtime.seconds() < 0.8) { //Activates motor to raise arm crunch with glyph
                        m7.setPower(-0.7);
                    } else {
                        m7.setPower(0);
                    }
                    if (c1.blue() > c1.red() && runtime.seconds() > 1) { //If the color sensor sees more blue than red, knock forwards
                        changeStep();
                        CURRENT_STEP = steps.KNOCKFORWARDS;
                        break;
                    }

                    if (c1.red() > c1.blue() && runtime.seconds() > 1) { //If the color sensor sees more red than blue, knock back
                        changeStep();
                        CURRENT_STEP = steps.KNOCKBACK;
                        break;
                    }

                    if (c1.red() == c1.blue() && c1.red() == 0 && runtime.seconds() > 1) { //If there is no distinct difference between colors, raise the color sensor and hit nothing
                        changeStep();
                        CURRENT_STEP = steps.RAISESERVO;
                        break;
                    }

                    if (runtime.seconds() > 0.5) {
                        s1.setPosition(0.75); //Continues to lower jewel arm
                    }
                    break;

                case KNOCKBACK: //Rotates the robot back to hit off jewel

                    if (runtime.seconds() > 1) {
                        changeStep();
                        CURRENT_STEP = steps.RAISESERVO;
                        break;
                    }
                    if (runtime.seconds() < 0.3) {
                        setRotationPower(-0.15);
                        break;
                    }
                    if ((runtime.seconds() > 0.5 && runtime.seconds() < 0.9) && modernRoboticsI2cGyro.getIntegratedZValue() < 0) {
                        s1.setPosition(0);
                        setRotationPower(0.15);
                        break;
                    }
                    break;

                case KNOCKFORWARDS: //Drives the robot forward to hit off jewel

                    if (runtime.seconds() > 0.3) {
                        changeStep();
                        CURRENT_STEP = steps.RAISESERVO;
                        break;
                    }
                    setDrivePower(0.2, 0);
                    break;

                case RAISESERVO: //Raises jewel arm

                    s1.setPosition(0); //Sets servo 1 position to 0 (raises jewel arm)
                    changeStep();
                    CURRENT_STEP = steps.DRIVETOCRYPTOBOX;
                    break;

                case DRIVETOCRYPTOBOX: //Drives forward off balancing stone

                    if (rangeCM2 < 60) { //Moves forward until wall
                        changeStep();
                        CURRENT_STEP = steps.CHECK_ROTATION;
                        break;
                    }
                    setDrivePower(0.08, 0);
                    s1.setPosition(0); //Sets servo 1 position to 0 (raises jewel arm)
                    break;

                case CHECK_ROTATION: //Confirms that we are aligned perpendicular to the cryptobox

                    target = 0; //Sets target for the precise rotation
                    if ((modernRoboticsI2cGyro.getIntegratedZValue() > target - 2 && modernRoboticsI2cGyro.getIntegratedZValue() < target + 2) || checkTime.seconds() > 2) { //If the gyro is in the range of 4 degrees or the step takes more than two seconds, move on
                        changeStep();
                        CURRENT_STEP = steps.ALIGN_CRYPTO;
                        break;
                    }
                    if (!firstCheck && runtime.seconds() > 0.2) { //If it has not checked for the initial starting orientation of the robot, do so (only runs once)
                        readingPrev = integratedZ;
                        if (integratedZ > target) { //If the gyro is reading above the target, start moving counterclockwise
                            power = -0.12;
                        }
                        if (integratedZ < target) { //If the gyro is reading below the target, start moving clockwise
                            power = 0.12;
                        }
                        firstCheck = true; //Confirms step only runs once
                        runtime.reset();
                    } else { //If the first step information has been completed
                        if (runtime.seconds() > 0.5) { //Every 0.5 seconds, check to make sure the gyro has changed
                            if (power > 0 && integratedZ > target) { //If the power is positive but the robot must turn counterclockwise, readjust
                                power = -0.12;
                            }
                            if (power < 0 && integratedZ < target) { //If the power is negative but the robot must turn clockwise, readjust
                                power = 0.12;
                            }
                            if (integratedZ == readingPrev) { //If the gyro has not changed, increase the rotation speed
                                power = setRotationPrecise(target, power);
                            }
                            readingPrev = integratedZ;
                            runtime.reset();
                        }
                    }
                    setRotationPower(power);
                    break;

                case ALIGN_CRYPTO: //Strafes to the correct column using the targetPosition variable

                    double gyroTarget = 0;
                    if ((rangeCM3 < targetPosition + 2 && rangeCM3 > targetPosition - 2) || runtime.seconds() > 3) { //If the robot arrives within 4 cm of its target or it takes longer than 3 seconds to complete, move on
                        changeStep();
                        checkTime.reset();
                        CURRENT_STEP = steps.POSITIONCHECK;
                        break;
                    }
                    setRangeTargetR3(targetPosition, rangeCM3, gyroTarget); //Sends the target position and the
                    break;
/*
                case CENTERCOLUMN: //Beginning of the case statement CENTERCLOMUN

                    //Exponential regression equation to decrease speed as we approach target position
                    speed = ((Math.pow(rangeCM3 - (centerposition + 15), 2)) / 5000) + .13;

                    straight = 1; //Sets gyro variable to 

                    if(integratedZ > straight) { //Changes turn variable
                        turn = .1;
                    } else if(integratedZ < straight) {
                        turn = -.1;
                    } else {
                        turn = 0;
                    }

                    if(columnRangeCheckNeeded(rangeCM2)) { //Checks range from cryptobox
                        CURRENT_STEP = steps.POSITIONCHECK;
                        break;
                    } else { //If in range

                        if(runtime.seconds() > 1 && rangeCM3 >= (centerposition - centertolerance) && rangeCM3 <= (centerposition + centertolerance) && integratedZ <= (straight + gyroTolerance) && integratedZ >= (straight - gyroTolerance) || checkTime.seconds() > 25) { //If checkPosition runtime is past 1 second
                            setDrivePower(0, 0); //Stops robot
                            CURRENT_STEP = steps.FORWARD; //Changes step to FORWARD
                            break; //Exits switch statement
                        } else if(rangeCM3 >= (centerposition - centertolerance) && rangeCM3 <= (centerposition + centertolerance)) { //If in range
                            if(integratedZ <= (straight + gyroTolerance) && integratedZ >= (straight - gyroTolerance)) { //If in range and in angle
                                setDrivePower(0, 0); //Stops robot
                                break; //Exits switch statement
                            } else { //If in range but outside angle
                                setRotationPower(-turn); //Rotates robot
                                runtime.reset(); //Resets the runtime
                                break; //Exits switch statement
                            }
                        } else { //If outside range
                            if(rangeCM3 < (centerposition - centertolerance)) { //If too close to wall
                                setStrafePower(speed, turn); //Strafes right and rotates
                                runtime.reset(); //Resets the runtime
                                break; //Exits switch statement
                            } else { //If too far from wall
                                setStrafePower(-0.26, turn); //Strafes left and rotates
                                runtime.reset(); //Resets the runtime
                                break; //Exits switch statement
                            }
                        }
                    }
                case RIGHTCOLUMN: //Beginning of the case statement RIGHTCOLUMN

                    //Exponential regression equation to decrease speed as we approach target position
                    speed = ((Math.pow(rangeCM3 - (rightposition + 5), 2)) / 5000) + .4;

                    straight = 1; //Sets gyro variable to 0

                    if(integratedZ > straight) { //Changes turn variable
                        turn = .1;
                    } else if(integratedZ < straight) {
                        turn = -.1;
                    } else {
                        turn = 0;
                    }

                    if(columnRangeCheckNeeded(rangeCM2)) { //Checks range from cryptobox
                        CURRENT_STEP = steps.POSITIONCHECK;
                        break;
                    } else { //If in range

                        if(runtime.seconds() > 1 && rangeCM3 >= (rightposition - righttolerance) && rangeCM3 <= (rightposition + righttolerance) && integratedZ <= (straight + gyroTolerance) && integratedZ >= (straight - gyroTolerance) || checkTime.seconds() > 25) { //If checkPosition runtime is past 1 second
                            setDrivePower(0, 0); //Stops robot
                            CURRENT_STEP = steps.FORWARD; //Changes step to FORWARD
                            break; //Exits switch statement
                        } else if(rangeCM3 >= (rightposition - righttolerance) && rangeCM3 <= (rightposition + righttolerance)) { //If in range
                            if(integratedZ <= (straight + gyroTolerance) && integratedZ >= (straight - gyroTolerance)) { //If in range and in angle
                                setDrivePower(0, 0); //Stops robot
                                break; //Exits switch statement
                            } else { //If in range but outside angle
                                setRotationPower(-turn); //Rotates robot
                                runtime.reset(); //Resets the runtime
                                break; //Exits switch statement
                            }
                        } else { //If outside range
                            if(rangeCM3 < (rightposition - righttolerance)) { //If too close to wall
                                setStrafePower(speed, turn); //Strafes right and rotates
                                runtime.reset(); //Resets the runtime
                                break; //Exits switch statement
                            } else { //If too far from wall
                                setStrafePower(-0.26, turn); //Strafes left and rotates
                                runtime.reset(); //Resets the runtime
                                break; //Exits switch statement
                            }
                        }
                    }
*/
                case POSITIONCHECK: //Confirms exact distance from wall down to the cm
/*
                    if(!columnRangeCheckNeeded(rangeCM2)) { //Checks if robot is too close or far from cryptobox
                        setDrivePower(0, 0); //Stops robot
                        if(image == 1) {
                            CURRENT_STEP = steps.LEFTCOLUMN; //Changes step to LEFTCOLUMN
                            runtime.reset(); //Resets the runtime
                        }
                        if(image == 2) {
                            CURRENT_STEP = steps.CENTERCOLUMN; //Changes step to CENTERCLOMUN
                            runtime.reset(); //Resets the runtime
                        }
                        if(image == 3) {
                            CURRENT_STEP = steps.RIGHTCOLUMN; //Changes step to RIGHTCOLUMN
                            runtime.reset(); //Resets the runtime
                        }
                        break; //Exits switch statement
                    }

                    if(rangeCM2 < rangeCheckClose) { //If robot is too close
                        setDrivePower(-0.1, 0); //Drive backward without using gyro
                    } else { //If robot is too far
                        setDrivePower(0.1, 0); //Drive forward without using gyro
                    }

                    break; //Exits switch statement
*/
                    target = targetPosition; //Sets target position based off of variable targetPosition
                    if (rangeCM3 == target || checkTime.seconds() > 4) { //If the range reading is equal to the target or it takes longer than 4 seconds, move on
                        changeStep();
                        firstCheck = false;
                        CURRENT_STEP = steps.PRECISE_ROTATE_2;
                        break;
                    }
                    if (!firstCheck && runtime.seconds() > 0.2) { //If it has not checked for the initial starting orientation of the robot, do so (only runs once)
                        readingPrev = rangeCM3;
                        if (rangeCM3 > target) { //If the robot is too far from the wall, strafe left
                            power = -0.2;
                        }
                        if (rangeCM3 < target) { //If the robot is too close to the wall, strafe right
                            power = 0.2;
                        }
                        firstCheck = true; //Confirm step only runs once
                        runtime.reset();
                    } else { //If the first step information has been completed
                        if (runtime.seconds() > 0.5) { //Every 0.5 seconds, check to make sure the range has changed
                            if (power > 0 && rangeCM3 > target) { //If the robot is too far from the wall but is moving away from it, readjust
                                power = -0.2;
                            }
                            if (power < 0 && rangeCM3 < target) { //If the robot is too close to the wall but is moving toward it, readjust
                                power = 0.2;
                            }
                            if (rangeCM3 == readingPrev) { //If the robot's distance hasn't changed in the last 0.5 seconds, increase speed
                                power = setR3Precise(target, power);
                            }
                            readingPrev = rangeCM3;
                            runtime.reset();
                        }
                    }
                    setStrafePower(power, 0);
                    break;

                case PRECISE_ROTATE_2:

                    target = 0; //Sets target for the precise rotation
                    if ((modernRoboticsI2cGyro.getIntegratedZValue() > target - 2 && modernRoboticsI2cGyro.getIntegratedZValue() < target + 2) || checkTime.seconds() > 2) { //If the gyro is in the range of 4 degrees or the step takes more than two seconds, move on
                        changeStep();
                        CURRENT_STEP = steps.DROP_GLYPH;
                        break;
                    }
                    if (!firstCheck && runtime.seconds() > 0.2) { //If it has not checked for the initial starting orientation of the robot, do so (only runs once)
                        readingPrev = integratedZ;
                        if (integratedZ > target) { //If the gyro is reading above the target, start moving counterclockwise
                            power = -0.12;
                        }
                        if (integratedZ < target) { //If the gyro is reading below the target, start moving clockwise
                            power = 0.12;
                        }
                        firstCheck = true; //Confirms step only runs once
                        runtime.reset();
                    } else { //If the first step information has been completed
                        if (runtime.seconds() > 0.5) { //Every 0.5 seconds, check to make sure the gyro has changed
                            if (power > 0 && integratedZ > target) { //If the power is positive but the robot must turn counterclockwise, readjust
                                power = -0.12;
                            }
                            if (power < 0 && integratedZ < target) { //If the power is negative but the robot must turn clockwise, readjust
                                power = 0.12;
                            }
                            if (integratedZ == readingPrev) { //If the gyro has not changed, increase the rotation speed
                                power = setRotationPrecise(target, power);
                            }
                            readingPrev = integratedZ;
                            runtime.reset();
                        }
                    }
                    setRotationPower(power);
                    break;

                case DROP_GLYPH: //Places the glyph in the correct column

                    if (runtime.seconds() > 4) { //After four seconds, move on
                        changeStep();
                        CURRENT_STEP = steps.MOVE_RIGHT;
                        if (image == 3) { //REMOVE
                            CURRENT_STEP = steps.STOP;
                        }
                        break;
                    } else { //If four seconds isn't up yet
                        if (runtime.seconds() < 1.5) { //For first 1.5 seconds, drive forward
                            setDrivePower(0.1, 0);
                            break;
                        }
                        if (runtime.seconds() > 1.5 && runtime.seconds() < 2) { //Between 1.5 and 2 seconds, drop glyph and reverse
                            gripOpen();
                            setDrivePower(-0.1, 0);
                            break;
                        }
                        if (runtime.seconds() > 2 && runtime.seconds() < 3) { //Between 2 and 3 seconds, drive forward
                            setDrivePower(0.1, 0);
                            break;
                        }
                        if (runtime.seconds() > 3 && runtime.seconds() < 4) { //Between 3 and 4 seconds, reverse
                            setDrivePower(-0.1, 0);
                            break;
                        }
                        break;
                    }

                case MOVE_RIGHT: //Strafes to the right to avoid the balancing stone

                    if (runtime.seconds() > 1) { //Move on after 1 second of moving right
                        changeStep();
                        CURRENT_STEP = steps.FACE_PILE;
                        break;
                    }
                    setStrafePower(0.6, 0);
                    break;

                case FACE_PILE: //Rotates 150 degrees to face glyph pile

                    target = 150;
                    if (integratedZ > target - 20 && integratedZ < target + 20) { //If the robot's orientation is within 20 degrees of target, move on
                        changeStep();
                        CURRENT_STEP = steps.RESET;
                        if (matchTime.seconds() > 15) { //Conditional ending in case there is not enough time to go for second glyph
                            CURRENT_STEP = steps.BACKUP_ROTATE;
                            break;
                        }
                        break;
                    }
                    setRotationTarget(target);
                    break;

                case RESET: //Reset step to prepare looking for glyph

                    changeStep();
                    CURRENT_STEP = steps.FIND_GLYPHS;
                    break;

                case FIND_GLYPHS: //Drives forward toward glyph pile until glyphs are detected using optical distance sensors

                    if (odsLight3 > 0.025 || (odsLight1 > 0.08 || odsLight2 > 0.08) || runtime.seconds() > 4) { //If the optical distance sensors on arm crunch detect a glyph, move on
                        changeStep();
                        CURRENT_STEP = steps.SCAN_GLYPHS;
                        break;
                    }
                    m7.setPower(0.5); //Lowers arm crunch
                    gripScan(); //Sets arm crunch to scanning mode
                    setDrivePower(0.15, 0);
                    break;

                case SCAN_GLYPHS: //Strafes to the right to look for a gap in the glyph pit

                    if ((((odsLight3 == 0) || (odsLight1 > 0.05 && odsLight2 > 0.05)) && runtime.seconds() > 0.6) || runtime.seconds() > 6) { //If optical distance sensor 3 sees a gap, optical distance sensors 1 and 2 find a glyph, or it takes longer than 4 seconds, move on
                        changeStep();
                        CURRENT_STEP = steps.FORWARD_GLYPH;
                        break;
                    }
                    if (runtime.seconds() < 0.4) { //Reverses robot to give optical distance sensors better readings
                        setDrivePower(-0.08, 0);
                    } else { //Continues to strafe to the right
                        setStrafePower(0.4, 0);
                    }
                    gripScan();
                    m7.setPower(0);
                    break;

                case FORWARD_GLYPH: //Drives forward to get glyph in grasp

                    if (runtime.seconds() > 1) { //If the runtime is greater than 1 second, move on
                        changeStep();
                        CURRENT_STEP = steps.GRAB_GLYPH;
                        break;
                    }
                    gripOpen(); //Opens the arm crunch to prepare for grab
                    setDrivePower(0.2, 0);
                    break;

                case GRAB_GLYPH: //Closes servo to grab glyph

                    if (runtime.seconds() > 0.6) { //If the runtime is greater than 0.6 seconds, move on
                        changeStep();
                        CURRENT_STEP = steps.BACK_GLYPH;
                        break;
                    }
                    gripClose(); //Closes the arm crunch
                    break;

                case BACK_GLYPH: //Backs up while raising the glyph to return to the parking zone of cryptobox

                    if (runtime.seconds() > 1.5) { //If runtime is greater than 1.5 seconds, move on
                        changeStep();
                        CURRENT_STEP = steps.FACE_CRYPTO;
                        break;
                    }
                    if (runtime.seconds() < 0.6) { //If the runtime is less than 0.6 seconds, raise arm crunch with glyph
                        m7.setPower(-0.7);
                    } else {
                        m7.setPower(0);
                    }
                    setDrivePower(-0.2, 0);
                    break;

                case MOVE_LEFT: //Moves the robot to the left to better align with cryptobox

                    if (image != 3) {
                        if (runtime.seconds() > 1) {
                            changeStep();
                            CURRENT_STEP = steps.FACE_CRYPTO;
                            break;
                        }
                    } else {
                        if (runtime.seconds() > 1.5) { //If runtime is greater than 1.5 seconds, move on
                            changeStep();
                            CURRENT_STEP = steps.FACE_CRYPTO;
                            break;
                        }
                    }
                    setStrafePower(0.6, 0);
                    break;

                case FACE_CRYPTO: //Rotate the robot on a diagonal to face the cryptobox to place glyph

                    target = 50; //Sets target
                    if (integratedZ < target) {
                        changeStep();
                        CURRENT_STEP = steps.DROP_GLYPH_2;
                        break;
                    }
                    setRotationPower(-0.3);
                    break;

                case DROP_GLYPH_2: //Places the second glyph

                    if (runtime.seconds() > 3.5) { //If runtime is greater than 3.5 seconds, move on
                        changeStep();
                        CURRENT_STEP = steps.STOP;
                        break;
                    } else { //If runtime is less than 3.5 seconds
                        if (runtime.seconds() < 1.5) { //If runtime is less than 1.5 seconds, go forwards at a diagonal
                            m2.setPower(0.7);
                            m3.setPower(0.7);
                            break;
                        }
                        if (runtime.seconds() > 1.5 && runtime.seconds() < 2) { //If runtime is greater than 1.5 seconds, and less than 2 seconds, reverse at a diagonal
                            gripOpen(); //Drops glyph
                            m2.setPower(-0.4);
                            m3.setPower(-0.4);
                        }
                        if (runtime.seconds() > 2 && runtime.seconds() < 3) { //If runtime is greater than 2 seconds, and less than 3 seconds, go forwards at a diagonal
                            m2.setPower(0.7);
                            m3.setPower(0.7);
                        }
                        if (runtime.seconds() > 3 && runtime.seconds() < 3.5) { //If runtime is greater than 3 seconds, and less than 3.5 seconds, reverse at a diagonal
                            m2.setPower(-0.4);
                            m3.setPower(-0.4);
                            break;
                        }
                        break;
                    }

                case BACKUP_ROTATE:

                    if(runtime.seconds() > 1.5) {
                        changeStep();
                        CURRENT_STEP = steps.STOP;
                        break;
                    }
                    if(runtime.seconds() < 1) {
                        setStrafePower(.2,0);
                    }
                    if(runtime.seconds() > 1 && runtime.seconds() < 1.5) {
                        setDrivePower(-.2,0);
                    }

                    break;

                case STOP: //Stop all movements
                    setDrivePower(0, 0);
                    m5.setPower(0);
                    m6.setPower(0);
                    m7.setPower(0);
                    break;
            }
        }
    }
}