package org.firstinspires.ftc.teamcode; //Use the package org.firstinspires.ftc.teamcode

import android.support.annotation.VisibleForTesting;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor; //Import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor; //Import com.qualcomm.robotcore.hardware.ColorSensor for the color sensors
import com.qualcomm.robotcore.hardware.DcMotor; //Import com.qualcomm.robotcore.hardware.DcMotor for motors
import com.qualcomm.robotcore.hardware.GyroSensor; //Import com.qualcomm.robotcore.hardware.GyroSensor for the gyro sensor
import com.qualcomm.robotcore.hardware.I2cAddr; //Import com.qualcomm.robotcore.hardware.I2cAddr to allow to change I2c addresses
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor; //Import com.qualcomm.robotcore.hardware.OpticalDistanceSensor for the optical distance sensor
import com.qualcomm.robotcore.hardware.Servo; //Import com.qualcomm.robotcore.hardware.Servo for servos
import com.qualcomm.robotcore.hardware.TouchSensor; //Import com.qualcomm.robotcore.hardware.TouchSensor for touch sensors
import com.qualcomm.robotcore.eventloop.opmode.Autonomous; //Imports com.qualcomm.robotcore.eventloop.opmode.Autonomous for autonomous additions
import com.qualcomm.robotcore.eventloop.opmode.OpMode; //Imports com.qualcomm.robotcore.eventloop.opmode.OpMode for opmode additions
import com.qualcomm.robotcore.util.ElapsedTime; //Imports com.qualcomm.robotcore.util.ElapsedTime for timed events

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit; //Imports org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "AutoTestBedRed", group = "Autonomous") //Display name and group found in on controller phone
@Disabled
public class AutoTestBedRed extends AutoTestBedInit { //Imports presets for initiation from TardisOpModeAutonomous

    VuforiaLocalizer vuforia;

    public enum steps { //All steps for completing autonomous

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //Main steps - These steps run in sequence when the program is going perfect with no unexpected readings in sensor inputs.//
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            STARTCAMERA,
            SCANIMAGE,
            LOWERSERVO1,
            LOWERSERVO2,
            LOWERSERVO3,
            SENSECOLOR,
            KNOCKFORWARDS, //Knocks blue jewel
            KNOCKBACK,
            DRIVETOCRYPTOBOX,
            LEFTCOLUMN,
            CENTERCLOMUN,
            RIGHTCOLUMN,
            STOP //[Stop] The title says it all.

    } //End of steps for autonomous

    public steps CURRENT_STEP = steps.STARTCAMERA; //Sets the variable CURRENT_STEP to the first step in the sequence

    private ElapsedTime runtime = new ElapsedTime(); //Creates a variable for runtime so we can have timed events

    float image = 0;
    float straight = 0;
    double turn = 0;

    VuforiaTrackable relicTemplate;

    @Override //Method overrides parent class
    public void loop() { //Starts loop for the program

        double rangeCM1 = r1.getDistance(DistanceUnit.CM);
        double rangeCM2 = r2.getDistance(DistanceUnit.CM);
        int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue(); //Gyro integratedZ value

        ///////////////////////////
        //Telemetry for debugging//
        ///////////////////////////

        telemetry.addData("Step",  CURRENT_STEP + "\nImage" + image + "\nColorBlue: " + c1.blue()
                + "\nColorRed: " + c1.red() + "\nRange1: " + rangeCM1 + "\nRange2: " + rangeCM2); //Adds telemetry to debug
        telemetry.update(); //Updates telemetry with new information

        /////////////////////////////
        //Start of switch statement//
        /////////////////////////////

        switch (CURRENT_STEP) { //Beginning of the switch- this sets the current step to whatever CURRENT_STEP is set to

            ///////////////////////
            //START OF MAIN STEPS//
            ///////////////////////

            case STARTCAMERA:

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
                relicTemplate = relicTrackables.get(0);
                relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

                relicTrackables.activate();
                CURRENT_STEP = steps.SCANIMAGE;

            case SCANIMAGE: //Beginning of case statement START_RESET

                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

                if(image != 0) {
                    CURRENT_STEP = steps.LOWERSERVO1;
                    break; //Exits switch statement
                }

                if (vuMark == RelicRecoveryVuMark.LEFT){
                    image = 1;
                }

                if (vuMark == RelicRecoveryVuMark.CENTER){
                    image = 2;
                }

                if (vuMark == RelicRecoveryVuMark.RIGHT){
                    image = 3;
                }
                break;

            case LOWERSERVO1: //Beginning of the case statement

                m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving

                s1.setPosition(0.6);

                runtime.reset();
                CURRENT_STEP = steps.LOWERSERVO2;
                break; //Exits switch statement

            case LOWERSERVO2: //Beginning of the case statement

                m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving

                if(runtime.seconds() > .5) {
                    s1.setPosition(0.7);
                    runtime.reset();
                    CURRENT_STEP = steps.LOWERSERVO3;
                }
                break; //Exits switch statement

            case LOWERSERVO3: //Beginning of the case statement

                m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving

                if(runtime.seconds() > .5) {
                    s1.setPosition(0.73);
                    runtime.reset();
                    CURRENT_STEP = steps.SENSECOLOR;
                }
                break; //Exits switch statement

            case SENSECOLOR: //Beginning of case statement START_RESET

                if(runtime.seconds() > .5) {
                    s1.setPosition(0.75);
                }

                if(c1.blue() > 40 && runtime.seconds() > 1) {
                    CURRENT_STEP = steps.KNOCKBACK;
                    runtime.reset();
                    break; //Exits switch statement
                }

                if(c1.red() > 40 && runtime.seconds() > 1) {
                    CURRENT_STEP = steps.KNOCKFORWARDS;
                    runtime.reset();
                    break; //Exits switch statement
                }
                break;

            case KNOCKBACK: //Beginning of case statement START_RESET

                if(runtime.seconds() > 0.1){
                    CURRENT_STEP = steps.DRIVETOCRYPTOBOX;
                    break; //Exits switch statement
                }

                m1.setPower(-0.2);
                m2.setPower(-0.2);
                m3.setPower(-0.2);
                m4.setPower(-0.2);

                break;

            case KNOCKFORWARDS: //Beginning of case statement START_RESET

                if(runtime.seconds() > 0.3){
                    CURRENT_STEP = steps.DRIVETOCRYPTOBOX;
                    break; //Exits switch statement
                }
                m1.setPower(0.2);
                m2.setPower(0.2);
                m3.setPower(0.2);
                m4.setPower(0.2);

                break;

            case DRIVETOCRYPTOBOX: //Beginning of the case statement

                if (rangeCM1 < 40) {
                    m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                    m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                    m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                    m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                    CURRENT_STEP = steps.STOP;
                    break;
                }
                m1.setPower(-0.2); //Sets motor 1 power to 0 to make sure it is not moving
                m2.setPower(-0.2); //Sets motor 2 power to 0 to make sure it is not moving
                m3.setPower(-0.2); //Sets motor 3 power to 0 to make sure it is not moving
                m4.setPower(-0.2); //Sets motor 4 power to 0 to make sure it is not moving
                s1.setPosition(0);
                break; //Exits switch statement

            case LEFTCOLUMN: //Beginning of the case statement

                straight = 0;

                if (integratedZ < straight) { //Checks to see if the robot is moving and the value of straight is greater than the value integratedZ
                    turn = .05; //Sets the turn value to .05
                } else if (integratedZ > straight) { //Checks to see if the robot is moving and the value of straight is less than the value integratedZ
                    turn = -.05; //Sets the turn value to -.05
                } else { //Default value (robot is not moving)
                    turn = 0; //Sets the turn value to 0
                } //End of else statement
                if (rangeCM1 == 20 && rangeCM2 == 60) { //STOP
                    m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                    m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                    m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                    m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                    CURRENT_STEP = steps.STOP;
                    break;
                }
                if (rangeCM1 > 20 && rangeCM2 < 60) { //DIAGONAL FORWARD LEFT
                    m1.setPower(0 - turn); //Sets motor 1 power to 0 to make sure it is not moving
                    m2.setPower(0.3 + turn); //Sets motor 2 power to 0 to make sure it is not moving
                    m3.setPower(0.3 - turn); //Sets motor 3 power to 0 to make sure it is not moving
                    m4.setPower(0 + turn); //Sets motor 4 power to 0 to make sure it is not moving
                    break; //Exits switch statement
                }

                if (rangeCM1 > 20 && rangeCM2 == 60) { //FORWARD
                    m1.setPower(0.2 - turn); //Sets motor 1 power to 0 to make sure it is not moving
                    m2.setPower(0.2 + turn); //Sets motor 2 power to 0 to make sure it is not moving
                    m3.setPower(0.2 - turn); //Sets motor 3 power to 0 to make sure it is not moving
                    m4.setPower(0.2 + turn); //Sets motor 4 power to 0 to make sure it is not moving
                    break; //Exits switch statement
                }

                if (rangeCM1 > 20 && rangeCM2 > 60) { //DIAGONAL FORWARD RIGHT
                    m1.setPower(0.3 - turn); //Sets motor 1 power to 0 to make sure it is not moving
                    m2.setPower(0 + turn); //Sets motor 2 power to 0 to make sure it is not moving
                    m3.setPower(0 - turn); //Sets motor 3 power to 0 to make sure it is not moving
                    m4.setPower(0.3 + turn); //Sets motor 4 power to 0 to make sure it is not moving
                    break; //Exits switch statement
                }

                if (rangeCM1 == 20 && rangeCM2 > 60) {  //RIGHT
                    m1.setPower(0.2- turn); //Sets motor 1 power to 0 to make sure it is not moving
                    m2.setPower(-0.2 + turn); //Sets motor 2 power to 0 to make sure it is not moving
                    m3.setPower(-0.2 - turn); //Sets motor 3 power to 0 to make sure it is not moving
                    m4.setPower(0.2 + turn); //Sets motor 4 power to 0 to make sure it is not moving
                    break; //Exits switch statement
                }

                if (rangeCM1 < 20 && rangeCM2 > 60) { //DIAGONAL BACK RIGHT
                    m1.setPower(0 - turn); //Sets motor 1 power to 0 to make sure it is not moving
                    m2.setPower(-0.3 + turn); //Sets motor 2 power to 0 to make sure it is not moving
                    m3.setPower(-0.3 - turn); //Sets motor 3 power to 0 to make sure it is not moving
                    m4.setPower(0 + turn); //Sets motor 4 power to 0 to make sure it is not moving
                    break; //Exits switch statement
                }

                if (rangeCM1 < 20 && rangeCM2 == 60) { //BACK
                    m1.setPower(-0.2 - turn); //Sets motor 1 power to 0 to make sure it is not moving
                    m2.setPower(-0.2 + turn); //Sets motor 2 power to 0 to make sure it is not moving
                    m3.setPower(-0.2 - turn); //Sets motor 3 power to 0 to make sure it is not moving
                    m4.setPower(-0.2 + turn); //Sets motor 4 power to 0 to make sure it is not moving
                    break; //Exits switch statement
                }

                if (rangeCM1 < 20 && rangeCM2 < 60) { //DIAGONAL BACK LEFT
                    m1.setPower(-0.3 - turn); //Sets motor 1 power to 0 to make sure it is not moving
                    m2.setPower(0 + turn); //Sets motor 2 power to 0 to make sure it is not moving
                    m3.setPower(0 - turn); //Sets motor 3 power to 0 to make sure it is not moving
                    m4.setPower(-0.3 + turn); //Sets motor 4 power to 0 to make sure it is not moving
                    break; //Exits switch statement
                }
                if (rangeCM1 == 20 && rangeCM2 < 60) {  //LEFT
                    m1.setPower(-0.2 - turn); //Sets motor 1 power to 0 to make sure it is not moving
                    m2.setPower(0.2 + turn); //Sets motor 2 power to 0 to make sure it is not moving
                    m3.setPower(0.2 - turn); //Sets motor 3 power to 0 to make sure it is not moving
                    m4.setPower(-0.2 + turn); //Sets motor 4 power to 0 to make sure it is not moving
                    break; //Exits switch statement
                }
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
