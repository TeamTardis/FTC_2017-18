package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.teamcode.AutoSteps.steps.KNOCKBACK;
import static org.firstinspires.ftc.teamcode.AutoSteps.steps.KNOCKFORWARDS;
//Imports

@Autonomous(name="RedCorner", group ="Teleop")

public class RedCorner extends AutoSteps {

    public static final String TAG = "RedCorner";

    VuforiaLocalizer vuforia;

    DcMotor m1; //Define dcMotor as m1
    DcMotor m2; //Define dcMotor as m2
    DcMotor m3; //Define dcMotor as m3
    DcMotor m4; //Define dcMotor as m4
    DcMotor m5; //Define dcMotor as m5
    Servo s1; //Color sensor arm servo
    Servo s2; //Claw grip servo
    Servo s3; //Wrist rotation
//    Servo s4; //Claw vertical

    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    ColorSensor c1;

    I2cDeviceSynch r1reader;
    ModernRoboticsI2cRangeSensor r1;

    I2cDeviceSynch r2reader;
    ModernRoboticsI2cRangeSensor r2;

    I2cDeviceSynch r3reader;
    ModernRoboticsI2cRangeSensor r3;

    @Override
    public void runOpMode() {

        m1 = hardwareMap.dcMotor.get("m1"); //Sets m1 in the config
        m2 = hardwareMap.dcMotor.get("m2"); //Sets m2 in the config
        m3 = hardwareMap.dcMotor.get("m3"); //Sets m3 in the config
        m4 = hardwareMap.dcMotor.get("m4"); //Sets m4 in the config
        m5 = hardwareMap.dcMotor.get("m5"); //Sets m4 in the config

        s1 = hardwareMap.servo.get("s1"); //Sets s1 i the config
        s2 = hardwareMap.servo.get("s2"); //Sets s1 i the config
        s3 = hardwareMap.servo.get("s3"); //Sets s1 i the config
//        s4 = hardwareMap.servo.get("s4"); //Sets s1 i the config

        m1.setDirection(DcMotor.Direction.REVERSE);
        m3.setDirection(DcMotor.Direction.REVERSE);


        s1.setPosition(0);
        s2.setPosition(0);
        s3.setPosition(0.45);
//        s4.setPosition(0.5);

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;

        c1 = hardwareMap.colorSensor.get("c1"); //Sets colorSensor to c1 in the config
        c1.enableLed(true); //Turns Color Sensor LED off

        modernRoboticsI2cGyro.calibrate();

        while(modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("", "Gyro Calibrating. Please wait...");
            telemetry.update();
        }
        telemetry.addData("", "Gyro Calibrated. Good luck!");
        telemetry.update();

        r1reader = hardwareMap.i2cDeviceSynch.get("r1");
        r1 = new ModernRoboticsI2cRangeSensor(r1reader);
        r1.setI2cAddress(I2cAddr.create8bit(0x2a));

        r2reader = hardwareMap.i2cDeviceSynch.get("r2");
        r2 = new ModernRoboticsI2cRangeSensor(r2reader);
        r2.setI2cAddress(I2cAddr.create8bit(0x28));

        r3reader = hardwareMap.i2cDeviceSynch.get("r3");
        r3 = new ModernRoboticsI2cRangeSensor(r3reader);
        r3.setI2cAddress(I2cAddr.create8bit(0x2c));

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

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        relicTrackables.activate();

        VuforiaLocalizer vuforia;

        steps CURRENT_STEP = steps.SCANIMAGE; //Sets the variable CURRENT_STEP to the first step in the sequence

        ElapsedTime runtime = new ElapsedTime(); //Creates a variable for runtime so we can have timed events

        float image = 0;
        float straight = 0;
        double turn = 0;
        double speed = 0;

        waitForStart();

        while (opModeIsActive()) {

            double rangeCM1 = r1.getDistance(DistanceUnit.CM);
            double rangeCM2 = r2.getDistance(DistanceUnit.CM);
            int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue(); //Gyro integratedZ value

            ///////////////////////////
            //Telemetry for debugging//
            ///////////////////////////

            telemetry.addData("Step",  CURRENT_STEP + "\nImage" + image + "\nColorBlue: " + c1.blue()
                    + "\nColorRed: " + c1.red() + "\nRange1: " + rangeCM1); //Adds telemetry to debug
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

                    if(image != 0) {
                        CURRENT_STEP = steps.LOWERSERVO;
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

                case LOWERSERVO: //Beginning of the case statement

                    m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                    m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                    m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                    m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving

                    s1.setPosition(0.7);
                    runtime.reset();
                    CURRENT_STEP = steps.SENSECOLOR;

                    break; //Exits switch statement

                case SENSECOLOR: //Beginning of case statement START_RESET

                    if(runtime.seconds() > .5) {
                        s1.setPosition(0.82);
                    }

                    if(c1.blue() > 10 && runtime.seconds() > 1) {
                        CURRENT_STEP = KNOCKFORWARDS;
                        runtime.reset();
                        break; //Exits switch statement
                    }

                    if(c1.red() > 10 && runtime.seconds() > 1) {
                        CURRENT_STEP = steps.KNOCKBACK;
                        runtime.reset();
                        break; //Exits switch statement
                    }
                    break;

                case KNOCKBACK: //Beginning of case statement START_RESET

                    if(runtime.seconds() > 0.3){
                        m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                        CURRENT_STEP = steps.RAISESERVO;
                        break; //Exits switch statement
                    }

                    m1.setPower(-0.2);
                    m2.setPower(-0.2);
                    m3.setPower(-0.2);
                    m4.setPower(-0.2);

                    break;

                case KNOCKFORWARDS: //Beginning of case statement START_RESET

                    if(runtime.seconds() > 0.3){
                        m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                        CURRENT_STEP = steps.RAISESERVO;
                        break; //Exits switch statement
                    }
                    m1.setPower(0.2);
                    m2.setPower(0.2);
                    m3.setPower(0.2);
                    m4.setPower(0.2);

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

                    if (rangeCM2 < 40) {
                        m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                        if(image == 1) {
                            CURRENT_STEP = steps.LEFTCOLUMN;
                        }
                        if(image == 2) {
                            CURRENT_STEP = steps.CENTERCLOMUN;
                        }
                        if(image == 3) {
                            CURRENT_STEP = steps.RIGHTCOLUMN;
                        }
                        break;
                    }
                    m1.setPower(0.15); //Sets motor 1 power to 0 to make sure it is not moving
                    m2.setPower(0.15); //Sets motor 2 power to 0 to make sure it is not moving
                    m3.setPower(0.15); //Sets motor 3 power to 0 to make sure it is not moving
                    m4.setPower(0.15); //Sets motor 4 power to 0 to make sure it is not moving
                    s1.setPosition(0);
                    break; //Exits switch statement

                case LEFTCOLUMN: //Beginning of the case statement

                    speed = (Math.pow(.971, rangeCM1))*.898;

                    straight = 0;

                    if (integratedZ < straight) { //Checks to see if the robot is moving and the value of straight is greater than the value integratedZ
                        turn = .05; //Sets the turn value to .05
                    } else if (integratedZ > straight) { //Checks to see if the robot is moving and the value of straight is less than the value integratedZ
                        turn = -.05; //Sets the turn value to -.05
                    } else { //Default value (robot is not moving)
                        turn = 0; //Sets the turn value to 0
                    } //End of else statement

                    if (rangeCM1 == 70) { //STOP
                        m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                        CURRENT_STEP = steps.STOP;
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
                        m1.setPower(0.1 - turn); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(-0.1 + turn); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(-0.1 - turn); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0.1 + turn); //Sets motor 4 power to 0 to make sure it is not moving
                        break; //Exits switch statement
                    }
                    /*
                    if (rangeCM2 == 20 && rangeCM1 == 75) { //STOP
                        m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                        CURRENT_STEP = steps.STOP;
                        break;
                    }
                    if (rangeCM2 > 20 && rangeCM1 < 75) { //DIAGONAL FORWARD LEFT
                        m1.setPower(0 - turn); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0.15 + turn); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0.15 - turn); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0 + turn); //Sets motor 4 power to 0 to make sure it is not moving
                        break; //Exits switch statement
                    }

                    if (rangeCM2 > 20 && rangeCM1 == 75) { //FORWARD
                        m1.setPower(0.1 - turn); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0.1 + turn); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0.1 - turn); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0.1 + turn); //Sets motor 4 power to 0 to make sure it is not moving
                        break; //Exits switch statement
                    }

                    if (rangeCM2 > 20 && rangeCM1 > 75) { //DIAGONAL FORWARD RIGHT
                        m1.setPower(0.15 - turn); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0 + turn); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0 - turn); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0.15 + turn); //Sets motor 4 power to 0 to make sure it is not moving
                        break; //Exits switch statement
                    }

                    if (rangeCM2 == 20 && rangeCM1 > 75) {  //RIGHT
                        m1.setPower(0.1 - turn); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(-0.1 + turn); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(-0.1 - turn); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0.1 + turn); //Sets motor 4 power to 0 to make sure it is not moving
                        break; //Exits switch statement
                    }

                    if (rangeCM2 < 20 && rangeCM1 > 75) { //DIAGONAL BACK RIGHT
                        m1.setPower(0 - turn); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(-0.15 + turn); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(-0.15 - turn); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0 + turn); //Sets motor 4 power to 0 to make sure it is not moving
                        break; //Exits switch statement
                    }

                    if (rangeCM2 < 20 && rangeCM1 == 75) { //BACK
                        m1.setPower(-0.1 - turn); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(-0.1 + turn); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(-0.1 - turn); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(-0.1 + turn); //Sets motor 4 power to 0 to make sure it is not moving
                        break; //Exits switch statement
                    }

                    if (rangeCM2 < 20 && rangeCM1 < 75) { //DIAGONAL BACK LEFT
                        m1.setPower(-0.15 - turn); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0 + turn); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0 - turn); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(-0.15 + turn); //Sets motor 4 power to 0 to make sure it is not moving
                        break; //Exits switch statement
                    }
                    if (rangeCM2 == 20 && rangeCM1 < 75) {  //LEFT
                        m1.setPower(-0.1 - turn); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0.1 + turn); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0.1 - turn); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(-0.1 + turn); //Sets motor 4 power to 0 to make sure it is not moving
                        break; //Exits switch statement
                    }
                    */
                    break;

                case CENTERCLOMUN: //Beginning of the case statement STOP

                    speed = (Math.pow(.966, rangeCM1))*.8;

                    straight = 0;

                    if (integratedZ < straight) { //Checks to see if the robot is moving and the value of straight is greater than the value integratedZ
                        turn = .05; //Sets the turn value to .05
                    } else if (integratedZ > straight) { //Checks to see if the robot is moving and the value of straight is less than the value integratedZ
                        turn = -.05; //Sets the turn value to -.05
                    } else { //Default value (robot is not moving)
                        turn = 0; //Sets the turn value to 0
                    } //End of else statement

                    if (rangeCM1 == 50) { //STOP
                        m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                        CURRENT_STEP = steps.STOP;
                        break;
                    }

                    if (rangeCM1 < 50) { //LESS THAN (left)
                        m1.setPower(-speed - turn); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(speed + turn); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(speed - turn); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(-speed + turn); //Sets motor 4 power to 0 to make sure it is not moving
                        break; //Exits switch statement
                    }

                    if (rangeCM1 > 50) {
                        m1.setPower(0.1 - turn); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(-0.1 + turn); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(-0.1 - turn); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0.1 + turn); //Sets motor 4 power to 0 to make sure it is not moving
                        break; //Exits switch statement
                    }
                    break; //Exits switch statement

                case RIGHTCOLUMN: //Beginning of the case statement STOP

                    speed = (Math.pow(.957, rangeCM1))*.722;

                    straight = 0;

                    if (integratedZ < straight) { //Checks to see if the robot is moving and the value of straight is greater than the value integratedZ
                        turn = .05; //Sets the turn value to .05
                    } else if (integratedZ > straight) { //Checks to see if the robot is moving and the value of straight is less than the value integratedZ
                        turn = -.05; //Sets the turn value to -.05
                    } else { //Default value (robot is not moving)
                        turn = 0; //Sets the turn value to 0
                    } //End of else statement

                    if (rangeCM1 == 35) { //STOP
                        m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                        CURRENT_STEP = steps.STOP;
                        break;
                    }

                    if (rangeCM1 < 35) { //LESS THAN (left)
                        m1.setPower(-speed - turn); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(speed + turn); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(speed - turn); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(-speed + turn); //Sets motor 4 power to 0 to make sure it is not moving
                        break; //Exits switch statement
                    }

                    if (rangeCM1 > 35) {
                        m1.setPower(0.1 - turn); //Sets motor 1 power to 0 to make sure it is not moving
                        m2.setPower(-0.1 + turn); //Sets motor 2 power to 0 to make sure it is not moving
                        m3.setPower(-0.1 - turn); //Sets motor 3 power to 0 to make sure it is not moving
                        m4.setPower(0.1 + turn); //Sets motor 4 power to 0 to make sure it is not moving
                        break; //Exits switch statement
                    }

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

