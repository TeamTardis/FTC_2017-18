package org.firstinspires.ftc.teamcode;
/**
 * Created by Corning Robotics on 9/25/16.
 */
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static com.sun.tools.javac.util.Constants.format;
//Imports

@TeleOp(name="MecanumTestBed", group="TeleOp")
public class MecanumTestBed extends OpMode {
    DcMotor m1; //Define dcMotor as m1
    DcMotor m2; //Define dcMotor as m2
    DcMotor m3; //Define dcMotor as m3
    DcMotor m4; //Define dcMotor as m4
    DcMotor m5; //Define dcMotor as m4
    DcMotor m6; //Define dcMotor as m4
    Servo s1; //Color sensor arm servo
    Servo s2; //Claw grip servo
    Servo s3; //Wrist rotation
    Servo s4; //Claw vertical
    Servo s5; //Second claw grip
    Servo s6; //Arm extension
/*
    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    ColorSensor c1;
    I2cDeviceSynch r2reader;
    ModernRoboticsI2cRangeSensor r2;

    I2cDeviceSynch r1reader;
    ModernRoboticsI2cRangeSensor r1;

    I2cDeviceSynch r3reader;
    ModernRoboticsI2cRangeSensor r3;
*/
    public void init() { //Start of the initiation for autonomous

        m1 = hardwareMap.dcMotor.get("m1"); //Sets m1 to m3 in the config
        m2 = hardwareMap.dcMotor.get("m2"); //Sets m2 to m1 in the config
        m3 = hardwareMap.dcMotor.get("m3"); //Sets m3 to m4 in the config
        m4 = hardwareMap.dcMotor.get("m4"); //Sets m4 to m2 in the config
        m5 = hardwareMap.dcMotor.get("m5"); //Sets m4 to m2 in the config
        m6 = hardwareMap.dcMotor.get("m6"); //Sets m4 to m2 in the config

        s1 = hardwareMap.servo.get("s1"); //Sets s1 i the config
        s2 = hardwareMap.servo.get("s2"); //Sets s1 i the config
        s3 = hardwareMap.servo.get("s3"); //Sets s1 i the config
        s4 = hardwareMap.servo.get("s4"); //Sets s1 i the config
        s5 = hardwareMap.servo.get("s5"); //Sets s1 i the config
        s6 = hardwareMap.servo.get("s6"); //Sets s1 i the config

        m2.setDirection(DcMotor.Direction.REVERSE);
        m4.setDirection(DcMotor.Direction.REVERSE);

        m5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
/*
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;
        modernRoboticsI2cGyro.calibrate();

        c1 = hardwareMap.colorSensor.get("c1"); //Sets colorSensor to c1 in the config
        c1.enableLed(true); //Turns Color Sensor LED off

        while (modernRoboticsI2cGyro.isCalibrating()) {
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
        r3 = new ModernRoboticsI2cRangeSensor(r2reader);
        r3.setI2cAddress(I2cAddr.create8bit(0x2c));
 */
    } //Ends initiation

    ElapsedTime runtime = new ElapsedTime();

    float straight = 0;
    double turn = 0;
    double wristPosition = .45;
    double power;
    float encoderCurrent;
    double encoderDifference;
    float encoderTarget = 0;

    /*    double limitToOne(double in) //Creates variable for driving vector
        {
            if(in < -1)
            {
                return -1;
            }
            if(in > 1)
            {
                return 1;
            }

            return in;
        }
    */
    @Override
    public void loop() {

        /*
        int heading = modernRoboticsI2cGyro.getHeading(); //Gyro heading value
        int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue(); //Gyro integratedZ value
        */

        encoderCurrent = m5.getCurrentPosition();

        telemetry.addData("m5 curr position: ",encoderCurrent + "\nencoderDifference: " + encoderDifference + "\nencoderTarget: " + encoderTarget);

        float LUD = gamepad1.left_stick_y; //Variable for left stick y axis
        float LRL = -gamepad1.left_stick_x; //Variable for left stick x axis
        float RUD = gamepad1.right_stick_y; //Variable for right stick y axis
        float RLR = -gamepad1.right_stick_x; //Variable for right stick x axix

        if (gamepad2.right_trigger != 0) {
            s2.setPosition(0.6);
        } else {
            s2.setPosition(1);
        }
        if (gamepad2.left_trigger != 0) {
            s5.setPosition(0);
        } else {
            s5.setPosition(1);
        }

        if (gamepad2.x) {
            s4.setPosition(0);
        } else if (gamepad2.y) {
            s4.setPosition(1);
        } else {
            s4.setPosition(0.5);
        }

        if (gamepad2.right_stick_x < .1) {
            m6.setPower(gamepad2.right_stick_x / 3);
        } else if (gamepad2.right_stick_x > .1) {
            m6.setPower(gamepad2.right_stick_x / 3);
        } else {
            m6.setPower(0);
        }

        if (gamepad2.left_stick_y < .3) {
            encoderTarget = encoderTarget + 5;
        }
        if (gamepad2.left_stick_y > .3) {
            encoderTarget = encoderTarget - 5;
        }

        encoderDifference = encoderCurrent - encoderTarget;
        power = ((Math.pow(encoderDifference, 3) * -.0000015) + (-.007 * encoderDifference)) / 2;

        m5.setPower(power);

        if (gamepad2.a) {
            wristPosition += .01;
        }
        if (gamepad2.b) {
            wristPosition -= .01;
        }

        if(gamepad2.left_bumper) {
            s6.setPosition(1);
        } else if(gamepad2.right_bumper) {
            s6.setPosition(0);
        } else {
            s6.setPosition(.5);
        }

        double stest = wristPosition;
        s3.setPosition(wristPosition);
        /*
        if (RLR != 0) { //Checks if the robot is rotating
            straight = integratedZ; //If rotating, then set the straight varible to the integratedZ value
        } //End of if statement
        */
        //if (integratedZ < straight && LUD != 0 && LRL != 0) { //Checks to see if the robot is moving and the value of straight is greater than the value integratedZ
        //    turn = .4; //Sets the turn value to .4
        //} else if (integratedZ > straight && LUD != 0 && LRL != 0) { //Checks to see if the robot is moving and the value of straight is less than the value integratedZ
        //    turn = -.4; //Sets the turn value to -.4
        //} else { //Default value (robot is not moving)
        turn = 0; //Sets the turn value to 0
        //} //End of else statement

        //telemetry.addData("", "Gyro ZValue: " + integratedZ + "\nGyro Straight:" + straight); //Adds telemetry for debugging
        //telemetry.update(); //Updates telemetry

        if (gamepad1.right_trigger == 0) { //Controls for normal mode

//            if (RLR == 0) {
                m1.setPower(((LRL + LUD) / 8) + (RLR / 6) - turn); //Steering for top left
                m2.setPower(((LUD - LRL) / 8) - (RLR / 6) + turn); //Steering for top right
                m3.setPower(((LUD - LRL) / 8) + (RLR / 6) - turn); //Steering for back left
                m4.setPower(((LRL + LUD) / 8) - (RLR / 6) + turn); //Steering for back right
/*            } else {
                m1.setPower(RLR); //Turning for top left motor
                m2.setPower(-RLR); //Turning for top right motor
                m3.setPower(RLR); //Turning for back left motor
                m4.setPower(-RLR); //Turning for back right motor
            }
*/
/*
                int facingDeg = -45 - heading; //Robot's rotation
                double facingRad = Math.toRadians(facingDeg); // Convert to radians

                double cs = Math.cos(facingRad);
                double sn = Math.sin(facingRad);

                double headX = LRL * cs - LUD * sn; //Rotated vector (Relative heading)
                double headY = LRL * sn + LUD * cs; //Each is in range -root 2 to root 2

                headX /= Math.sqrt(2); //In range -1 to 1
                headY /= Math.sqrt(2);

                telemetry.addData("absHead", "(" + LRL + ", " + LUD + ")");
                telemetry.addData("head", heading);
                telemetry.addData("relHead", "(" + headX + ", " + headY + ")");


        double frontLeftPower = limitToOne(-headY + stickRot);
        double frontRightPower = limitToOne(headX + stickRot);
        double backLeftPower = limitToOne(-headX + stickRot);
        double backRightPower = limitToOne(headY + stickRot);


        if(gamepad1.right_trigger == 0) { //Controls for normal mode

            m1.setPower(frontLeftPower);
            m2.setPower(frontRightPower);
            m3.setPower(backLeftPower);
            m4.setPower(backRightPower);
*/
        } else { //Controls for slow mode

//            if (RLR == 0) {
                m1.setPower(((LRL + LUD) / 4) + RLR - turn); //Steering for top left
                m2.setPower(((LUD - LRL) / 4) - RLR + turn); //Steering for top right
                m3.setPower(((LUD - LRL) / 4) + RLR - turn); //Steering for back left
                m4.setPower(((LRL + LUD) / 4) - RLR + turn); //Steering for back right
/*            } else {
                m1.setPower(RLR / 4); //Turning for top left motor
                m2.setPower(-RLR / 4); //Turning for top right motor
                m3.setPower(RLR / 4); //Turning for back left motor
                m4.setPower(-RLR / 4); //Turning for back right motor
            }
*/

/*            m1.setPower(frontLeftPower / 2);
            m2.setPower(frontRightPower / 2);
            m3.setPower(backLeftPower / 2);
            m4.setPower(backRightPower / 2);
*/

        }
    }
}

