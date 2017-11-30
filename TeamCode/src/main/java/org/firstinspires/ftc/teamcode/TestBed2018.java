package org.firstinspires.ftc.teamcode;
/**
 * Created by Corning Robotics on 9/25/16.
 */
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

import java.math.RoundingMode;
import java.text.DecimalFormat;

import static com.sun.tools.javac.util.Constants.format;
//Imports

@TeleOp(name="TestBed", group="TeleOp")
//@Disabled
public class TestBed2018 extends OpMode {
    DcMotor m1; //Define dcMotor as m1
    DcMotor m2; //Define dcMotor as m2
    DcMotor m3; //Define dcMotor as m3
    DcMotor m4; //Define dcMotor as m4
    DcMotor m5; //Define dcMotor as m4

    Servo s1; //Claw grip servo
    Servo s2; //Claw rotation servo

    public void init() { //Start of the initiation for autonomous

        m1 = hardwareMap.dcMotor.get("m1"); //Sets m1 to m3 in the config
        m2 = hardwareMap.dcMotor.get("m2"); //Sets m2 to m1 in the config
        m3 = hardwareMap.dcMotor.get("m3"); //Sets m3 to m4 in the config
        m4 = hardwareMap.dcMotor.get("m4"); //Sets m4 to m2 in the config
        m5 = hardwareMap.dcMotor.get("m5"); //Sets m4 to m2 in the config

        s1 = hardwareMap.servo.get("s1"); //Sets s1 i the config
        s2 = hardwareMap.servo.get("s2"); //Sets s1 i the config

        m2.setDirection(DcMotor.Direction.REVERSE);
        m4.setDirection(DcMotor.Direction.REVERSE);

    } //Ends initiation

    double turn = 0;
    double power;
    float encoderCurrent;
    double encoderDifference;
    float encoderTarget = 0;
    double wristRotation = 0;

    @Override
    public void loop() {

        encoderCurrent = m5.getCurrentPosition();
        wristRotation = -0.000475586557*encoderCurrent;

        telemetry.addData("m5 curr position: ",encoderCurrent + "\ns2 curr position1: " + wristRotation);

        float LUD = gamepad1.left_stick_y; //Variable for left stick y axis
        float LRL = -gamepad1.left_stick_x; //Variable for left stick x axis
        float RUD = gamepad1.right_stick_y; //Variable for right stick y axis
        float RLR = -gamepad1.right_stick_x; //Variable for right stick x axis

        if (gamepad1.right_trigger != 0) {
            s1.setPosition(1); //Set servo position to 0.8
        } else {
            s1.setPosition(0); //Set servo position to 1
        }

        if (gamepad1.a) {
            m5.setPower(.3);
        } else if (gamepad1.b) {
            m5.setPower(-.1);
        } else {
            m5.setPower(0);
        }

        s2.setPosition(wristRotation);

        if (gamepad1.right_trigger == 0) { //Controls for slow mode

            m1.setPower(((LRL + LUD) / 4) + (RLR / 2) - turn); //Turning for top left
            m2.setPower(((LUD - LRL) / 4) - (RLR / 2) + turn); //Steering for top right
            m3.setPower(((LUD - LRL) / 4) + (RLR / 2) - turn); //Steering for back left
            m4.setPower(((LRL + LUD) / 4) - (RLR / 2) + turn); //Steering for back right

        } else { //Controls for fast mode

            m1.setPower(((LRL + LUD) / 2) + RLR - turn); //Steering for top left
            m2.setPower(((LUD - LRL) / 2) - RLR + turn); //Steering for top right
            m3.setPower(((LUD - LRL) / 2) + RLR - turn); //Steering for back left
            m4.setPower(((LRL + LUD) / 2) - RLR + turn); //Steering for back right

        }
    }
}