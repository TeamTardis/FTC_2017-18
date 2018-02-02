package org.firstinspires.ftc.teamcode;
/**
 * Created by Corning Robotics on 9/25/16.
 */

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//Imports

@TeleOp(name = "BoyScout_87", group = "TeleOp")
public class BoyScout_87 extends OpMode {
    DcMotor mFL; //Define dcMotor as m1
    DcMotor mFR; //Define dcMotor as m2
    DcMotor mBL; //Define dcMotor as m3
    DcMotor mBR; //Define dcMotor as m4
    DcMotor m5; //Define dcMotor as m4

    Servo claw; //Claw grip servo

    public void init() { //Start of the initiation for autonomous

        mFL = hardwareMap.dcMotor.get("m1"); //Sets m1 to m3 in the config
        mFR = hardwareMap.dcMotor.get("m2"); //Sets m2 to m1 in the config
        mBL = hardwareMap.dcMotor.get("m3"); //Sets m3 to m4 in the config
        mBR = hardwareMap.dcMotor.get("m4"); //Sets m4 to m2 in the config
        m5 = hardwareMap.dcMotor.get("m5"); //Sets m4 to m2 in the config

        claw = hardwareMap.servo.get("s1"); //Sets s1 i the config

        mFR.setDirection(DcMotor.Direction.REVERSE);
        mBR.setDirection(DcMotor.Direction.REVERSE);

        claw.setPosition(0);

    } //Ends initiation


    @Override
    public void loop() {

        float LUD = gamepad1.left_stick_y; //Variable for left stick y axis
        float LRL = -gamepad1.left_stick_x; //Variable for left stick x axis
        float RUD = gamepad1.right_stick_y; //Variable for right stick y axis
        float RLR = -gamepad1.right_stick_x; //Variable for right stick x axis



        if (gamepad1.right_trigger != 0){
            claw.setPosition(0);
        } else {
            claw.setPosition(1);
        }

        mFL.setPower(((LRL + LUD) / 2) + (RLR / 4));
        mFR.setPower(((LUD - LRL) / 2) - (RLR / 4));
        mBL.setPower(((LUD - LRL) / 2) + (RLR / 4));
        mBR.setPower(((LRL + LUD) / 2) - (RLR / 4));

    }
}






















