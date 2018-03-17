//TeleOp
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
//Imports Files for robot parts

@TeleOp(name = "TeleOp2018", group = "TeleOp")
public class TeleOp2018 extends OpMode {

    DcMotor m1; //Front left motor
    DcMotor m2; //Front right motor
    DcMotor m3; //Back left motor
    DcMotor m4; //Back right motor
    DcMotor m5; //Arm raise motor
    DcMotor m6; //Arm base rotation motor
    DcMotor m7; //Arm crunch vertical motor

    Servo s1; //Color sensor arm servo
    Servo s2; //Relic Claw servo
    Servo s3; //Arm Crunch A
    Servo s4; //Arm Crunch B
    Servo s6; //Arm extension
    Servo s7; //Backup gripper

    IntegratingGyroscope gyro; //Gyro
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    ColorSensor c1; //Color sensor

    I2cDeviceSynch r1reader;
    ModernRoboticsI2cRangeSensor r1; //Right range sensor

    I2cDeviceSynch r2reader;
    ModernRoboticsI2cRangeSensor r2; //Front range sensor

    I2cDeviceSynch r3reader;
    ModernRoboticsI2cRangeSensor r3; //Left range sensor

    I2cDeviceSynch r4reader;
    ModernRoboticsI2cRangeSensor r4; //Back range sensor

    TouchSensor touchSensor1; //Define touch sensor as touchSensor1 (mast limit switch)

    ElapsedTime runtime; //Time variable
    ElapsedTime runtime2; //Time variabe for slight opening of arm crunch and relic claw

    float straight = 0; //Variable for straight motion
    double turn = 0; //Variable for turn
    boolean relicPos; //Sets relic gripper position

    public void init() { //Start of the initiation for teleop

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

        m2.setDirection(DcMotor.Direction.REVERSE); //Sets m2 direction to REVERSE
        m4.setDirection(DcMotor.Direction.REVERSE); //Sets m4 direction to REVERSE

        touchSensor1 = hardwareMap.touchSensor.get("t1"); //Sets touchSensor1 to t1 in the config

        runtime = new ElapsedTime(); //Creates runtime variable for using time
        runtime2 = new ElapsedTime(); //Creates runtime variable for using time

        s1.setPosition(0); //Pulls jewel appendage against side of robot
        s2.setPosition(.3); //Opens Relic Claw
        s3.setPosition(0.2); //Sets Arm Crunch Servo A
        s4.setPosition(1); //Sets Arm Crunch Servo B
        s6.setPosition(0.5); //Sets arm extension to not move
        s7.setPosition(0.9); //Sets second relic claw to tuck in

    } //Ends initiation

    @Override
    public void loop() {  //Start of main loop

        //Sets variables for gamepad 1 joysticks
        double LUD1 = gamepad1.left_stick_y;
        double LRL1 = -gamepad1.left_stick_x * 1.5; //Multiplied by 1.5 to increase strafe speed
        double RUD1 = gamepad1.right_stick_y;
        double RLR1 = -gamepad1.right_stick_x;

        //Sets variables for gamepad 2 joysticks
        double LUD2 = gamepad2.left_stick_y;
        double LRL2 = -gamepad2.left_stick_x * 1.5; //Multiplied by 1.5 to increase strafe speed
        double RUD2 = gamepad2.right_stick_y;
        double RLR2 = gamepad2.right_stick_x;

        //Controls for Relic Claw (Controller 2)
        if (gamepad2.left_trigger != 0) { //If left trigger is pressed, open claw
            s2.setPosition(0.7); //Opens claw
            relicPos = false;
        } else if (gamepad2.x) { //If x is pressed, move relic claw out of way for second claw
            s2.setPosition(1);
            relicPos = true;
        } else if (relicPos) { //After x is pressed, hold relic claw out of way
            s2.setPosition(1);
        } else { //If not pressed, close claw
            s2.setPosition(0.3); //Closes claw
            runtime2.reset();
        }

        //Controls for second relic claw (Controller 2)
        if (gamepad2.y) { //If y is pressed, close claw
            s7.setPosition(0); //Servo closes
        } else { //If y is nt pressed, tuck claw out of way
            s7.setPosition(0.9);
        }

        //Controls the raising and lower the arm crunch mast. (Controller 2)
        if (LUD2 > 0.1 && !touchSensor1.isPressed()) { //If the y axis is raised
            m7.setPower(LUD2 / 2); //Raise the arm mast
        } else if (LUD2 < -0.1) { //If the y axis is lowered
            m7.setPower(LUD2); //Lower the arm mast
        } else {
            m7.setPower(0); //If stick isn't touched, do not move.
        }

        //Controls for rotating arm base (Controller 2)
        if (RLR2 < -.1 || RLR2 > 0.1) { //If the x axis of right stick is pressed, move arm base
            m6.setPower(RLR2 / 2); //Sets motor power to half of the right joystick
        } else { //If the x axis of right stick is not pressed, hold at current position
            m6.setPower(0); //Sets motor power to 0
        }

        //Controls for arm raise motor (controller 2)
        if (RUD2 < -0.1 || RUD2 > 0.1) { //If the y axis of left stick is pressed, raise arm
            m5.setPower(-RUD2); //Sets motor power to joystick speed
        } else { //If the y axis of left stick is not pressed, hold at current position
            m5.setPower(0); //Sets motor power to 0
        }

        if (gamepad2.left_bumper){ //Extend arm servo
            s6.setPosition(1);
        } else if (gamepad2.right_bumper) { //Retract arm servo
            s6.setPosition(0);
        } else { //Dont move arm servo
            s6.setPosition(0.5);
        }

        //Controls for arm crunch (Controller 2)
        if (gamepad2.right_trigger != 0) { //If right trigger is pressed, close claw
            s3.setPosition(0.76); //Sets servo position to 0.36
            s4.setPosition(0.45); //Sets servo position to 0.58
            runtime.reset();
        } else if (gamepad2.right_trigger == 0 && runtime.seconds() < 0.8) {
            s3.setPosition(0.57); //Sets servo position to 0.36
            s4.setPosition(0.6); //Sets servo position to 0.58
        } else { //If not pressed, open arm crunch
            s3.setPosition(0.47); //Sets servo position to 1
            s4.setPosition(0.75); //Sets servo position to 1
        }

        //Controls for drive train (Controller 1)
        if (gamepad1.right_trigger == 0) { //Controls for slow mode (default)0
            m1.setPower(((LRL1 + LUD1) / 3) + (RLR1 / 2) - turn); //Steering for top left
            m2.setPower(((LUD1 - LRL1) / 3) - (RLR1 / 2) + turn); //Steering for top right
            m3.setPower(((LUD1 - LRL1) / 3) + (RLR1 / 2) - turn); //Steering for back left
            m4.setPower(((LRL1 + LUD1) / 3) - (RLR1 / 2) + turn); //Steering for back right
        } else { //Controls for normal mode
            m1.setPower(((LRL1 + LUD1) / 1.5) + (RLR1 / 1.2) - turn); //Steering for top left
            m2.setPower(((LUD1 - LRL1) / 1.5) - (RLR1 / 1.2) + turn); //Steering for top right
            m3.setPower(((LUD1 - LRL1) / 1.5) + (RLR1 / 1.2) - turn); //Steering for back left
            m4.setPower(((LRL1 + LUD1) / 1.5) - (RLR1 / 1.2) + turn); //Steering for back right
        }
    }
}