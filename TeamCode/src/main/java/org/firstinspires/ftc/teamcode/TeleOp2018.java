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
    ElapsedTime runtime2;

    float straight = 0; //Variable for straight motion
    double turn = 0; //Variable for turn
    double wristPosition = 0; //Variable for wristPosition
    double armPosition = 0.5;
    float encoderCurrent; //Variable for current encoder value

    public void init() { //Start of the initiation for autonomous

        m1 = hardwareMap.dcMotor.get("m1"); //Sets m1 to m1 in the config
        m2 = hardwareMap.dcMotor.get("m2"); //Sets m2 to m2 in the config
        m3 = hardwareMap.dcMotor.get("m3"); //Sets m3 to m3 in the config
        m4 = hardwareMap.dcMotor.get("m4"); //Sets m4 to m4 in the config
        m5 = hardwareMap.dcMotor.get("m5"); //Sets m5 to m5 in the config
        m6 = hardwareMap.dcMotor.get("m6"); //Sets m6 to m6 in the config
        m7 = hardwareMap.dcMotor.get("m7"); //Sets m7 to m7 in the config

        s1 = hardwareMap.servo.get("s1"); //Sets s1 in the config
        s2 = hardwareMap.servo.get("s2"); //Sets s2 in the config
        s3 = hardwareMap.servo.get("s3"); //Sets s3 in the config
        s4 = hardwareMap.servo.get("s4"); //Sets s4 in the config
//        s5 = hardwareMap.servo.get("s5"); //Sets s5 in the config
        s6 = hardwareMap.servo.get("s6"); //Sets s6 in the config

        m2.setDirection(DcMotor.Direction.REVERSE); //Sets m2 direction to REVERSE
        m4.setDirection(DcMotor.Direction.REVERSE); //Sets m4 direction to REVERSE

        touchSensor1 = hardwareMap.touchSensor.get("t1"); //Sets touchSensor1 to t1 in the config

        runtime = new ElapsedTime(); //Creates runtime variable for using time
        runtime2 = new ElapsedTime(); //Creates runtime variable for using time

        s1.setPosition(0); //Pulls jewel appendage against side of robot
        s2.setPosition(1); //Opens Relic Claw
        s3.setPosition(0); //Sets Arm Crunch Servo A
        s4.setPosition(1); //Sets Arm Crunch Servo B
//        s5.setPosition(0.5); //Opens 2nd gripper *NOT USED*
        s6.setPosition(0.5); //Sets arm extension to not move

    } //Ends initiation


    @Override
    public void loop() {  //Start of main loop

        encoderCurrent = m7.getCurrentPosition(); //Defines encoderCurrent as m7 encoder position

        telemetry.addData("Motor 7: ", encoderCurrent); //Telemetry for variables

        float LUD = gamepad1.left_stick_y; //Variable for left stick y axis on gamepad 1 for driver control
        double LRL = (-gamepad1.left_stick_x) * 1.5; //Variable for left stick x axis on gamepad 1 for driver control
        float RUD = gamepad1.right_stick_y; //Variable for right stick y axis on gamepad 1 for driver control
        float RLR = -gamepad1.right_stick_x; //Variable for right stick x axis on gamepad 1 for driver control

        //Controls for Relic Claw (Controller 2)
        if (gamepad2.left_trigger != 0) { //If left trigger is pressed, close claw
            s2.setPosition(0.7); //Sets servo position to 1
        } else if (gamepad2.left_trigger != 0 && runtime2.seconds() < 0.5) {
            s2.setPosition(0.6);
            runtime2.reset();
        } else { //If not pressed, open claw
            s2.setPosition(0.3); //Sets servo position to .5
            runtime2.reset();
        }

        //Controls the raising and lower the arm crunch mast. (Controller 2)
        if (gamepad2.left_stick_y > 0.1 && !touchSensor1.isPressed()) { //If the y axis is raised
            m7.setPower(gamepad2.left_stick_y / 4); //Raise the arm mast
        } else if (gamepad2.left_stick_y < -0.1) { //If the y axis is lowered
            m7.setPower(gamepad2.left_stick_y / 2); //Lower the arm mast
        } else {
            m7.setPower(0); //If stick isn't touched, do not move.
        }

        //Controls for rotating arm base (Controller 2)
        if (gamepad2.right_stick_x < 0.1 || gamepad2.right_stick_x > 0.1) { //If the x axis of right stick is pressed, move arm base
            m6.setPower(gamepad2.right_stick_x / 4); //Sets motor power to 1/4th of joystick speed
        } else { //If the x axis of right stick is not pressed, hold at current position
            m6.setPower(0); //Sets motor power to 0
        }

        //Controls for arm raise motor (controller 2)
        if (gamepad2.right_stick_y < 0.1 || gamepad2.right_stick_y > 0.1) { //If the y axis of left stick is pressed, raise arm
            m5.setPower(-gamepad2.right_stick_y); //Sets motor power to joystick speed
        } else { //If the y axis of left stick is not pressed, hold at current position
            m5.setPower(0); //Sets motor power to 0
        }

        //Controls for arm extension (controller 2)
        if (gamepad2.left_bumper && armPosition <= 1) {
            armPosition += 0.005; //Retraction for arm extension
            s6.setPosition(armPosition); //Sets servo position to arm extension servo
        } else if (gamepad2.right_bumper && armPosition >= 0.5) {
            armPosition -= 0.005; //Extension for arm extension
            s6.setPosition(armPosition); //Sets servo position to arm extension servo
        }

        //Controls for arm crunch (Controller 2)
        if (gamepad2.right_trigger != 0) { //If right trigger is pressed, close claw
            s3.setPosition(0.36); //Sets servo position to 0.36
            s4.setPosition(0.52); //Sets servo position to 0.58
            runtime.reset();
        } else if (gamepad2.right_trigger == 0 && runtime.seconds() < 0.5) {
            s3.setPosition(0.21); //Sets servo position to 0.36
            s4.setPosition(0.67); //Sets servo position to 0.58
        } else { //If not pressed, open arm crunch
            s3.setPosition(0); //Sets servo position to 1
            s4.setPosition(0.9); //Sets servo position to 1
        }

        //Controls for drive train (Controller 1)
        if (gamepad1.right_trigger == 0) { //Controls for slow mode (default)0
            m1.setPower(((LRL + LUD) / 3) + (RLR / 4) - turn); //Steering for top left
            m2.setPower(((LUD - LRL) / 3) - (RLR / 4) + turn); //Steering for top right
            m3.setPower(((LUD - LRL) / 3) + (RLR / 4) - turn); //Steering for back left
            m4.setPower(((LRL + LUD) / 3) - (RLR / 4) + turn); //Steering for back right
        } else { //Controls for normal mode
            m1.setPower(((LRL + LUD) / 1.5) + (RLR / 1.2) - turn); //Steering for top left
            m2.setPower(((LUD - LRL) / 1.5) - (RLR / 1.2) + turn); //Steering for top right
            m3.setPower(((LUD - LRL) / 1.5) + (RLR / 1.2) - turn); //Steering for back left
            m4.setPower(((LRL + LUD) / 1.5) - (RLR / 1.2) + turn); //Steering for back right
        }
    }
}