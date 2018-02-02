package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;
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

@TeleOp(name = "New Test TeleOp 2018", group = "TeleOp")
public class ToddBot extends TardisRobot{

    /**
     * Define motors and sensors
     */

    DcMotor mFL; //Front left motor
    DcMotor mFR; //Front right motor
    DcMotor mBL; //Back left motor
    DcMotor mBR; //Back right motor
    DcMotor mArmRaise; //Arm raise motor
    DcMotor mArmRotate; //Arm base rotation motor
    DcMotor mArmCrunch; //Arm crunch vertical motor

    Servo sColorSensorArm; //Color sensor arm servo
    Servo sRelicClaw; //Relic Claw servo
    Servo sArmCrunchA; //Arm Crunch A
    Servo sArmCrunchB; //Arm Crunch B
    Servo sArmExtension; //Arm extension

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
        mFL = hardwareMap.dcMotor.get("m1"); //Sets mFL to m1 in the config
        mFR = hardwareMap.dcMotor.get("m2"); //Sets mFR to m2 in the config
        mBL = hardwareMap.dcMotor.get("m3"); //Sets mBL to m3 in the config
        mBR = hardwareMap.dcMotor.get("m4"); //Sets mBR to m4 in the config
        mArmRaise = hardwareMap.dcMotor.get("m5"); //Sets mArmRaise to m5 in the config
        mArmRotate = hardwareMap.dcMotor.get("m6"); //Sets mArmRotate to m6 in the config
        mArmCrunch = hardwareMap.dcMotor.get("m7"); //Sets mArmCrunch to m7 in the config

        sColorSensorArm = hardwareMap.servo.get("s1"); //Sets sColorSensorArm in the config
        sRelicClaw = hardwareMap.servo.get("s2"); //Sets sRelicClaw in the config
        sArmCrunchA = hardwareMap.servo.get("s3"); //Sets sArmCrunchA in the config
        sArmCrunchB = hardwareMap.servo.get("s4"); //Sets sArmCrunchB in the config
        //        s5 = hardwareMap.servo.get("s5"); //Sets s5 in the config
        sArmExtension = hardwareMap.servo.get("s6"); //Sets sArmExtension in the config

        mFR.setDirection(DcMotor.Direction.REVERSE); //Sets mFR direction to REVERSE
        mBR.setDirection(DcMotor.Direction.REVERSE); //Sets mBR direction to REVERSE

        touchSensor1 = hardwareMap.touchSensor.get("t1"); //Sets touchSensor1 to t1 in the config

        runtime = new ElapsedTime(); //Creates runtime variable for using time
        runtime2 = new ElapsedTime(); //Creates runtime variable for using time

        sColorSensorArm.setPosition(0); //Pulls jewel appendage against side of robot
        sRelicClaw.setPosition(1); //Opens Relic Claw
        sArmCrunchA.setPosition(0); //Sets Arm Crunch Servo A
        sArmCrunchB.setPosition(1); //Sets Arm Crunch Servo B
        //        s5.setPosition(0.5); //Opens 2nd gripper *NOT USED*
        sArmExtension.setPosition(0.5); //Sets arm extension to not move
    } //Ends

    @Override
    public void loop(){
        this.updateFromController();
    }

    public void updateFromController() {  //Start of main loop
        encoderCurrent = mArmCrunch.getCurrentPosition(); //Defines encoderCurrent as mArmCrunch encoder position
        telemetry.addData("Motor 7: ", encoderCurrent); //Telemetry for variables

        float LUD = gamepad1.left_stick_y; //Variable for left stick y axis on gamepad 1 for driver control
        double LRL = (-gamepad1.left_stick_x) * 1.5; //Variable for left stick x axis on gamepad 1 for driver control
        float RUD = gamepad1.right_stick_y; //Variable for right stick y axis on gamepad 1 for driver control
        float RLR = -gamepad1.right_stick_x; //Variable for right stick x axis on gamepad 1 for driver control

        //Controls for Relic Claw (Controller 2)
        if (gamepad2.left_trigger != 0) { //If left trigger is pressed, close claw
            sRelicClaw.setPosition(0.7); //Sets servo position to 1
            runtime2.reset();
        } else if (gamepad2.left_trigger == 0 && runtime2.seconds() < 0.5) {
            sRelicClaw.setPosition(0.6);
        } else { //If not pressed, open claw
            sRelicClaw.setPosition(0); //Sets servo position to .5
        }

        //Controls the raising and lower the arm crunch mast. (Controller 2)
        if (gamepad2.left_stick_y > 0.1 && !touchSensor1.isPressed()) { //If the y axis is raised
            mArmCrunch.setPower(gamepad2.left_stick_y / 4); //Raise the arm mast
        } else if (gamepad2.left_stick_y < -0.1) { //If the y axis is lowered
            mArmCrunch.setPower(gamepad2.left_stick_y / 2); //Lower the arm mast
        } else {
            mArmCrunch.setPower(0); //If stick isn't touched, do not move.
        }

        //Controls for rotating arm base (Controller 2)
        if (gamepad2.right_stick_x < 0.1 || gamepad2.right_stick_x > 0.1) { //If the x axis of right stick is pressed, move arm base
            mArmRotate.setPower(gamepad2.right_stick_x / 4); //Sets motor power to 1/4th of joystick speed
        } else { //If the x axis of right stick is not pressed, hold at current position
            mArmRotate.setPower(0); //Sets motor power to 0
        }

        //Controls for arm raise motor (controller 2)
        if (gamepad2.right_stick_y < 0.1 || gamepad2.right_stick_y > 0.1) { //If the y axis of left stick is pressed, raise arm
            mArmRaise.setPower(-gamepad2.right_stick_y); //Sets motor power to joystick speed
        } else { //If the y axis of left stick is not pressed, hold at current position
            mArmRaise.setPower(0); //Sets motor power to 0
        }

        //Controls for arm extension (controller 2)
        if (gamepad2.left_bumper && armPosition <= 1) {
            armPosition += 0.005; //Retraction for arm extension
            sArmExtension.setPosition(armPosition); //Sets servo position to arm extension servo
        } else if (gamepad2.right_bumper && armPosition >= 0.5) {
            armPosition -= 0.005; //Extension for arm extension
            sArmExtension.setPosition(armPosition); //Sets servo position to arm extension servo
        }

        //Controls for arm crunch (Controller 2)
        if (gamepad2.right_trigger != 0) { //If right trigger is pressed, close claw
            sArmCrunchA.setPosition(0.36); //Sets servo position to 0.36
            sArmCrunchB.setPosition(0.52); //Sets servo position to 0.58
            runtime.reset();
        } else if (gamepad2.right_trigger == 0 & runtime.seconds() < 0.5) {
            sArmCrunchA.setPosition(0.21); //Sets servo position to 0.36
            sArmCrunchB.setPosition(0.67); //Sets servo position to 0.58
        } else { //If not pressed, open arm crunch
            sArmCrunchA.setPosition(0); //Sets servo position to 1
            sArmCrunchB.setPosition(0.9); //Sets servo position to 1
        }

        //Controls for drive train (Controller 1)
        if (gamepad1.right_trigger == 0) { //Controls for slow mode (default)0
            mFL.setPower(((LRL + LUD) / 3) + (RLR / 4) - turn); //Steering for top left
            mFR.setPower(((LUD - LRL) / 3) - (RLR / 4) + turn); //Steering for top right
            mBL.setPower(((LUD - LRL) / 3) + (RLR / 4) - turn); //Steering for back left
            mBR.setPower(((LRL + LUD) / 3) - (RLR / 4) + turn); //Steering for back right
        } else { //Controls for normal mode
            mFL.setPower(((LRL + LUD) / 1.5) + (RLR / 1.2) - turn); //Steering for top left
            mFR.setPower(((LUD - LRL) / 1.5) - (RLR / 1.2) + turn); //Steering for top right
            mBL.setPower(((LUD - LRL) / 1.5) + (RLR / 1.2) - turn); //Steering for back left
            mBR.setPower(((LRL + LUD) / 1.5) - (RLR / 1.2) + turn); //Steering for back right
        }
    }
}
