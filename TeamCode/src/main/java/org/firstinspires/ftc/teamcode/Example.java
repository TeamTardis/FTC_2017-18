package org.firstinspires.ftc.teamcode;
/**
 * Created by Corning Robotics on 9/25/16.
 */

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//Imports

@TeleOp(name="Example", group="TeleOp")
@Disabled
public class Example extends OpMode {

    DcMotor m1; //Define dcMotor as m1
    DcMotor m2;
    DcMotor m3;
    DcMotor m4;

    //Servo s1; //Color sensor arm servo
    //Servo s2;

    //ModernRoboticsI2cGyro gyro;

    public void init() { //Start of the initiation for autonomous

        m1 = hardwareMap.dcMotor.get("m1"); //Sets m1 to m3 in the config
        m2 = hardwareMap.dcMotor.get("m2"); //Sets m1 to m3 in the config
        m3 = hardwareMap.dcMotor.get("m3");
        m4 = hardwareMap.dcMotor.get("m4");

        m2.setDirection(DcMotor.Direction.REVERSE);
        m4.setDirection(DcMotor.Direction.REVERSE);

        //s1 = hardwareMap.servo.get("s1"); //Sets s1 i the config
        //s2 = hardwareMap.servo.get("s2"); //Sets s1 i the config

        //gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

        //r1 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "r1");

        //c1 = hardwareMap.colorSensor.get("c1");

        /*gyro.calibrate();
        while(gyro.isCalibrating()) {
            telemetry.addData("", "Gyro Calibrating. Please wait...");
            telemetry.update();
        }
        telemetry.addData("", "Gyro Calibrated. Good luck!");
        telemetry.update();
*/
    } //Ends initiation

    ElapsedTime runtime = new ElapsedTime();
    float ljsy;
    float ljsx;
    float rjsx;
    @Override
    public void loop() {
        ljsy = -gamepad1.left_stick_y;
        ljsx = -gamepad1.left_stick_x;
        rjsx = -gamepad1.right_stick_x;
        //telemetry.addData("Left stick y", ljsy);
        //telemetry.update();
        /*left joystick controls movement left//right and up/down
          right joystick controls rotation
        */

        if(rjsx > .1 || rjsx < -.1) {
            if (rjsx > .1 || rjsx < -.1) {
                m1.setPower(rjsx);
                m2.setPower(-rjsx);
                m3.setPower(rjsx);
                m4.setPower(-rjsx);
            }
        } else if((rjsx < .1 || rjsx > -.1) && (ljsy > .1 || ljsy < -.1) || (ljsx > .1 || ljsx < -.1)) {
            if (Math.abs(ljsx) < Math.abs(ljsy)) {
                m1.setPower(ljsy);
                m2.setPower(ljsy);
                m3.setPower(ljsy);
                m4.setPower(ljsy);
            } else {
                m1.setPower(ljsx);
                m2.setPower(-ljsx);
                m3.setPower(-ljsx);
                m4.setPower(ljsx);
            }
        } else {
            m1.setPower(0);
            m2.setPower(0);
            m3.setPower(0);
            m4.setPower(0);
        }
    }
}
