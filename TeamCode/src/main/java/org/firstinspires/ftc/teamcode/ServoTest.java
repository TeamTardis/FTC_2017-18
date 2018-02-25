package org.firstinspires.ftc.teamcode;
/**
 * Created by Corning Robotics on 9/25/16.
 */
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@TeleOp(name="ServoTest", group="TeleOp")

public class ServoTest extends OpMode {

    Servo servo;

    public void init() { //Start of the initiation for autonomous

        servo = hardwareMap.servo.get("servo"); //Sets s1 i the config

    } //Ends initiation

    @Override
    public void loop() {

        float LUD = gamepad1.left_stick_y; //Variable for left stick y axis

        double stest = 0;

        if (stest >= 0 && stest <= 1){
            if (LUD > 0) {
                stest += (LUD + 1) / 2;
            } else if (LUD < 0){
                stest -= (LUD - 1) / 2;
            }
        }

        telemetry.addData("","Servo" + servo.getPosition()); //Adds telemetry to debug
        telemetry.update(); //Updates telemetry with new information

        servo.setPosition(stest);

    }
}