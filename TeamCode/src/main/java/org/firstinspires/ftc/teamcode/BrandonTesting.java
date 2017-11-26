package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.teamcode.AutoSteps.steps.KNOCKFORWARDS;
//Imports

@Autonomous(name="BrandonTesting", group ="Autonomous")

public class BrandonTesting extends BrandonSteps {

    public static final String TAG = "BrandonTesting";

    DcMotor m1; //Define dcMotor as m1
    DcMotor m2; //Define dcMotor as m2
    DcMotor m3; //Define dcMotor as m3
    DcMotor m4; //Define dcMotor as m4
    DcMotor m5; //Define dcMotor as m5

    Servo s1; //Define Servo as s1
    Servo s2; //Define Servo as s2

    @Override
    public void runOpMode() {

        m1 = hardwareMap.dcMotor.get("m1"); //Sets m1 in the config
        m2 = hardwareMap.dcMotor.get("m2"); //Sets m2 in the config
        m3 = hardwareMap.dcMotor.get("m3"); //Sets m3 in the config
        m4 = hardwareMap.dcMotor.get("m4"); //Sets m4 in the config
        m5 = hardwareMap.dcMotor.get("m5"); //Sets m4 in the config

        m1.setDirection(DcMotor.Direction.REVERSE);
        m3.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        steps CURRENT_STEP = steps.FORWARD; //Sets the variable CURRENT_STEP to the first step in the sequence

        ElapsedTime runtime = new ElapsedTime(); //Creates a variable for runtime so we can have timed events

        float encoderCurrent;

        waitForStart();

        while(opModeIsActive()) {

            encoderCurrent = m5.getCurrentPosition();

            switch(CURRENT_STEP) { //Beginning of the switch- this sets the current step to whatever CURRENT_STEP is set to

                ///////////////////////
                //START OF MAIN STEPS//
                ///////////////////////

                case FORWARD:

                    if (runtime.seconds() > 2) {
                        m1.setPower(0);
                        m2.setPower(0);
                        m3.setPower(0);
                        m4.setPower(0);
                        runtime.reset();
                        CURRENT_STEP = steps.ROTATE;
                        break;
                    }
                    m1.setPower(.5);
                    m2.setPower(.5);
                    m3.setPower(.5);
                    m4.setPower(.5);
                    break;

                case LOWER:

                    if (encoderCurrent > 1200)  {
                        m1.setPower(0);
                        m2.setPower(0);
                        m3.setPower(0);
                        m4.setPower(0);
                        runtime.reset();
                        CURRENT_STEP = steps.CLAW;
                        break;
                    }
                    m5.setPower(.2);
                    break;

                case ROTATE:

                    if (runtime.seconds() > 6) {
                        m1.setPower(0);
                        m2.setPower(0);
                        m3.setPower(0);
                        m4.setPower(0);
                        runtime.reset();
                        CURRENT_STEP = steps.STOP;
                        break;
                    }
                    m1.setPower(.1);
                    m2.setPower(-.1);
                    m3.setPower(.1);
                    m4.setPower(-.1);
                    break;

                case RETURN:

                    if (runtime.seconds() > 2 ) {
                        m1.setPower(0);
                        m2.setPower(0);
                        m3.setPower(0);
                        m4.setPower(0);
                        runtime.reset();
                        CURRENT_STEP = steps.STOP;
                        break;
                    }
                        m1.setPower(.5);
                        m2.setPower(.5);
                        m3.setPower(.5);
                        m4.setPower(.5);

                case STOP:

                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    m5.setPower(0);
                    break;
            }
        }
    }
}

