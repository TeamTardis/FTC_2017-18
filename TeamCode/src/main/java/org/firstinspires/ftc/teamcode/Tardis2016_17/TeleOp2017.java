//TeleOp-2017

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TeleOp-2017", group = "TeleOp")
public class TeleOp2017 extends org.firstinspires.ftc.teamcode.TardisOpModeAutonomous { //Init code in separate program

    public enum steps {
        NORMAL_DRIVE,
        SHOOT_BALL
    }

    public steps CURRENT_STEP = steps.NORMAL_DRIVE; //Sets the variable CURRENT_STEP to the first step in the sequence

    //Toggle Boolean states
    boolean sweeperCurr = false; //Toggle sweeper current state
    boolean sweeperPrev = false; //Toggle sweeper previous state
    boolean sweeperOn = false; //Toggle sweeper on state
    boolean shooterCurr = false; //Toggle shooter current state
    boolean shooterPrev = false; //Toggle shooter previous state
    boolean shooterOn = false; //Toggle shooter on state
    boolean linearCurr = false; //Toggle linear actuator current state
    boolean linearPrev = false; //Toggle linear actuator previous state
    boolean linearExt = false; //Toggle linear actuator extended state

    int beaconSelect;

    ElapsedTime runtime = new ElapsedTime(); //Runtime

    float LUD; //Left joystick y axis
    float LRL; //Left joystick x axis
    float R;   //Right joystick x axis
    float LUD2; //Left joystick y axis (Game pad 2)
    float RUD; //Right joystick y axis (Game pad 2)
    double threshold = 0.1; //Threshold
    int shooterSpeed = 2; //Shooter speed toggle variable

    @Override
    public void loop() {

//        telemetry.addData(">", "Color Red (Ground): " + colorSensor2.red() + "\nColor Blue (Ground): " + colorSensor2.blue() + "\nzValue: " + gyro.getIntegratedZValue() + "\nShooter Speed: " + shooterSpeed + "\nLinear: " + linearExt); //Adds telemetry to debug
//        telemetry.update(); //Updates telemetry with new information

        LUD = gamepad1.left_stick_y;   //(Game pad 1)
        LRL = -gamepad1.left_stick_x;  //(Game pad 1)
        R = gamepad1.right_stick_x;    //(Game pad 1)
        LUD2 = gamepad2.left_stick_y;  //(Game pad 2)
        RUD = -gamepad2.right_stick_y; //(Game pad 2)

        //Check status of right bumper (Game pad 1)
        shooterCurr = gamepad1.left_bumper;

        //Check for button state transitions.
        if ((shooterCurr == true) && (shooterCurr != shooterPrev)) {

            //Shooter transitioning to toggled state
            shooterOn = !shooterOn;
            if (shooterOn) {
                m5.setPower(1); //Shooter On
                m6.setPower(1);
            } else {
                m5.setPower(0.1); //Shooter Slow
                m6.setPower(0.1);
            }
        }

        shooterPrev = shooterCurr; //Sets back to toggleable state

/*        if (gamepad1.y)  {
            m5.setMaxSpeed(1800);  //Set max speed high for shooter
            m6.setMaxSpeed(1800);
            shooterSpeed = 3; //Shooter speed variable = 3
        }
        if (gamepad1.b)  {
            m5.setMaxSpeed(1700);  //Set max speed medium for shooter
            m6.setMaxSpeed(1700);
            shooterSpeed = 2; //Shooter speed variable = 2
        }
        if (gamepad1.a)  {
            m5.setMaxSpeed(1600);  //Set max speed low for shooter
            m6.setMaxSpeed(1600);
            shooterSpeed = 1; //Shooter speed variable = 1
        }
*/
        //Check status of right bumper
        linearCurr = gamepad2.x;

        //Check for button state transitions.
        if ((linearCurr) && (linearCurr != linearPrev)) {

            //Sweeper transitioning to toggled state
            linearExt = !linearExt;
            if (linearExt) {
                s1.setPosition(.1);
            } else {
                s1.setPosition(.6);
            }
        }
        linearPrev = linearCurr; //Sets back to toggleable state

        //If the left trigger is pressed hold servos up (Game pad2)
        if (gamepad2.left_trigger == 1) {
            SFront.setPosition(.3); //Up
        } else {
            SFront.setPosition(0); //Down
        }

        //Mast code with limit touch sensor switches (Game pad 2)
        if ((LUD2 > threshold) && (!touchSensor2.isPressed())) { //If the limit 2 is not pressed
            m8.setPower(1); //Go up
        } else if ((LUD2 < -threshold) && (!touchSensor1.isPressed())) { //If the limit 1 is not pressed
            m8.setPower(-1); //Go down
        } else {
            m8.setPower(0); //Do nothing
        }

        //Scooper Y is up, b is middle, a is down
        if (gamepad2.a) {
            s3.setPosition(0.06); //Down
        } else if (gamepad2.b) {
            s3.setPosition(0.15); //Mid
        } else if (gamepad2.y) {
            s3.setPosition(0.32); //Up
        }

        //Check status of right bumper
        sweeperCurr = gamepad2.right_trigger == 1;

        //Check for button state transitions.
        if ((sweeperCurr == true) && (sweeperCurr != sweeperPrev)) {

            //Sweeper transitioning to toggled state
            sweeperOn = !sweeperOn;
            if (sweeperOn) {
                m7.setPower(-1); //Sweep in
            } else {
                m7.setPower(0); //Do nothing
            }
        }
        sweeperPrev = sweeperCurr; //Sets back to toggleable state

        if (!sweeperOn && gamepad2.right_bumper) { //If the toggle is off and the trigger is pressed
            m7.setPower(1); //Sweep out
        } else if (!sweeperOn && !gamepad2.right_bumper) {
            m7.setPower(0); //Set sweeper power to 0
        }

        if (gamepad1.left_trigger == 1) {
            runtime.reset();
            CURRENT_STEP = steps.SHOOT_BALL;
        }

        if(CURRENT_STEP != steps.NORMAL_DRIVE && R > threshold || R < -threshold || LRL > threshold || LRL < -threshold || LUD > threshold || LUD < -threshold) {
            CURRENT_STEP = steps.NORMAL_DRIVE;
        }

        switch (CURRENT_STEP) {

            case NORMAL_DRIVE:

                //If no buttons are pressed, control normal
                //If R is not used,then use left Stick (R overrides L)
                if (gamepad1.right_trigger == 0) {
                    if (R == 0) {
                        //Controls for Left Stick
                        m1.setPower((LRL + LUD) / 2);   //Motor1 is x-axis plus y-axis
                        m2.setPower((LUD - LRL) / 2);   //Motor2 is x-axis minus y-axis
                        m3.setPower((LUD - LRL) / 2);   //Motor3 is x-axis minus y axis(parallel to M2)
                        m4.setPower((LRL + LUD) / 2);   //Motor4 is x-axis plus y-axis(parallel to M1)
                        break;
                    }
                    //If R is Outside the threshold, then rotate on the direction of Right Stick
                    if (R > threshold || R < -threshold) {
                        //Controls for Right Stick
                        m1.setPower(R / 2); //Set power to negative x-axis (orientation)
                        m2.setPower(-R / 2); //Set power to positive x-axis
                        m3.setPower(R / 2); //Set power to negative x-axis (orientation)
                        m4.setPower(-R / 2); //Set power to positive x-axis
                        break;
                    }

                }
                //If trigger is pressed, go very slow
                else if (gamepad1.right_trigger == 1) {
                    //If R is not used,then use left Stick (R overrides L)
                    if (R == 0) {
                        //Controls for Left Stick
                        m1.setPower((LRL + LUD) / 4);   //Motor1 is x-axis plus y-axis
                        m2.setPower((LUD - LRL) / 4);   //Motor2 is x-axis minus y-axis
                        m3.setPower((LUD - LRL) / 4);   //Motor3 is x-axis minus y axis(parallel to M2)
                        m4.setPower((LRL + LUD) / 4);   //Motor4 is x-axis plus y-axis(parallel to M1)
                        break;
                    }
                    //If R is Outside the threshold, then rotate on the direction of Right Stick
                    if (R > threshold || R < -threshold) {
                        //Controls for Right Stick
                        m1.setPower(R / 6);    //Set power to negative x-axis (orientation)
                        m2.setPower(-R / 6);     //Set power to positive x-axis
                        m3.setPower(R / 6);    //Set power to negative x-axis (orientation)
                        m4.setPower(-R / 6);     //Set power to positive x-axis
                        break;
                    }
                    //If nothing at all is happening, do nothing
                    break;
                }
                break;

            case SHOOT_BALL: //Case for shooting a ball

                if (gamepad1.left_trigger == 1) { //If gamepad 2 left trigger is pushed
                    s2.setPosition(1); //Lift shooter servo
                    break; //Breaks loop
                } else { //If its not pressed
                    s2.setPosition(.05); //Shooter servo returns to down position
                    CURRENT_STEP = steps.NORMAL_DRIVE; //Changes case to normal drive
                    break; //Breaks loop
                }

        } //End of switch statement
    } //Void loop end
} //OpMode loop end

