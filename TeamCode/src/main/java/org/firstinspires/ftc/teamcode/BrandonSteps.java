package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by dobbinsms on 10/12/2017.
 */

public class BrandonSteps extends LinearOpMode {

    public enum steps { //All steps for completing autonomous

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //Main steps - These steps run in sequence when the program is going perfect with no unexpected readings in sensor inputs.//
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        FORWARD, //Moves forward for 2 seconds
        LOWER, //Lowers gripper for 2 seconds
        CLAW, //Closes claw
        ROTATE, //Rotate 180 degrees right
        RETURN, //Move forward for 2 seconds
        STOP //Stops the robot

    } //End of steps for autonomous

    @Override
    public void runOpMode() throws InterruptedException {

    }
}

