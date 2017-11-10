package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by dobbinsms on 10/12/2017.
 */

public class AutoSteps extends LinearOpMode {

    public enum steps { //All steps for completing autonomous

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //Main steps - These steps run in sequence when the program is going perfect with no unexpected readings in sensor inputs.//
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        STARTCAMERA,
        SCANIMAGE,
        LOWERSERVO,
        SENSECOLOR,
        KNOCKFORWARDS, //Knocks blue jewel
        KNOCKBACK,
        RAISESERVO,
        DRIVETOCRYPTOBOX,
        ROTATE,
        LEFTCOLUMN,
        CENTERCLOMUN,
        RIGHTCOLUMN,
        STOP //[Stop] The title says it all.

    } //End of steps for autonomous

    @Override
    public void runOpMode() throws InterruptedException {

    }
}

