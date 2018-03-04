//AutoSteps
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutoSteps extends LinearOpMode { //Extends LinearOpMode for autonomous

    public enum steps { //All steps for completing autonomous

        SCANIMAGE, //Runs vuforia to scan pictograph
        LOWERSERVO, //Extends jewel appendage
        SENSECOLOR, //Determines whether one jewel is red or blue
        KNOCKFORWARDS, //Knocks in direction of relic recovery zone
        KNOCKBACK, //Knocks away from relic recovery zone
        MOVE_CLOSER,
        RECOVER,
        MOVE_FARTHER,
        RAISESERVO, //Retracts jewel appendage
        DRIVETOCRYPTOBOX, //Drives towards cryptobox
        ROTATE, //Used only in RedFar, rotates to face cryptobox
        BACKUP, //Used only in RedFar, backs up to be further from cryptobox
        LEFTCOLUMN, //Positions robot to line up at the left cryptobox column
        CENTERCLOMUN, //Positions robot to line up at the center cryptobox column
        RIGHTCOLUMN, //Positions robot to line up at the right cryptobox column
        POSITIONCHECK, //Positions the robot to be in a correct range while finding the cryptobox column
        FORWARD, //Drives into cryptobox column to place glyph
        DROP, //Opens gripper and drops glyph
        BACK, //Backs out of cryptobox column
        FORWARD2, //Pushes in glyph to double check its placement
        BACK2, //Backs out to make sure we aren't supporting the glyph, stops in safety zone
        PARK, //Used in RedNear and BlueNear to park in safety zone
        STOP, //Sets all motor speeds to zero
        COND_BACK,
        COND_ROTATE,
        COND_PARK2,
        COND_EXPLORE,
        COND_EXPLOREHARDER,
        COND_FORWARD,
        COND_GRAB,
        COND_BACK2,
        COND_ROTATE2,
        COND_RETURN,
        COND_DROP,
        COND_BACK3,
        COND_FORWARD2,
        COND_BACK4,
        COND_PARK3


    } //End of steps for autonomous

    @Override
    public void runOpMode() throws InterruptedException {

    }
}

