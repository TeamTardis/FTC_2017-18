//AutoSteps
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

abstract class RangeTestBedSteps extends OpMode { //Extends LinearOpMode for autonomous

    public enum steps { //All steps for completing autonomous

        RUNTIME_RESET,
        DRIVE_OFF_BALANCE,
        BACKWARDS,
        ENCODER_FORWARDS,
        STRAFE,
        FORWARD_UNTIL_COLUMN,
        ROTATION,
        ROTATION2,
        FORWARD,
        DROP,
        BACK,
        FORWARD2,
        BACK2,
        STRAFE_LEFT,
        STRAFE_RIGHT,
        CENTER,
        STOP

    } //End of steps for autonomous

    public void runOpMode() throws InterruptedException {

    }
}

