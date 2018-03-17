//Tardis Autonomous Functions
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class TardisAutonomous extends LinearOpMode { //Extends LinearOpMode for autonomous

    public static final String TAG = "TardisAutonomous";

    /**
     * Front Left Motor, gearbox 40
     */
    DcMotor m1;

    /**
     * Front Right Motor, gearbox 40
     */
    DcMotor m2;

    /**
     * Back Right Motor, gearbox 40
     */
    DcMotor m3;

    /**
     * Back Left Motor, gearbox 40
     */
    DcMotor m4;

    /**
     * Arm Raise Motor, gearbox 60
     */
    DcMotor m5;

    /**
     * Arm Rotating Base Motor, gearbox 60
     */
    DcMotor m6;

    /**
     * Arm Crunch Vertical, gearbox 40
     */
    DcMotor m7;

    /**
     * Jewel Arm Servo, 190 degrees
     */
    Servo s1; //Color sensor arm servo

    /**
     * 1st Claw Grip Servo, 190 degrees
     */
    Servo s2; //Claw grip servo

    /**
     * Left Arm Crunch Servo, 190 degrees
     */
    Servo s3; //Left arm crunch servo

    /**
     * Right Arm Crunch Servo, 190 degrees
     */
    Servo s4; //Right arm crunch servo

    /**
     * Arm Extension Servo, continuous
     */
    Servo s6; //Arm extension

    /**
     * Backup Relic Servo, 190 degrees
     */
    Servo s7; //Backup relic gripper

    /**
     * Time Variable, use *.seconds()
     */
    ElapsedTime runtime; //Used for changing steps
    ElapsedTime checkTime; //Used for timed events within steps
    ElapsedTime matchTime; //Used for backup steps

    IntegratingGyroscope gyro; //Gyro
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    ColorSensor c1; //Color sensor

    I2cDeviceSynch r1reader; //Right range sensor
    ModernRoboticsI2cRangeSensor r1;

    I2cDeviceSynch r2reader; //Front range sensor
    ModernRoboticsI2cRangeSensor r2;

    I2cDeviceSynch r3reader; //Left range sensor
    ModernRoboticsI2cRangeSensor r3;

    I2cDeviceSynch r4reader; //Back range sensor
    ModernRoboticsI2cRangeSensor r4;

    OpticalDistanceSensor ods1; //Left "palm" arm crunch optical distance sensor
    OpticalDistanceSensor ods2; //Right "palm" arm crunch optical distance sensor
    OpticalDistanceSensor ods3; //Right "finger" arm crunch optical distance sensor

    VuforiaLocalizer vuforia; //Image recognition

    /**
     * Positive = Forward
     **/
    public void setDrivePower(double power, double turn) { //Drives robot forwards or backwards, availability for turn variable

        m1.setPower(power - turn);
        m2.setPower(power + turn);
        m3.setPower(power - turn);
        m4.setPower(power + turn);
    }

    /**
     * Positive = Right
     **/
    public void setStrafePower(double power, double turn) { //Drives robot to strafe left or right, availability for turn variable

        m1.setPower(power - turn);
        m2.setPower(-power + turn);
        m3.setPower(-power - turn);
        m4.setPower(power + turn);
    }

    /**
     * Positive = Forward
     **/
    public void setDriveEncoder(double encoder) { //Drives robot forwards or backwards, availability for turn variable

        double power = -((2 / (1 + Math.pow(Math.E, -.01 * ((encoder - m2.getCurrentPosition()) / 10)))) - 1); //A sigmoid function to allow the robot to slow as it reaches its target destination
        m1.setPower(power);
        m2.setPower(power);
        m3.setPower(power);
        m4.setPower(power);
    }

    /**
     * Positive = Clockwise
     **/
    public void setRotationPower(double power) { //Rotates robot using power variable

        m1.setPower(power);
        m2.setPower(-power);
        m3.setPower(power);
        m4.setPower(-power);
    }

    /**
     * Positive = Clockwise
     **/
    public void setRotationTarget(double target) { //Uses two "split" sigmoid functions to slow the robot as it reaches its target orientation

        double power;
        double gyro = modernRoboticsI2cGyro.getIntegratedZValue();
        double shift = 0.2; //Variable just used for experimentation and implementation
        if (gyro < 0) {
            power = (2 / (1 + Math.pow(Math.E, -(0.006 * (target - gyro) - shift)))) - 1;
        } else {
            power = (2 / (1 + Math.pow(Math.E, -(0.006 * (target - gyro) + shift)))) - 1;
        }
        m1.setPower(power); //Drives robot to rotate
        m2.setPower(-power);
        m3.setPower(power);
        m4.setPower(-power);
    }

    /**
     * Must be positive! Uses r3 (Left side of robot)
     **/
    public void setRangeTargetR3(double target, double lowPassR3, double gyroTarget) { //Uses sigmoid function to make robot slow as it reaches its target destination away from wall

        double power = 0;
        double range = lowPassR3;
        int gyro = modernRoboticsI2cGyro.getIntegratedZValue();
        double turn = 0;
        double shift = 0.2; //Variable just used for experimentation and implementation
        if (range < 1) { //Confirms there is no bad readings with the range sensor
            range = 1;
        } else if (range > 255) {
            range = 255;
        }

        if(gyro > gyroTarget) {
            turn = 0.18;
        }
        if(gyro < gyroTarget) {
            turn= -0.18;
        }
        power = (2 / (1 + Math.pow(Math.E, -(0.05 * (target - range))))) - 1; //Sigmoid function that allows the robot to slow as it reaches the target variable
        setStrafePower(power, turn);
    }

    /**
     * Positive = Clockwise
     **/
    public double setRotationPrecise(double target, double currPower) {//Rotates the robot so it can get oriented to the degree

        double gyro = modernRoboticsI2cGyro.getIntegratedZValue();
        double newPower = 0;
        if (gyro > target) {
            newPower = currPower -= 0.02;
        }
        if (gyro < target) {
            newPower = currPower += 0.02;
        }
        return newPower;
    }

    /**
     * Must be positive! r3 being used.
     **/
    public double setR3Precise(double target, double currPower) { //Increases power for precise movements

        double r3Value = r3.getDistance(DistanceUnit.CM);
        double newPower = 0;
        if (r3Value > target) {
            newPower = currPower -= 0.05;
        }
        if (r3Value < target) {
            newPower = currPower += 0.05;
        }
        return newPower;
    }

    /**
     * Steps for autonomous.
     **/
    public enum steps { //These steps are used throughout the red and blue autonomous programs

        SCANIMAGE,
        LOWERSERVO,
        SENSECOLOR,
        KNOCKFORWARDS,
        KNOCKBACK,
        RAISESERVO,
        OFF_STONE,
        CHECK_ROTATION,
        BACK_STONE,
        DRIVE_TO_CRYPTO,
        ROTATE,
        PRECISE_ROTATE,
        CHECK_DISTANCE,
        DROP_GLYPH,
        FIND_GLYPHS,
        SCAN_GLYPHS,
        FORWARD_GLYPH,
        GRAB_GLYPH,
        BACK_GLYPH,
        FACE_CRYPTO,
        DROP_GLYPH_2,
        FACE_PILE,
        BACKUP_ROTATE,
        EMRG_BACKUP,
        PRE_PLACE,
        MOVE_LEFT,
        FORWARD,
        ALIGN_CRYPTO,
        PRECISE_DISTANCE,
        LEFTCOLUMN,
        CENTERCOLUMN,
        RIGHTCOLUMN,
        POSITIONCHECK,
        MOVE_RIGHT,
        BACKUP,
        DRIVETOCRYPTOBOX,
        PRECISE_ROTATE_2,
        RESET,
        STOP
    }

    steps CURRENT_STEP = steps.SCANIMAGE; //Sets the variable CURRENT_STEP to the first step in the sequence

    /**
     * Changes step
     **/
    public void changeStep() { //Stops the robot before changing steps

        setDrivePower(0, 0);
        m5.setPower(0);
        m6.setPower(0);
        m7.setPower(0);
        runtime.reset();
    }

    /**
     * Closes the gripper
     **/
    public void gripClose() { //Closes the gripper

        s3.setPosition(0.76);
        s4.setPosition(0.45);
    }

    /**
     * Opens the gripper
     **/
    public void gripOpen() { //Opens the gripper

        s3.setPosition(0.47);
        s4.setPosition(0.75);
    }

    /**
     * Semi-opens gripper for autonomous scanning
     **/
    public void gripScan() { //Sets the gripper into scanning mode

        s3.setPosition(0.47);
        s4.setPosition(0.45);
    }

    /**
     * Optical distance sensor check for minimum readings
     **/
    public double odsCheck(double odsRawLight) {
        if (odsRawLight < 0.02) {
            odsRawLight = 0;
        }
        return odsRawLight;
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}