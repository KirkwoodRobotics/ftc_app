package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left motor"
 * Motor channel:  Right drive motor:        "right motor"
 * Servo channel:  Servo to raise/lower arm: "arm"
 * Servo channel:  Servo to open/close claw: "claw"
 *
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class Hardware
{
    // Public OpMode members
    public DcMotor frontLeftMotor  = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor   = null;
    public DcMotor backRightMotor  = null;

    public DcMotor arm             = null;

    MotorPowerCalc motorPower = new MotorPowerCalc();

    // Local OpMode members
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftMotor  = hwMap.dcMotor.get("frontLeft");
        frontRightMotor = hwMap.dcMotor.get("frontRight");
        backLeftMotor   = hwMap.dcMotor.get("backLeft");
        backRightMotor  = hwMap.dcMotor.get("backRight");

        arm             = hwMap.dcMotor.get("arm");

        // Set all motors to zero power
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        arm.setPower(0);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize servos.
        /*arm = hwMap.servo.get("arm");
        claw = hwMap.servo.get("claw");
        arm.setPosition(ARM_HOME);
        claw.setPosition(CLAW_HOME);*/
    }

    private void autoDriveEncoder(float gamepad1LeftX, float gamepad1LeftY, float gamepad1RightX, int pos)
    {
        // Reset encoder values to 0
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set target position of all motors
        frontLeftMotor.setTargetPosition(pos);
        frontRightMotor.setTargetPosition(pos);
        backLeftMotor.setTargetPosition(pos);
        backRightMotor.setTargetPosition(pos);

        // Set motors to RUN_TO_POSITION mode
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorPower.calcAndSetMotorPower(gamepad1LeftX, gamepad1LeftY, gamepad1RightX,
                frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);

        while(frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy()) {
            // wait until motors reach target position
        }

        motorPower.stop(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
    }

    /*
     * hAutoDrive = human (readable) auto drive (without encoders)
     *
     * Programs should use this method to drive autonomously, don't call autoDrive() directly
     *
     * TODO: add power input / magnitude input to allow for values other than -1 and 1
     */
    public void hAutoDrive(String dir, int periodMs) throws InterruptedException
    {
        switch (dir) {
            case "forward":
                motorPower.calcAndSetMotorPower(0, 1, 0,
                        frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
                break;
            case "backward":
                motorPower.calcAndSetMotorPower(0, -1, 0,
                        frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
                break;
            case "right":
                motorPower.calcAndSetMotorPower(1, 0, 0,
                        frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
                break;
            case "left":
                motorPower.calcAndSetMotorPower(-1, 0, 0,
                        frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
                break;
            case "rotateClockwise": // TODO: reverse to fit with what Josh wanted so we are consistent
                motorPower.calcAndSetMotorPower(0, 0, -1,
                    frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
                break;
            default:
                motorPower.calcAndSetMotorPower(0, 0, 0,
                        frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
                break;
        }

        waitForTick(periodMs);
        motorPower.calcAndSetMotorPower(0, 0, 0,
                frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
    }

    /*
     * hAutoDriveEncoder = human (readable) auto drive (with) encoders
     *
     * Programs should use this method to drive autonomously with encoders, don't call autoDriveEncoder() directly
     */
    public void hAutoDriveEncoder(String dir, int pos) throws InterruptedException
    {
        switch (dir) {
            case "forward":
                autoDriveEncoder(0, 1, 0, pos);
                break;
            case "backward":
                autoDriveEncoder(0, -1, 0, pos);
                break;
            case "right":
                autoDriveEncoder(1, 0, 0, pos);
                break;
            case "left":
                autoDriveEncoder(-1, 0, 0, pos);
                break;
            case "rotateClockwise": // TODO: reverse to fit with what Josh wanted so we are consistent
                autoDriveEncoder(0, 0, -1, pos);
                break;
            default:
                autoDriveEncoder(0, 0, 0, 0);
                break;
        }
    }

    // TODO: check out Josh's request for exponential sensitivity on joysticks
    public void teleDrive(Gamepad gamepad1)
    {
        double gamepad1LeftY = -gamepad1.left_stick_y;
        double gamepad1LeftX = gamepad1.left_stick_x;
        double gamepad1RightX = -gamepad1.right_stick_x; // reversed

        /*gamepad1LeftY = Math.pow((Math.tanh(gamepad1LeftY) / Math.tanh(1)), 3);
        gamepad1LeftX = Math.pow((Math.tanh(gamepad1LeftX) / Math.tanh(1)), 3);
        gamepad1RightX = Math.pow((Math.tanh(gamepad1RightX) / Math.tanh(1)), 3);*/

        motorPower.calcAndSetMotorPower(gamepad1LeftX, gamepad1LeftY, gamepad1RightX,
                frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}