package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    public DcMotor frontLeftMotor        = null;
    public DcMotor frontRightMotor       = null;
    public DcMotor backLeftMotor         = null;
    public DcMotor backRightMotor        = null;

    public DcMotor arm                   = null;
    public DcMotor loader                = null;
    public DcMotor capBallLifter         = null;

    public MotorPowerCalc motorPower     = new MotorPowerCalc();

    public TouchSensor touchSensor       = null;
    public ColorSensor beaconColorSensor = null;
    public ColorSensor lineColorSensor   = null;
    //public ModernRoboticsI2cRangeSensor rangeSensor    = null;

    HardwareMap hwMap                    = null;
    private ElapsedTime period           = new ElapsedTime();
    private Telemetry hwTelemetry        = null;

    final double ARM_DOWN_POWER          = -0.30;
    final long WAIT                      = 1000;

    public boolean redTeam;
    public boolean leftPos;
    public boolean pushed                = false;

    public final static int ANDYMARK_TICKS_PER_REV = 1120;

    public void init(HardwareMap ahwMap, Telemetry telem)
    {
        // save reference to HW Map
        hwMap = ahwMap;

        // save telemetry object
        hwTelemetry = telem;

        // Define and Initialize Motors
        frontLeftMotor  = hwMap.dcMotor.get("frontLeft");
        frontRightMotor = hwMap.dcMotor.get("frontRight");
        backLeftMotor   = hwMap.dcMotor.get("backLeft");
        backRightMotor  = hwMap.dcMotor.get("backRight");

        arm             = hwMap.dcMotor.get("arm");
        loader          = hwMap.dcMotor.get("loader");
        capBallLifter   = hwMap.dcMotor.get("capBallLifter");

        // Set all motors to zero power
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        arm.setPower(0);
        loader.setPower(0);

        loader.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        capBallLifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        touchSensor       = hwMap.touchSensor.get("sensor_touch");
        beaconColorSensor = hwMap.colorSensor.get("sensor_color_beacon");
        lineColorSensor   = hwMap.colorSensor.get("sensor_color_line");
        //rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
    }

    private void autoDriveEncoder(float gamepad1LeftX, float gamepad1LeftY, float gamepad1RightX, int pos, String dir)
    {
        // Reset encoder values to 0
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set target position of all motors
        if (dir.equals("forward") || dir.equals("backward")) {
            frontLeftMotor.setTargetPosition(pos);
            frontRightMotor.setTargetPosition(-pos);
            backLeftMotor.setTargetPosition(pos);
            backRightMotor.setTargetPosition(-pos);
        } else if (dir.equals("right") || dir.equals("left")) {
            frontLeftMotor.setTargetPosition(- pos);
            frontRightMotor.setTargetPosition(pos);
            backLeftMotor.setTargetPosition(pos);
            backRightMotor.setTargetPosition(-pos);
        } else if (dir.equals("clockwise") || dir.equals("counterclockwise")) {
            frontLeftMotor.setTargetPosition(pos);
            frontRightMotor.setTargetPosition(pos);
            backLeftMotor.setTargetPosition(pos);
            backRightMotor.setTargetPosition(pos);
        }

        // Set motors to RUN_TO_POSITION mode
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorPower.calcAndSetMotorPower(gamepad1LeftX, gamepad1LeftY, gamepad1RightX,
                frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);

        while (frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy()) {
            // wait until motors reach target position
            hwTelemetry.addData("frontLeftMotor:", frontLeftMotor.getCurrentPosition());
            hwTelemetry.addData("frontRightMotor:", frontRightMotor.getCurrentPosition());
            hwTelemetry.addData("backLeftMotor:", backLeftMotor.getCurrentPosition());
            hwTelemetry.addData("backRightMotor:", backRightMotor.getCurrentPosition());
            hwTelemetry.update();
        }

        motorPower.stop(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
    }

    /*
     * hAutoDriveEncoder = human readable auto drive (with) encoders
     *
     * Programs should use this method to drive autonomously with encoders, don't call autoDriveEncoder() directly
     */
    public void hAutoDriveEncoder(String dir, float power, int pos) throws InterruptedException
    {
        switch (dir) {
            case "forward":
                autoDriveEncoder(power, 0, 0, -pos, "forward");
                break;
            case "backward":
                autoDriveEncoder(power, 0, 0, pos, "backward");
                break;

            case "right":
                autoDriveEncoder(0, power, 0, -pos, "right");
                break;
            case "left":
                autoDriveEncoder(0, power, 0, pos, "left");
                break;

            case "clockwise":
                autoDriveEncoder(0, 0, power, -pos, "clockwise");
                break;
            case "counterclockwise":
                autoDriveEncoder(0, 0, power, pos, "counterclockwise");
                break;

            default:
                autoDriveEncoder(0, 0, 0, 0, "forward");
                break;
        }
    }

    public void teleDrive(Gamepad gamepad1)
    {
        double gamepad1LeftY = gamepad1.left_stick_x;
        double gamepad1LeftX = gamepad1.left_stick_y;
        double gamepad1RightX = gamepad1.right_stick_x;

        gamepad1LeftY = Math.pow((Math.tanh(gamepad1LeftY) / Math.tanh(1)), 3);
        gamepad1LeftX = Math.pow((Math.tanh(gamepad1LeftX) / Math.tanh(1)), 3);
        gamepad1RightX = Math.pow((Math.tanh(gamepad1RightX) / Math.tanh(1)), 3);

        motorPower.calcAndSetMotorPower(gamepad1LeftX, gamepad1LeftY, gamepad1RightX,
                frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
    }

    public void holdDownArm(int curPos) throws InterruptedException {
        curPos = -1; // if -1 gets returned then treat it as an error

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!touchSensor.isPressed()) {
            arm.setPower(ARM_DOWN_POWER);
        }

        // keep arm down before firing
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        curPos = arm.getCurrentPosition();
        arm.setTargetPosition(curPos + 650);
        arm.setPower(ARM_DOWN_POWER);

        waitForTick(WAIT);
    }

    public void fireArm(int curPos) throws InterruptedException {
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setTargetPosition(curPos - 650); // 500
        arm.setPower(ARM_DOWN_POWER);

        waitForTick(WAIT);
    }

    public void checkColor() throws InterruptedException {
        if (beaconColorSensor.blue() > beaconColorSensor.red()) {
            if (!redTeam) {
                // it is probably blue, since we are on the blue team push the button

                hAutoDriveEncoder("forward", 0.3f, 200); // distance untested

                pushed = true;
            }
        } else {
            if (redTeam) {
                // it is probably red, since we are on the red team push the button

                hAutoDriveEncoder("forward", 0.3f, 200); // distance untested

                pushed = true;
            }
        }

        hwTelemetry.addData("Red  ", beaconColorSensor.red());
        hwTelemetry.addData("Blue ", beaconColorSensor.blue());
        hwTelemetry.addData("pushed", pushed);
        hwTelemetry.update();
    }

    public void pushBeacon() throws InterruptedException {
        // move to left beacon button
       // hAutoDriveEncoder("left", 0.4f, 400); // distance untested

        // detect left color and push button if correct color
        checkColor();

        if (!pushed) {
            // move to right beacon button
            // hAutoDriveEncoder("right", 0.4f, 400); // distance untested

            // detect right color and push button if correct color
            checkColor();
        }
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
