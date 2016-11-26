package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
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
public class MotorPowerCalc {
    public double frontLeftPower;
    public double frontRightPower;
    public double backLeftPower;
    public double backRightPower;

    public void calcAndSetMotorPower(double gamepad1LeftX, double gamepad1LeftY, double gamepad1RightX,
                                     DcMotor frontLeftMotor, DcMotor frontRightMotor,
                                     DcMotor backLeftMotor, DcMotor backRightMotor)
    {
        // Calculate motor power
        frontLeftPower = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
        frontRightPower = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
        backLeftPower = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
        backRightPower = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

        // Clip motor power values so they stay within the range -1 to 1
        frontLeftPower = Range.clip(frontLeftPower, -1, 1);
        frontRightPower = Range.clip(frontRightPower, -1, 1);
        backLeftPower = Range.clip(backLeftPower, -1, 1);
        backRightPower = Range.clip(backRightPower, -1, 1);

        // Set power to all drive motors
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    public void stop(DcMotor frontLeftMotor, DcMotor frontRightMotor,
                     DcMotor backLeftMotor, DcMotor backRightMotor)

    {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public String printPower()
    {
        return "frontLeftPower: " + frontLeftPower + "\n" +
                "frontRightPower: " + frontRightPower + "\n" +
                "backLeftPower: " + backLeftPower + "\n" +
                "backRightPower: " + backRightPower + "\n";
    }
}