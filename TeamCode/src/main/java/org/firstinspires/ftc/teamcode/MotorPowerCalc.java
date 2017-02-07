package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * This is NOT an opmode.
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
        frontLeftPower  = gamepad1LeftY - gamepad1LeftX + gamepad1RightX;
        frontRightPower = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
        backLeftPower   = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
        backRightPower  = gamepad1LeftY + gamepad1LeftX + gamepad1RightX;

        // Clip motor power values so they stay within the range -1 to 1
        frontLeftPower  = Range.clip(frontLeftPower, -1, 1);
        frontRightPower = Range.clip(frontRightPower, -1, 1);
        backLeftPower   = Range.clip(backLeftPower, -1, 1);
        backRightPower  = Range.clip(backRightPower, -1, 1);

        // Set power to all drive motors
        frontLeftMotor.setPower(-frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(-backRightPower);
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