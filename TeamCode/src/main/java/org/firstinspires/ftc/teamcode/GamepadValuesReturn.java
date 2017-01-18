package org.firstinspires.ftc.teamcode;

/**
 * Created on 1/7/17.
 */

public class GamepadValuesReturn {
    private double gamepad1LeftY;
    private double gamepad1LeftX;
    private double gamepad1RightX;

    public String setAndPrint(double g1LeftY, double g1LeftX, double g1RightX) {
        gamepad1LeftY = g1LeftY;
        gamepad1LeftX = g1LeftX;
        gamepad1RightX = g1RightX;

        return "gamepad1LeftY: " + gamepad1LeftY + "\n" +
                "gamepad1LeftX: " + gamepad1LeftX + "\n" +
                "gamepad1RightX: " + gamepad1RightX + "\n";
    }
}
