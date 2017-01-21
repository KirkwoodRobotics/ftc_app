package org.firstinspires.ftc.teamcode;

/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto Drive Encoder", group="Linear Opmode")
public class AutoDriveEncoder extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot = new Hardware();

    final double ARM_DOWN_POWER = -0.30;
    final long WAIT             = 1000;

    private int curPos;

    public final static int TETRIX_TICKS_PER_REV = 1440;
    public final static int ANDYMARK_TICKS_PER_REV = 1120;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart(); // Wait for the game to start (driver presses PLAY)

        // start with arm pointing towards center vortex

        // to firing position
        robot.hAutoDriveEncoder("forward", 0.5f, 1450);

        // hold down arm before firing
        holdDownArm();
        robot.waitForTick(WAIT);

        // fire ball one
        fireArm();
        robot.waitForTick(WAIT);

        // hold down arm before loading
        holdDownArm();
        robot.waitForTick(2000);

        // load ball two
        robot.loader.setPower(1);
        robot.waitForTick(3500);
        robot.loader.setPower(0);
        robot.waitForTick(WAIT);

        // fire ball two
        fireArm();

        // to cap ball
        robot.hAutoDriveEncoder("forward", 0.8f, 4 * ANDYMARK_TICKS_PER_REV);

        // to close beacon
        /*robot.hAutoDriveEncoder("forward", 0.8f, 4 * ANDYMARK_TICKS_PER_REV);

        // to close beacon
        robot.hAutoDriveEncoder("backward", 0.8f, 2 * ANDYMARK_TICKS_PER_REV);
        robot.hAutoDriveEncoder("left", 0.8f, 4* ANDYMARK_TICKS_PER_REV);
        robot.hAutoDriveEncoder("forward", 0.8f, 2 * ANDYMARK_TICKS_PER_REV);

        // to corner vortex
        robot.hAutoDriveEncoder("backward", 0.8f, 1 * ANDYMARK_TICKS_PER_REV);
        robot.hAutoDriveEncoder("counterclockwise", 0.2f, 1 * ANDYMARK_TICKS_PER_REV);
        robot.hAutoDriveEncoder("left", 0.8f, 1 * ANDYMARK_TICKS_PER_REV);
        robot.hAutoDriveEncoder("forward", 0.5f, 3 * ANDYMARK_TICKS_PER_REV);*/
    }

    private void holdDownArm() throws InterruptedException {
        curPos = -1; // if -1 gets returned then treat it as an error

        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!robot.touchSensor.isPressed()) {
            robot.arm.setPower(ARM_DOWN_POWER);
        }

        if (robot.touchSensor.isPressed()) {
            // keep arm down before firing
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            curPos = robot.arm.getCurrentPosition();
            robot.arm.setTargetPosition(curPos + 500);
            robot.arm.setPower(ARM_DOWN_POWER);
        }

        robot.waitForTick(WAIT);
    }

    private void fireArm() throws InterruptedException {
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.arm.setTargetPosition(curPos - 500);
        robot.arm.setPower(ARM_DOWN_POWER);

        robot.waitForTick(WAIT);
    }
}