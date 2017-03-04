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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name="ADE - Full", group="Iterative Opmode")
public class AutoDriveEncoderFull extends OpMode {

    /* Declare OpMode members. */
    private Hardware robot = new Hardware();

    private final long WAIT = 1000;

    private int curPos;
    private int numPressed = 0;

    private boolean pushed = false;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);

        robot.beaconColorSensor.enableLed(true);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
    * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    *
    * TODO: switched this all around so I could get init_loop() now IDK what I needed it for
    */
    @Override
    public void init_loop() {
        try {
            if (robot.touchSensor.isPressed()) {
                switch (numPressed) {
                    case 1:
                        robot.redTeam = false;
                        robot.leftPos = false;
                        break;
                    case 2:
                        robot.leftPos = true;
                        break;
                    case 3:
                        robot.redTeam = true;
                        robot.leftPos = false;
                        break;
                    case 4:
                        robot.leftPos = true;
                        numPressed = 0;
                        break;
                }
                numPressed++;

                robot.waitForTick(500);
            }

            if (robot.leftPos) {
                telemetry.addData("Position", "Left");
            } else {
                telemetry.addData("Position", "Right");
            }

            if (robot.redTeam) {
                telemetry.addData("Team", "Red");
            } else {
                telemetry.addData("Team", "Blue");
            }

            telemetry.addData("Status", "init_loop()", numPressed);
            telemetry.update();
        } catch (InterruptedException err) {
            telemetry.addData("Error", "InterruptedException");
            telemetry.update();
        }
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     *
     * Using this like a linear opmode
     *
     * start with arm pointing towards center vortex
     */
    @Override
    public void start()  {
        try {
            //robot.pushBeacon();

            // to firing position
            robot.hAutoDriveEncoder("forward", 0.5f, 1450);

            // hold down arm before firing
            robot.holdDownArm(curPos);
            robot.waitForTick(WAIT);

            // fire ball one
            robot.fireArm(curPos);
            robot.waitForTick(WAIT);

            // hold down arm before loading
            robot.holdDownArm(curPos);
            robot.waitForTick(2000);

            // load ball two
            robot.loader.setPower(1);
            robot.waitForTick(3500);
            robot.loader.setPower(0);
            robot.waitForTick(WAIT);

            // fire ball two
            robot.fireArm(curPos);

            // to cap ball
            robot.hAutoDriveEncoder("forward", 0.8f, 4 * robot.ANDYMARK_TICKS_PER_REV);

            // to far beacon
            robot.hAutoDriveEncoder("forward", 0.8f, 4 * robot.ANDYMARK_TICKS_PER_REV);

            followLine();
            //robot.pushBeacon();

            // to close beacon
            robot.hAutoDriveEncoder("backward", 0.8f, 2 * robot.ANDYMARK_TICKS_PER_REV);
            robot.hAutoDriveEncoder("left", 0.8f, 4 * robot.ANDYMARK_TICKS_PER_REV);
            robot.hAutoDriveEncoder("forward", 0.8f, 2 * robot.ANDYMARK_TICKS_PER_REV);

            followLine();
            //robot.pushBeacon();

            // to corner vortex
            robot.hAutoDriveEncoder("backward", 0.8f, 1 * robot.ANDYMARK_TICKS_PER_REV);
            robot.hAutoDriveEncoder("counterclockwise", 0.2f, 1 * robot.ANDYMARK_TICKS_PER_REV);
            robot.hAutoDriveEncoder("left", 0.8f, 1 * robot.ANDYMARK_TICKS_PER_REV);
            robot.hAutoDriveEncoder("forward", 0.5f, 3 * robot.ANDYMARK_TICKS_PER_REV);
        } catch (InterruptedException err) {
            telemetry.addData("Error", "InterruptedException");
            telemetry.update();
        }
    }

    public void followLine() {
        // TODO: detect and follow line
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}