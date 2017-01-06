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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Holonomic Drive
 */

@TeleOp(name="HoloDrive", group="Linear Opmode")
public class HoloDrive extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    Hardware robot = new Hardware();

    private double armPower;

    int armPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);

        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        TouchSensor touchSensor = hardwareMap.touchSensor.get("sensor_touch");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.addData("Status", ": " + robot.motorPower.printPower().toString());
            telemetry.addData("arm cur pos", ": " + robot.arm.getCurrentPosition());

            robot.teleDrive(gamepad1);

            // arm
            if (gamepad2.a) {
                robot.arm.setPower(-0.55);

                /*robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                runArm(-1420);*/
            }

            if (touchSensor.isPressed()) {
                robot.arm.setPower(0);
            }



            if (gamepad2.dpad_right) {
                runArm(armPos);
                armPos -= 3;
            }

            if (gamepad2.dpad_left) {
                runArm(armPos);
                armPos += 3;
            }

            // loader
            robot.loader.setPower(gamepad2.left_stick_y);

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            telemetry.update();
        }
    }

    public void runArm(int pos) {
        // Set target position of motor
        robot.arm.setTargetPosition(pos);

        // Set motor to RUN_TO_POSITION mode
        //robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armPower = -1;

        armPower = Range.clip(armPower, -1, 1); // keep here just for kicks

        robot.arm.setPower(armPower);
    }
}