/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.lang.reflect.Constructor;

/* This opMode does the following steps:
 * 0) Wait till the encoders show reset to zero.
 * 1) Drives to the vicinity of the beacon using encoder counts
 * 2) Use the Legacy light sensor to locate the white line
 * 3) Tracks the line until the wall is reached
 * 4) Pushes up against wall to get square using constant power an time.
 * 5) Deploys the Climbers using the servo
 * 6) Drives to the Mountain using encoder counts
 * 7) Climbs the Mountain using constant speed and time
 * 8) Stops and waits for end of Auto
 *
 * The code is executed as a state machine.  Each "State" performs a specific task which takes time to execute.
 * An "Event" can cause a change in the state.  One or more "Actions" are performed when moving on to next state
 */

public class AutoStateMachine extends OpMode
{
    // A list of system States.
    private enum State
    {
        STATE_INITIAL,
        STATE_PREPARE_TO_FIRE_BALL,
        STATE_FIRE_BALL,
        STATE_DRIVE_TO_FAR_BEACON,
        STATE_DRIVE_TO_CLOSE_BEACON,
        STATE_DRIVE_TO_CENTER_VORTEX,
        STATE_STOP,
    }

    Hardware robot = new Hardware();

    private int         mLeftEncoderTarget;
    private int         mRightEncoderTarget;

    private State currentState;

    // Loop cycle time stats variables
    public ElapsedTime  runtime = new ElapsedTime();   // Time into round.
    private ElapsedTime stateTime = new ElapsedTime();  // Time into current state

    public AutoStateMachine()
    {

    }

    //--------------------------------------------------------------------------
    // init
    //--------------------------------------------------------------------------
    @Override
    public void init()
    {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");

        resetDriveEncoders();       // Reset Encoders to Zero
    }

    //--------------------------------------------------------------------------
    // loop
    //--------------------------------------------------------------------------
    // @Override
    public void init_loop()
    {
        // Send the current telemetry values
        displayEncoderTelemetry();
    }

    //--------------------------------------------------------------------------
    // start
    //--------------------------------------------------------------------------
    @Override
    public void start()
    {
        // Set initial state and start game clock
        runToPosition(); // Run to Position set by encoder targets
        runtime.reset(); // Zero game clock
        newState(State.STATE_INITIAL);
    }

    //--------------------------------------------------------------------------
    // loop
    //--------------------------------------------------------------------------
    @Override
    public void loop()
    {
        // Send the current state info (state and time) back to first line of driver station telemetry.
        telemetry.addData("0", String.format("%4.1f ", stateTime.time()) + currentState.toString());

        // Execute the current state.  Each STATE's case code does the following:
        // 1: Look for an EVENT that will cause a STATE change
        // 2: If an EVENT is found, take any required ACTION, and then set the next STATE
        //   else
        // 3: If no EVENT is found, do processing for the current STATE and send TELEMETRY data for STATE.
        //
        switch (currentState)
        {
            case STATE_INITIAL:         // Stay in this state until encoders are both Zero.
                if (encodersAtZero())
                {
                    newState(State.STATE_FIRE_BALL); // Next state
                }
                else
                {
                    // Display Diagnostic data for this state.
                    displayEncoderTelemetry();
                }

                break;

            case STATE_DRIVE_TO_FAR_BEACON: // Follow path until last segment is completed
                if (true)
                {
                    newState(State.STATE_DRIVE_TO_CLOSE_BEACON);      // Next State:
                }
                /*else
                {
                    // Display Diagnostic data for this state.
                    telemetry.addData("1", String.format("%d of %d. L %5d:%5s - R %5d:%5d ",
                                        mCurrentSeg, mCurrentPath.length,
                                        mLeftEncoderTarget, getLeftPosition(),
                                        mRightEncoderTarget, getRightPosition()));
                }*/
                break;

            case STATE_DRIVE_TO_CENTER_VORTEX: // Follow path until last segment is completed
                /*if (pathComplete())
                {
                    setDrivePower(0.5, 0.5);                // Action: Start Driving forward at 50 Speed
                    newState(State.STATE_CLIMB_MOUNTAIN);   // Next State:
                }
                else
                {
                    // Display Diagnostic data for this state.
                    telemetry.addData("1", String.format("%d of %d. L %5d:%5d - R %5d:%5d ",
                                                         mCurrentSeg, mCurrentPath.length,
                                                         mLeftEncoderTarget, getLeftPosition(),
                                                         mRightEncoderTarget, getRightPosition()));
                }*/
                break;

            case STATE_STOP:
                break;
        }
    }

    //--------------------------------------------------------------------------
    // stop
    //--------------------------------------------------------------------------
    @Override
    public void stop()
    {
        // put arm up
        // stop driving
    }

    //--------------------------------------------------------------------------
    // User Defined Utility functions here....
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    //  Transition to a new state.
    //--------------------------------------------------------------------------
    private void newState(State newState)
    {
        // Reset the state time, and then change to next state.
        stateTime.reset();
        currentState = newState;
    }

    private void displayEncoderTelemetry()
    {
        telemetry.addData("frontLeftMotor Enc", robot.frontLeftMotor.getCurrentPosition());
        telemetry.addData("frontRightMotor Enc", robot.frontRightMotor.getCurrentPosition());
        telemetry.addData("backLeftMotor Enc", robot.backLeftMotor.getCurrentPosition());
        telemetry.addData("backRightMotor Enc", robot.backRightMotor.getCurrentPosition());
    }
}

    //--------------------------------------------------------------------------
    // runToPosition ()
    // Set drive motors to RUN_TO_POSITION mode
    //--------------------------------------------------------------------------
    public void runToPosition()
    {
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //--------------------------------------------------------------------------
    // useConstantSpeed ()
    // Set drive motors to RUN_USING_ENCODERS mode
    //--------------------------------------------------------------------------
    public void useConstantSpeed()
    {
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //--------------------------------------------------------------------------
    // resetDriveEncoders()
    // Reset stop and reset drive motor encoders
    //--------------------------------------------------------------------------
    public void resetDriveEncoders() { setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); }

    //--------------------------------------------------------------------------
    // setDriveMode ()
    // Set both drive motors to new mode if they need changing.
    //--------------------------------------------------------------------------
    public void setDriveMode(DcMotor.RunMode mode)
    {
        // Ensure the motors are in the correct mode.
        if (robot.frontLeftMotor.getMode() != mode)
            robot.frontLeftMotor.setMode(mode);

        if (robot.frontRightMotor.getMode() != mode)
            robot.frontRightMotor.setMode(mode);

        if (robot.backLeftMotor.getMode() != mode)
            robot.backLeftMotor.setMode(mode);

        if (robot.backRightMotor.getMode() != mode)
            robot.backRightMotor.setMode(mode);
    }

    //--------------------------------------------------------------------------
    // moveComplete()
    // Return true if motors have both reached the desired encoder target
    //--------------------------------------------------------------------------
    boolean moveComplete()
    {
        //  return (!mLeftMotor.isBusy() && !mRightMotor.isBusy());
        return false; //((Math.abs(getLeftPosition() - mLeftEncoderTarget) < 10) &&
                //(Math.abs(getRightPosition() - mRightEncoderTarget) < 10));
    }

    //--------------------------------------------------------------------------
    // encodersAtZero()
    // Return true if all drive encoders read zero
    //--------------------------------------------------------------------------
    boolean encodersAtZero()
    {
        return ((robot.frontLeftMotor.getCurrentPosition() == 0) &&
                (robot.frontRightMotor.getCurrentPosition() == 0) &&
                (robot.backLeftMotor.getCurrentPosition() == 0) &&
                (robot.backRightMotor.getCurrentPosition() == 0));

    }
}