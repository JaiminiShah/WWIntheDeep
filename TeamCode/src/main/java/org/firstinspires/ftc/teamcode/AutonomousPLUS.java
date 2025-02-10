package org.firstinspires.ftc.teamcode;
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot1;
//import org.firstinspires.ftc.teamcode.OLD.Autonomous.AprilTags.MayFlowers;

/**
 * This is the autonomous mode. It moves the robot without us having to touch the controller.
 * Previous programmers really sucked at explaining what any of this meant, so we're trying to do better.
 * This is our third year now of using this file. It's kind of poetic and also adorable.
 */

public class AutonomousPLUS extends LinearOpMode {

    // This section tells the program all of the different pieces of hardware that are on our robot that we will use in the program.
    private ElapsedTime runtime = new ElapsedTime();


    public double speed = 0.6;
    public int sleepTime;
    public boolean inMarker;
    public double power;
    public double slidePos;

    //DO NOT DELETE THIS LINE! CAPITALIZATION IS VERY IMPORTANT!!!
    public Robot1 robot = null;
    public PIDController controller;
    public static double p=0,i=0,d=0;
    public static double f=0;
    public static int targetposition=0;
    private final double ticks_in_degree=537.6/360;


    @Override
    public void runOpMode() {

        robot = new Robot1(hardwareMap, telemetry, this);
    }

    //These are the basic functions for mechnum movement during auto... Don't mess with these unless something is inverted
    // Remember Without ODO pods there will be some inconsistency due to mechnum slippage

    public void moveRobotForward(int ticks, long pause) {
        if (opModeIsActive()) {
            robot.setTargets("Forward", ticks); // Inverted... Lol
            robot.positionRunningMode();
        }
        robot.powerSet(speed);

        while (opModeIsActive() && robot.isWheelsBusy()) {
            robot.tellMotorOutput();
        }

        robot.stopAllMotors();
        robot.encoderRunningMode();
        sleep(pause);
        robot.encoderReset();
    }

    public void moveRobotBackward(int ticks, long pause) {
        if (opModeIsActive()) {
            robot.setTargets("Backward", ticks);
            robot.positionRunningMode();
            robot.powerSet(speed);

            while (opModeIsActive() && robot.isWheelsBusy()) {
                robot.tellMotorOutput();
            }

            robot.stopAllMotors();
            robot.encoderRunningMode();
            sleep(pause);
            robot.encoderReset();
        }

    }

    public void moveRobotLeft(int ticks, long pause) {

        if (opModeIsActive()) {
            robot.setTargets("Left", ticks);
            robot.positionRunningMode();
            robot.powerSet(speed);

            while (opModeIsActive() && robot.isWheelsBusy()) {
                robot.tellMotorOutput();
            }

            robot.stopAllMotors();
            robot.encoderRunningMode();
            sleep(pause);
            robot.encoderReset();
        }
    }

    public void moveRobotRight(int ticks, long pause) {

        if (opModeIsActive()) {
            robot.setTargets("Right", ticks);
            robot.positionRunningMode();
            robot.powerSet(speed);

            while (opModeIsActive() && robot.isWheelsBusy()) {
                robot.tellMotorOutput();
            }

            robot.stopAllMotors();
            robot.encoderRunningMode();
            sleep(pause);
            robot.encoderReset();
        }
    }

    public void turnRobotRight(int ticks, long pause) {

        if (opModeIsActive()) {
            robot.setTargets("Turn Right", ticks);
            robot.positionRunningMode();
            robot.powerSet(speed);

            while (opModeIsActive() && robot.isWheelsBusy()) {
                robot.tellMotorOutput();
            }

            robot.stopAllMotors();
            robot.encoderRunningMode();
            sleep(pause);
            robot.encoderReset();
        }
    }

    public void turnRobotLeft(int ticks, long pause) {

        if (opModeIsActive()) {
            robot.setTargets("Turn Left", ticks);
            robot.positionRunningMode();
            robot.powerSet(speed);

            while (opModeIsActive() && robot.isWheelsBusy()) {
                robot.tellMotorOutput();
            }

            robot.stopAllMotors();
            robot.encoderRunningMode();
            sleep(pause);
            robot.encoderReset();

        }
    }

    public void moveDiagonalRight(int ticks, long pause) {
        //This moves along the 45/225 axis, Positive ticks move forward and negative move back
        if (opModeIsActive()) {
            robot.setTargets("Diagonal Right", ticks);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.powerSet(speed);

            while (opModeIsActive() && robot.isWheelsBusy()) {
                robot.tellMotorOutput();
            }

            robot.stopAllMotors();
            robot.encoderRunningMode();
            sleep(pause);
            robot.encoderReset();
        }
    }

    public void moveDiagonalLeft(int ticks, long pause) {
        //moves along the 135/315 axis, positive ticks move forward and negative ticks move back
        if (opModeIsActive()) {
            robot.setTargets("Diagonal Left", ticks);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.powerSet(speed);

            while (opModeIsActive() && robot.isWheelsBusy()) {
                robot.tellMotorOutput();
            }

            robot.stopAllMotors();
            robot.encoderRunningMode();
            sleep(pause);
            robot.encoderReset();
        }
    }


    public void timeDriveForward(long time, long pause)
    {//time is in milliseconds
        ElapsedTime timer = new ElapsedTime();
        robot.encoderRunningMode();
        timer.reset();
        while (timer.milliseconds() < time)
        {
            robot.frontLeft.setPower(-speed);
            robot.rearLeft.setPower(-speed);
            robot.frontRight.setPower(-speed);
            robot.rearRight.setPower(-speed);
        }
        robot.stopAllMotors();
        sleep(pause);
    }

    public void timeDriveBackward(long time, long pause)
    {//time is in milliseconds
        ElapsedTime timer = new ElapsedTime();
        robot.encoderRunningMode();
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < time)
        {
            robot.frontLeft.setPower(speed);
            robot.rearLeft.setPower(speed);
            robot.frontRight.setPower(speed);
            robot.rearRight.setPower(speed);
        }
        robot.stopAllMotors();
        sleep(pause);
    }

    public void timeDriveRight(long time, long pause)
    {//time is in milliseconds
        ElapsedTime timer = new ElapsedTime();
        robot.encoderRunningMode();
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < time)
        {
            robot.frontLeft.setPower(-speed);
            robot.rearLeft.setPower(speed);
            robot.frontRight.setPower(speed);
            robot.rearRight.setPower(-speed);
        }
        robot.stopAllMotors();
        sleep(pause);
    }

    public void timeDriveLeft(long time, long pause)
    {//time is in milliseconds
        ElapsedTime timer = new ElapsedTime();
        robot.encoderRunningMode();
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < time)
        {
            robot.frontLeft.setPower(speed);
            robot.rearLeft.setPower(-speed);
            robot.frontRight.setPower(-speed);
            robot.rearRight.setPower(speed);
        }
        robot.stopAllMotors();
        sleep(pause);
    }

    public void timeTurnleft(long time, long pause)
    {//time is in milliseconds
        ElapsedTime timer = new ElapsedTime();
        robot.encoderRunningMode();
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < time)
        {
            robot.frontLeft.setPower(speed);
            robot.rearLeft.setPower(speed);
            robot.frontRight.setPower(-speed);
            robot.rearRight.setPower(-speed);
        }
        robot.stopAllMotors();
        sleep(pause);
    }

    public void timeTurnRight(long time, long pause)
    {//time is in milliseconds
        ElapsedTime timer = new ElapsedTime();
        robot.encoderRunningMode();
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < time)
        {
            robot.frontLeft.setPower(-speed);
            robot.rearLeft.setPower(-speed);
            robot.frontRight.setPower(speed);
            robot.rearRight.setPower(speed);
        }
        robot.stopAllMotors();
        sleep(pause);
    }

    public void timeDiagonalRight(long time, long pause, int PosOneForward_MinusOneBack)
    {// This moves along the 45/225 axis. Changing the last int to -1 will make it go back, pos 1 will go forward
        ElapsedTime timer = new ElapsedTime();
        robot.encoderRunningMode();
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < time)
        {
            robot.frontLeft.setPower(-speed *  PosOneForward_MinusOneBack);
            robot.rearLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.rearRight.setPower(-speed * PosOneForward_MinusOneBack);
        }
        robot.stopAllMotors();
        sleep(pause);
    }

    public void timeDiagonalLeft(long time, long pause, int PosOneForward_MinusOneBack)
    {//Moves along the 135/315 degree axis. Changing the last int to -1 will make it go back, pos 1 will go forward
        ElapsedTime timer = new ElapsedTime();
        robot.encoderRunningMode();
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < time)
        {
            robot.frontLeft.setPower(0);
            robot.rearLeft.setPower(-speed * PosOneForward_MinusOneBack);
            robot.frontRight.setPower(-speed * PosOneForward_MinusOneBack);
            robot.rearRight.setPower(0);
        }
        robot.stopAllMotors();
        sleep(pause);
    }
    public void supperBasket() {
        robot.liftMotor.setTargetPosition(2000);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setPower(0.7);
        while (robot.liftMotor.isBusy())
        {
            telemetry.addLine("Linear upper Basket");
            telemetry.addData("Target Slides1", "%7d", robot.liftMotor.getTargetPosition());

            telemetry.addData("Actual Slides1", "%7d", robot.liftMotor.getCurrentPosition());

            telemetry.update();
        }


    }
    public void slowerBasket(){
        robot.liftMotor.setTargetPosition(1000);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setPower(0.7);
        while (robot.liftMotor.isBusy())
        {
            telemetry.addLine("Linear Lower Basket");
            telemetry.addData("Target Slides1", "%7d", robot.liftMotor.getTargetPosition());

            telemetry.addData("Actual Slides1", "%7d", robot.liftMotor.getCurrentPosition());

            telemetry.update();
        }

    }
    public void sstop(){
        robot.liftMotor.setTargetPosition(100);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setPower(0.7);
        while (robot.liftMotor.isBusy())
        {
            telemetry.addLine("down position");
            telemetry.addData("Target Slides1", "%7d", robot.liftMotor.getTargetPosition());

            telemetry.addData("Actual Slides1", "%7d", robot.liftMotor.getCurrentPosition());

            telemetry.update();
        }


    }
    public void eject(){
        robot.flapper.setPower(-7);

    }
    public void grabber() {
        robot.flapper.setPower(7);
    }
    public void armMove(int target){
        controller.setPID(p,i,d);
        int armpos1 = robot.armMotor.getCurrentPosition();
        targetposition=target;

        double armPID = controller.calculate(armpos1,targetposition);

        double armFF = Math.cos(Math.toRadians(targetposition / ticks_in_degree)) * f;

        double armPower = armPID + armFF;

        robot.armMotor.setTargetPosition(targetposition);
        robot.armMotor.setPower(armPower);

    }


     /*   //Make a timer for running the intake spit-out
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < 1000 )
        {
            tellMotorOutput();
        }
        timer.reset();
        while (timer.milliseconds() < 1000 )
        {
            intake_spin(-.5);
        }
        intake_spin(0);*/







    public void setMotorTolerance(int ticks) {
        robot.frontLeft.setTargetPositionTolerance(ticks);
        robot.frontRight.setTargetPositionTolerance(ticks);
        robot.rearLeft.setTargetPositionTolerance(ticks);
        robot.rearRight.setTargetPositionTolerance(ticks);
    }

    public void prepareNextAction(long pause) {
        sleep(pause);
        robot.encoderReset();
    }


    public int convertInchesToTicks(int inches){
        int ticks = (int) ((537.6 * inches) / (3.77953 * 3.1415926535));
        return ticks;
    }


}