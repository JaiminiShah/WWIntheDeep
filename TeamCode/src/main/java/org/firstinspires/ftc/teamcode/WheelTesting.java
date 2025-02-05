package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(group = "Tests", name = "Wheel Test")
public class WheelTesting extends OpMode {
    DcMotorEx
            rearLeft = null,
            rearRight = null,
            frontLeft = null,
            frontRight = null;
    private DcMotor[] motors = null;
    private String[] motorNames = null;

    private Gamepad gamepad1Previous = new Gamepad();
    private Gamepad gamepad1Current = new Gamepad();

    private short motorNumberToTest = 0;


    @Override
    public void init() {
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");

        rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        //Direction?
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        motors = new DcMotor[] {frontLeft, rearLeft, frontRight, rearRight};
        motorNames = new String[] {"Front Left Motor", "Back Left Motor", "Front Right Motor", "Back Right Motor"};
    }

    @Override
    public void loop() {
        gamepad1Previous.copy(gamepad1Current);
        gamepad1Current.copy(gamepad1);

        if (gamepad1Current.dpad_up && !gamepad1Previous.dpad_up) {
            motorNumberToTest = (short) ((motorNumberToTest + 1) % 4);
        }

        if (gamepad1Current.dpad_down && !gamepad1Previous.dpad_down) {
            if (motorNumberToTest == 0) {
                motorNumberToTest = (short) (motors.length - 1);
            } else {
                motorNumberToTest = (short) ((motorNumberToTest - 1) % 4);
            }
        }

        if (gamepad1Current.dpad_left) {
            motors[motorNumberToTest].setDirection(DcMotor.Direction.REVERSE);
        }

        if (gamepad1Current.dpad_right) {
            motors[motorNumberToTest].setDirection(DcMotor.Direction.FORWARD);
        }

        if (gamepad1Current.b) {
            motors[motorNumberToTest].setPower(1);
        } else {
            motors[motorNumberToTest].setPower(0);
        }

        telemetry.addData("Motor Number: ", motorNumberToTest);
        telemetry.addData("Motor Name: ", motorNames[motorNumberToTest]);
        telemetry.addData("Motor Direction: ", motors[motorNumberToTest].getDirection());
        telemetry.addData("Motor Power: ", motors[motorNumberToTest].getPower());
    }
}