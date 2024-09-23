package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="WW_Teleop2024", group="GreenwoodFTC") //declares the name of the class and the

public class WW_Teleop2024 extends OpMode{
    double
            DRIVE_TRAIN_SPEED = 0.7,
            controllerDeadZone = 0.3;
    DcMotorEx
            frontLeftDrive = null,
            frontRightDrive = null,
            rearLeftDrive = null,
            rearRightDrive = null;
    HardwareMap hwMap =  null;
    ElapsedTime runTime = new ElapsedTime();


    @Override
    public void init() {
        //Configuring the RobotController with the same device name and Mapping device to HardwareMap
        rearLeftDrive=hardwareMap.get(DcMotorEx.class, "rear_left_drive");

        rearRightDrive = hardwareMap.get(DcMotorEx.class, "rear_right_drive");
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        // Direction
        frontLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        //Resetting the Encoder
        frontLeftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //Setting motors to run without encoders
        frontLeftDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rearRightDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        // Setting power to zero
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);


    }

    @Override
    public void loop() {

        float FLspeed = -gamepad1.left_stick_y + gamepad1.left_stick_x;
        float BLspeed = -gamepad1.left_stick_y - gamepad1.left_stick_x;
        float FRspeed = -gamepad1.right_stick_y - gamepad1.right_stick_x;
        float BRspeed = -gamepad1.right_stick_y + gamepad1.right_stick_x;


        telemetry.addData("Value of Left_stick Y", gamepad1.left_stick_y);
        telemetry.addData("Value of Left Stick X", gamepad1.left_stick_x);
        telemetry.addData("Value of Right Stick Y", gamepad1.right_stick_y);
        telemetry.addData("Value of Right Stick X", gamepad1.right_stick_x);
        telemetry.addData("FLspeed", FLspeed);
        telemetry.addData("BLspeed", BLspeed);
        telemetry.addData("FRspeed", FRspeed);
        telemetry.addData("BRspeed", BRspeed);
        telemetry.update();

        if (false) {
            rearLeftDrive.setPower(BLspeed * DRIVE_TRAIN_SPEED);
            rearRightDrive.setPower(BRspeed * DRIVE_TRAIN_SPEED);
            frontLeftDrive.setPower(FLspeed * DRIVE_TRAIN_SPEED);
            frontRightDrive.setPower(FRspeed * DRIVE_TRAIN_SPEED);
        } else {
            boolean left_dead_y = Math.abs(gamepad1.left_stick_y) < controllerDeadZone;
            boolean left_dead_x = Math.abs(gamepad1.left_stick_x) < controllerDeadZone;
            boolean right_dead_x = Math.abs(gamepad1.right_stick_x) < controllerDeadZone;
            boolean right_dead_y = Math.abs(gamepad1.right_stick_y) < controllerDeadZone;
            if (left_dead_x && left_dead_y && right_dead_x && right_dead_y) {
                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                rearLeftDrive.setPower(0);
                rearRightDrive.setPower(0);
            } else {
                rearLeftDrive.setPower(BLspeed * DRIVE_TRAIN_SPEED);
                rearRightDrive.setPower(BRspeed * DRIVE_TRAIN_SPEED);
                frontLeftDrive.setPower(FLspeed * DRIVE_TRAIN_SPEED);
                frontRightDrive.setPower(FRspeed * DRIVE_TRAIN_SPEED);
            }
        }

    }
    @Override
    public void stop(){
        frontLeftDrive.setPower(0);
        rearLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearRightDrive.setPower(0);

    }
}
