package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name="Arm Tuning", group="Tests")

public class ArmTesting extends LinearOpMode {

    public DcMotor armMotor      = null;

    public final double armTicksPerDegree = 19.7924893140647; //exact fraction is (194481/9826)


    @Override
    public void runOpMode() {

        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            armMotor.setPower(0);

            telemetry.addData("Arm Ticks", armMotor.getCurrentPosition());
            telemetry.addData("Arm Degrees", armMotor.getCurrentPosition() / armTicksPerDegree);

            telemetry.update();

        }
    }
}