package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp
public class Time_Auto extends LinearOpMode {
    DcMotorEx
            rearLeft = null,
            rearRight = null,
            frontLeft = null,
            frontRight = null;

    @Override
    public void runOpMode() {
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");

        rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
              // Wait for the game to start (driver presses PLAY)
        //Direction?
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        ElapsedTime runtime = new ElapsedTime();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            while (runtime.seconds() < .75) {
                frontLeft.setPower(1);
                frontRight.setPower(1);
                rearLeft.setPower(-1);
                rearRight.setPower(-1);
            }
            frontRight.setPower(0);
            frontLeft.setPower(0);
            rearLeft.setPower(0);
            rearRight.setPower(0);
            while (runtime.seconds() < 4) {
                frontLeft.setPower(1);
                frontRight.setPower(1);
                rearRight.setPower(1);
                rearLeft.setPower(1);
            }
            while (runtime.seconds() < 4.3) {
                frontLeft.setPower(-1);
                frontRight.setPower(-1);
                rearLeft.setPower(1);
                rearRight.setPower(1);
            }
            while (runtime.seconds() < 8) {
                frontRight.setPower(-1);
                frontLeft.setPower(-1);
                rearLeft.setPower(-1);
                rearRight.setPower(-1);
            }
            while (runtime.seconds() < 11) {
                frontLeft.setPower(1);
                frontRight.setPower(1);
                rearLeft.setPower(1);
                rearRight.setPower(1);
            }

            frontRight.setPower(0);
            frontLeft.setPower(0);
            rearRight.setPower(0);
            rearLeft.setPower(0);

            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}
