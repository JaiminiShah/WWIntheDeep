package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="WW_AutoBR2024", group="Autonomous")
public class WW_AutoBR2024 extends AutoLinearAbstract2023 {
    DcMotorEx armMotor,
            liftMotor;

    CRServoImplEx flapper;

    ElapsedTime waitTimer;

    public void wait(double waitTime) {
        waitTimer = new ElapsedTime();
        // waitTimer.reset();
        while (waitTimer.seconds() < waitTime) {
        }
        waitTimer.reset();
    }


    @Override
    public void runOpMode() {

        // Execute the typical autonomous program elements.
        // super.runOpMode finishes as soon as the Drive Station start/play button is pressed.
        RunAutoInput = true;
        super.runOpMode();


        driveTrain.goStraightToTarget(24.5, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Going to place pixel on spike");
            if (Kill(28)) {
                break;
            }
        }
        turnToHeading(-90);
        driveTrain.goStraightToTarget(6, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Going to place pixel on spike");
            if (Kill(28)) {
                break;
            }
        }

        telemetry.addLine("Autonomous Done");
        telemetry.update();
    }
}




