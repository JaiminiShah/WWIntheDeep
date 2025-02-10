package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.AutonomousPLUS;

@Autonomous(name="Blue_NetZone", group="Autonomous")
public class Blue_NetZone extends AutonomousPLUS{
    public String currentPosition;
    public String target;

    public void runOpMode() {

        super.runOpMode();


        waitForStart();

        setMotorTolerance(10);

        speed = .6;
        sleep(200);

        moveRobotForward(500, 0);
        //moveRobotRight(950, 0); // This is the code to start on a different tile
        armMove(1000);
        speed = .3;
        moveRobotForward(350, 0);
        speed = .6;
        slowerBasket();

        prepareNextAction(2);
        moveRobotBackward(425,0);

        // First sample
        moveRobotLeft(1345, 0);
        setMotorTolerance(8);
        turnRobotRight(1530, 0);
        setMotorTolerance(10);

        prepareNextAction(2000);


    }
}





