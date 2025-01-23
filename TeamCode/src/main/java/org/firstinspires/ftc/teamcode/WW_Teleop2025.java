package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Config
@TeleOp(name="WW_Teleop2025", group="WiredWoodmen")
public class WW_Teleop2025 extends OpMode {
    DcMotorEx
            liftMotor = null,
           armMotor=null;
    // ServoImplEx intake1;
    CRServoImplEx flapper;
    final double ARM_TICKS_PER_DEGREE =
            28 *(250047.0/4913.0)*(100.0/20.0)*1/360.0;
    final double liftIncrement=23.0;
    double kp=0.77
            ,ki=0.003
            ,kd=0.004,
            f=0.01;
    double GROUND_POS=0;
    double ARM_CLEAR_BARRIER=70;
    double LOW_BASKET=280;
    double HIGH_BASKET=320;
    double armPosition;
    double armPower;

    private double integralSum = 0;
    private double lastError = 0;

    private final double ticks_in_degree=610.8/360;
    double target;
    double state;
    double tolerance;
    HardwareMap hwMap = null;
    ElapsedTime runTime = new ElapsedTime();
    double liftPosition = 0;
    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double FLAPPER_IN   = 0.7;
    final double FLAPPER_OUT  = -0.7;
    final double FLAPPER_STOP=0.0;
    double liftposincrement=23.0;

    @Override
    public void init() {
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        armMotor=hardwareMap.get(DcMotorEx.class,"armMotor");
        /* Define and initialize servos.*/
        flapper = hardwareMap.get(CRServoImplEx.class, "flapper");
        //Direction?
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setPower(0);
        armMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Setting motors to run without encoders
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);
        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        armMotor.setTargetPosition(0);
       // armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0);
        liftMotor.setTargetPosition(0);
       // liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0);
        flapper.setPower(0.0);
        // intake1.setPosition(INTAKE_OFF);
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //this will send a telemetry message to signify robot waiting;
        telemetry.addLine("AUTOBOTS ROLL OUT");
        telemetrylift();

        telemetry.update();
        runTime.reset();


    }
    @Override
    public void start() {
        armMotor.setTargetPosition((int)GROUND_POS);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setVelocity(400);
        liftMotor.setTargetPosition(0);
        // liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.0);
        telemetrylift();
    }
    public double update(double target, double state, double tolerance) {
        double error = target - state; // Calculate current error
        double deltaTime = runTime.seconds(); // Time elapsed since last update

        // Check if error is within tolerance
        if (Math.abs(error) < tolerance) {
            integralSum = 0; // Reset integral if within tolerance
            lastError = error; // Update last error
            runTime.reset(); // Reset timer for next update
            return 0; // Motor power set to 0 if within tolerance
        }

        // Proportional term
        double pTerm = kp * error;

        // Integral term (accumulate error over time)
        integralSum += error * deltaTime;
        double iTerm = ki * integralSum;

        // Derivative term (rate of change of error)
        double derivative = (error - lastError) / deltaTime;
        double dTerm = kd * derivative;
        double ff=Math.cos(Math.toRadians(target/ticks_in_degree))*f;

        // PID output
        double output=pTerm+iTerm+dTerm;
       // double output = pTerm + iTerm + dTerm +ff ;
        //double output1=pTerm+iTerm+dTerm;

        // Update last error and reset timer for next iteration
        lastError = error;
        runTime.reset(); // Reset timer after each update

        return output;
    }


    @Override
    public void loop() {
        /* Program for slides to raise up and down */
        if (gamepad2.right_bumper  && liftPosition>0) {
            liftPosition -= liftposincrement;
        }
        if(gamepad2.left_bumper && liftPosition<2300){
            liftPosition+=liftposincrement;
        }
        /*here we check to see if the lift is trying to go higher than the maximum extension.
         *if it is, we set the variable to the max.
         */
        if (liftPosition > 2300){
            liftPosition = 2300;
        }
        //same as above, we see if the lift is trying to go below 0, and if it is, we set it to 0.
        if (liftPosition < 0){
            liftPosition = 0;
        }

        liftMotor.setTargetPosition((int) (liftPosition));
        // liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) liftMotor).setVelocity(400);

        //  liftMotor.setPower(0.7);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Controls for intake
        if (gamepad2.left_bumper) {
            flapper.setPower(FLAPPER_IN);
        }

        else if (gamepad2.right_bumper) {
            flapper.setPower(FLAPPER_OUT);
        }

        else {
            flapper.setPower(FLAPPER_STOP);
        }

        // Controls for armRotator
        if (gamepad2.a) {

            armPower = update(LOW_BASKET,armMotor.getCurrentPosition(),15);
            //armPower1=pid1.update(LOW_BASKET,armRotator.getCurrentPosition(),15);
            armPosition=LOW_BASKET;
            // wrist.setPosition(.7);
        }

        else if (gamepad2.b) {
            armPower = update(HIGH_BASKET,armMotor.getCurrentPosition(),15);
            //armPower1=pid1.update(HIGH_BASKET,armRotator.getCurrentPosition(),15);
            armPosition=HIGH_BASKET;

            // armPosition = LOW_BASKET;
            //wrist.setPosition(.5);
        }

        else if (gamepad2.y) {
            armPower = update(ARM_CLEAR_BARRIER,armMotor.getCurrentPosition(),15);
            //   armPower1=pid1.update(ARM_CLEAR_BARRIER,armRotator.getCurrentPosition(),8);
            armPosition = ARM_CLEAR_BARRIER;
            //wrist.setPosition(.5);
        } else if (gamepad2.x) {
            armPower = update(GROUND_POS,armMotor.getCurrentPosition(),15);
            //armPower1=pid1.update(GROUND_POS,armRotator.getCurrentPosition(),8);
            armPosition = GROUND_POS;
            //wrist.setPosition(.5);
        }


        armMotor.setTargetPosition((int)(armPosition));
        armMotor.setPower(armPower);
        //((DcMotorEx) armRotator).setVelocity(200);
        // armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //armRotator2.setTargetPosition((int) (armPosition));
        // armRotator2.setPower(armPower1);
        // ((DcMotorEx) armRotator2).setVelocity(200);
        //armRotator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        //==========================================================//
        //                        Telemetry                           //
        //==========================================================//

        telemetrylift();

        telemetryarm();




    }
    public void telemetrylift(){
        telemetry.addData("lift variable", liftPosition);
        telemetry.addData("Lift Target Position",liftMotor.getTargetPosition());
        telemetry.addData("Lift motor velocity",liftMotor.getVelocity());
        telemetry.addData("lift current position", liftMotor.getCurrentPosition());
        telemetry.addData("liftMotor Current:",(liftMotor.getCurrent(CurrentUnit.AMPS)));
        telemetry.update();

    }
    public void telemetryarm(){
        telemetry.addData("arm Target Position: ", armMotor.getTargetPosition());
        telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
        telemetry.addData("Lift motor velocity",armMotor.getVelocity());
        telemetry.addData("Arm Motor current",armMotor.getCurrent(CurrentUnit.AMPS));
    }

}
