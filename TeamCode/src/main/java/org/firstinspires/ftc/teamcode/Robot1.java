package org.firstinspires.ftc.teamcode;



import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

public class Robot1 {

    public DcMotorEx rearLeft;
    public DcMotorEx rearRight;
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx liftMotor;
    public DcMotorEx armMotor;
    public CRServo flapper;
    public boolean teleopEncoderMode;
    public boolean teleopPowerMode;
    public PIDController controller;
    public static double p=0,i=0,d=0;
    public static double f=0;
    public static int targetposition=0;
    private final double ticks_in_degree=537.6/360;

    //public DistanceSensor distanceSensor;


    //public WebcamName CamCam;

    public Telemetry telemetry;
    //public BNO055IMU imu;

    //init and declare war
    public OpMode opmode;
    public HardwareMap hardwareMap;
    public static double parkingZone;
    public String startingPosition;
    public String controlMode = "Field Centric";// Robot Centric
    public String intakeFlipperPos ="UP";
    public IMU.Parameters imuParameters;

    //Initialize motors and servos
    public Robot1(HardwareMap hardwareMap, Telemetry telemetry, OpMode opmode){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.opmode = opmode;

        // This section turns the names of the pieces of hardware into variables that we can program with.
        // Make sure that the device name is the exact same thing you typed in on the configuration on the driver hub.
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");

        rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.FORWARD);//Might need inverted
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);//Might need inverted

        // This tells the motors to chill when we're not powering them.
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //This is new..
        telemetry.addData("Status", "Initialized");

    }


    public boolean isWheelsBusy(){
        return frontRight.isBusy() || frontLeft.isBusy() || rearRight.isBusy() || rearLeft.isBusy();
    }

    public void stopAllMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearRight.setPower(0);
        rearLeft.setPower(0);
    }

    public void setTargets(String direction, int ticks) {

        //This is all inverted (big sigh)

        if (Objects.equals(direction, "Right")){
            frontLeft.setTargetPosition(-ticks + frontLeft.getCurrentPosition());
            frontRight.setTargetPosition(ticks + frontRight.getCurrentPosition());
            rearLeft.setTargetPosition(ticks + rearLeft.getCurrentPosition());
            rearRight.setTargetPosition(-ticks + rearRight.getCurrentPosition());

        } else if (Objects.equals(direction, "Left")){
            frontLeft.setTargetPosition(ticks + frontLeft.getCurrentPosition());
            frontRight.setTargetPosition(-ticks + frontRight.getCurrentPosition());
            rearLeft.setTargetPosition(-ticks + rearLeft.getCurrentPosition());
            rearRight.setTargetPosition(ticks + rearRight.getCurrentPosition());

        } else if (Objects.equals(direction,"Forward")){
            frontLeft.setTargetPosition(-ticks + frontLeft.getCurrentPosition());
            frontRight.setTargetPosition(-ticks + frontRight.getCurrentPosition());
            rearLeft.setTargetPosition(-ticks + rearLeft.getCurrentPosition());
            rearRight.setTargetPosition(-ticks + rearRight.getCurrentPosition());

        } else if (Objects.equals(direction,"Backward")) {
            frontLeft.setTargetPosition(ticks + frontLeft.getCurrentPosition());
            frontRight.setTargetPosition(ticks + frontRight.getCurrentPosition());
            rearLeft.setTargetPosition(ticks + rearLeft.getCurrentPosition());
            rearRight.setTargetPosition(ticks + rearRight.getCurrentPosition());

        } else if (Objects.equals(direction,"Turn Right")) {
            frontLeft.setTargetPosition(-ticks + frontLeft.getCurrentPosition());
            frontRight.setTargetPosition(ticks + frontRight.getCurrentPosition());
            rearLeft.setTargetPosition(-ticks + rearLeft.getCurrentPosition());
            rearRight.setTargetPosition(ticks + rearRight.getCurrentPosition());

        } else if (Objects.equals(direction, "Turn Left")) {
            frontLeft.setTargetPosition(ticks + frontLeft.getCurrentPosition());
            frontRight.setTargetPosition(-ticks + frontRight.getCurrentPosition());
            rearLeft.setTargetPosition(ticks + rearLeft.getCurrentPosition());
            rearRight.setTargetPosition(-ticks + rearRight.getCurrentPosition());
        }
        else if (Objects.equals(direction, "Diagonal Right")) {
            frontLeft.setTargetPosition(-ticks + frontLeft.getCurrentPosition());
            frontRight.setPower(0);
            rearLeft.setPower(0);
            rearRight.setTargetPosition(-ticks + rearRight.getCurrentPosition());
        }
        else if (Objects.equals(direction, "Diagonal Left")) {
            frontLeft.setPower(0);
            frontRight.setTargetPosition(-ticks + frontRight.getCurrentPosition());
            rearLeft.setTargetPosition(-ticks + rearLeft.getCurrentPosition());
            rearRight.setPower(0);
        }




    }

    public void positionRunningMode(){

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void powerRunningMode()
    {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void powerSet(double speed) {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        rearLeft.setPower(speed);
        rearRight.setPower(speed);

    }
    public DcMotor.RunMode encoderRunningMode(){
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        return null;
    }

    public void encoderReset(){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @SuppressLint("DefaultLocale")
    public void tellMotorOutput(){
        telemetry.addData("Control Mode", controlMode);
        telemetry.addData("Motors", String.format("FL Power(%.2f) FL Location (%d) FL Target (%d)", frontLeft.getPower(), frontLeft.getCurrentPosition(), frontLeft.getTargetPosition()));
        telemetry.addData("Motors", String.format("FR Power(%.2f) FR Location (%d) FR Target (%d)", frontRight.getPower(), frontRight.getCurrentPosition(), frontRight.getTargetPosition()));
        telemetry.addData("Motors", String.format("BL Power(%.2f) BL Location (%d) BL Target (%d)", rearLeft.getPower(), rearLeft.getCurrentPosition(), rearLeft.getTargetPosition()));
        telemetry.addData("Motors", String.format("BR Power(%.2f) BR Location (%d) BR Target (%d)", rearRight.getPower(), rearRight.getCurrentPosition(), rearRight.getTargetPosition()));
        telemetry.addData("Motors", String.format("LiftyL Power (%.2f) LiftyL Location (%d) LiftyL Target (%d)", armMotor.getPower(), armMotor.getCurrentPosition(), armMotor.getTargetPosition()));
        telemetry.addData("Motors", String.format("LiftyR Power (%.2f) LiftyR Location (%d) LiftyR Target (%d)", liftMotor.getPower(), liftMotor.getCurrentPosition(), liftMotor.getTargetPosition()));
        telemetry.addData("Flipper", flapper.getPower());
        telemetry.update();
    }

    public double inchesToTicks(double inches){
        // returns the inches * ticks per rotation / wheel circ
        return ((inches/12.25) * 537.6 / .5);
        //todo Reference that 1 inch ~= 50 ticks
    }

    public void supperBasket() {
        liftMotor.setTargetPosition(2000);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.7);
        while (liftMotor.isBusy())
        {
            telemetry.addLine("Linear upper Basket");
            telemetry.addData("Target Slides1", "%7d", liftMotor.getTargetPosition());

            telemetry.addData("Actual Slides1", "%7d", liftMotor.getCurrentPosition());

            telemetry.update();
        }


    }
    public void slowerBasket(){
        liftMotor.setTargetPosition(1000);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.7);
        while (liftMotor.isBusy())
        {
            telemetry.addLine("Linear Lower Basket");
            telemetry.addData("Target Slides1", "%7d", liftMotor.getTargetPosition());

            telemetry.addData("Actual Slides1", "%7d", liftMotor.getCurrentPosition());

            telemetry.update();
        }

    }
    public void sstop(){
        liftMotor.setTargetPosition(100);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.7);
        while (liftMotor.isBusy())
        {
            telemetry.addLine("down position");
            telemetry.addData("Target Slides1", "%7d", liftMotor.getTargetPosition());

            telemetry.addData("Actual Slides1", "%7d", liftMotor.getCurrentPosition());

            telemetry.update();
        }


    }
    public void eject(){
        flapper.setPower(-7);

    }
    public void grabber() {
        flapper.setPower(7);
    }
    public void armMove(int target){
        controller.setPID(p,i,d);
        int armpos1 = armMotor.getCurrentPosition();
        targetposition=target;

        double armPID = controller.calculate(armpos1,targetposition);

        double armFF = Math.cos(Math.toRadians(targetposition / ticks_in_degree)) * f;

        double armPower = armPID + armFF;

        armMotor.setTargetPosition(targetposition);
        armMotor.setPower(armPower);

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




       /* while(intakeFlipper.getPosition() != .75)
        {
            intakePosition("UP");
            tellMotorOutput();
        }*/



  /*  // one side may be backwards due to the direction that the motor was faced
    public void moveArm(String direction){
        if (direction == "Up"){
            liftyL.setPower(1);
            liftyL.setDirection(DcMotor.Direction.FORWARD);//inverted
            liftyR.setPower(1);
            liftyR.setDirection(DcMotor.Direction.FORWARD);//inverted
        } else if (direction == "Down"){
            liftyL.setPower(0.25);
            liftyL.setDirection(DcMotor.Direction.REVERSE);//Inverted
            liftyR.setPower(0.25);
            liftyR.setDirection(DcMotor.Direction.REVERSE);//Inverted
        }
    }

    ElapsedTime timer = new ElapsedTime();

    public void holdArm(){
        liftyL.setDirection(DcMotor.Direction.FORWARD);//
        liftyL.setPower(0.05);
        liftyR.setDirection(DcMotor.Direction.FORWARD);//
        liftyR.setPower(0.05);
    }


    public boolean primaryClawClosed = false;*/





}