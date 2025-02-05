package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// TEST DURING MEETING

@Autonomous
public class Auton_IMU extends LinearOpMode {

    DcMotorEx
            rearLeft = null,
            rearRight = null,
            frontLeft = null,
            frontRight = null,
            liftMotor = null,
            armMotor=null;
    CRServoImplEx flapper;
    ElapsedTime waitTimer;
    public PIDController controller;
    public static double p=0,i=0,d=0;
    public static double f=0;
    public static int targetposition=0;
    private final double ticks_in_degree=537.6/360;


    public void wait(double waitTime) {
        waitTimer = new ElapsedTime();
        // waitTimer.reset();
        while (waitTimer.seconds() < waitTime) {
        }
        waitTimer.reset();
    }

    private IMU imu;
    private Orientation angles;

    // double cpi = 50; (this cpi was too short, around 2.5 inches- keeping as backup tho!)
    double cpi = 60;

    // Target distance to move (in inches)
    private double targetDistance = 1;

    // Motor power
    private double motorPower = 0.5;

    // Estimated robot speed (inches per second)
    private double robotSpeed = 10; // Adjust this value based on your robot's speed

    @Override
    public void runOpMode() throws InterruptedException {
        controller=new PIDController(p,i,d);
        // Initialize hardware map
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");

        rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotor.setPower(0);
        armMotor.setPower(0);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Setting motors to run without encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");

        waitForStart();


        // COMMENT AND TEST ONE AT A TIME

        // BLUE OBSERVATION
     /*   SClaw.setPosition(0.5);
        linearup(11.5, 0.5);
        SWrist1.setPosition(0.65);
        SWrist2.setPosition(0.65);
        sleep(500);
        backward(19, 0.4);
        sleep(500);
        SClaw.setPosition(0.9);
        sleep(500);
        // set back
        SWrist1.setPosition(0.95);
        SWrist2.setPosition(0.95);
        forward(5, 0.4);
        sleep(500);
        lineardown(11.5, 0.5);
        sleep(200);
        strafeLeft(17, 0.7);
        sleep(500);
        spinRight(40, 0.3);// distance 40, power 0.3 is magical
        SWrist1.setPosition(0.95);
        SWrist2.setPosition(0.95);
        backward(11, 0.3);
        sleep(500);
        SClaw.setPosition(0.5);
        sleep(200);
        linearup(11.5, 0.5);
        // wrist set position
        SWrist1.setPosition(0.65);
        SWrist2.setPosition(0.65);
        sleep(1000);*/

//        SClaw.setPosition(0.9);



        // THIS CODE HAS NOT BEEN TESTED
    /*   strafeRight(20, 0.3);
        strafeRight(5, 0.3);
        backward(15, 0.3);

        strafeRight(6, 0.3);
        backward(40, 0.3); // push "sample" into parking
        forward(40, 0.3);
        strafeRight(12, 0.3);
        backward(40, 0.3); // push "sample" into parking
        forward(40, 0.3);
        strafeRight(12, 0.3);
        backward(40, 0.3); // push "sample" into parking
        strafeLeft(4, 0.3);
        backward(2, 0.3); // PARK*/


    }

    private void forward(double distance, double power) {
        // Get initial heading
        angles   = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double initialHeading = angles.firstAngle;

        // Start moving forward
        frontRight.setPower(power);
        frontLeft.setPower(power);
        rearRight.setPower(power);
        rearLeft.setPower(power);

        // Get the current time in milliseconds
        long startTime = System.currentTimeMillis();

        // Move for estimated time
        while ((System.currentTimeMillis() - startTime) < (distance / robotSpeed) * 1000) {
            // Get current heading
            angles   = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentHeading = angles.firstAngle;

            // Calculate heading error
            double headingError = currentHeading - initialHeading;

            // Adjust motor powers based on heading error (optional)
            // double leftPower = power - headingError * kP;
            // double rightPower = power + headingError * kP;

            // Set motor powers (with optional heading correction)
            // BR.setPower(leftPower);
            // BL.setPower(leftPower);
            // FR.setPower(rightPower);
            // FL.setPower(rightPower);

            // For now, keep motor powers constant
            frontLeft.setPower(power);
            frontRight.setPower(power);
            rearLeft.setPower(power);
            rearRight.setPower(power);

            // Calculate elapsed time
            double elapsedTime = (System.currentTimeMillis() - startTime) / 1000.0;

            // Estimate distance traveled
            double estimatedDistance = elapsedTime * robotSpeed;

            // Display telemetry
            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Motor Power", power);
            telemetry.addData("Time Elapsed", elapsedTime);
            telemetry.addData("Estimated Distance", estimatedDistance);
            telemetry.update();

            sleep(10); // Adjust sleep time for telemetry updates
        }

        // Stop motors
        frontRight.setPower(0);
        frontLeft.setPower(0);
        rearRight.setPower(0);
        rearLeft.setPower(0);
    }

    private void backward(double distance, double power) {
        // Get initial heading
        angles   = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double initialHeading = angles.firstAngle;

        // Start moving backward
        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        rearLeft.setPower(-power);
        rearRight.setPower(-power);

        // Get the current time in milliseconds
        long startTime = System.currentTimeMillis();

        // Move for estimated time
        while ((System.currentTimeMillis() - startTime) < (distance / robotSpeed) * 1000) {
            // Get current heading
            angles   = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentHeading = angles.firstAngle;

            // Calculate heading error
            double headingError = currentHeading - initialHeading;

            // Adjust motor powers based on heading error (optional)
            // double leftPower = power - headingError * kP;
            // double rightPower = power + headingError * kP;

            // Set motor powers (with optional heading correction)
            // BR.setPower(leftPower);
            // BL.setPower(leftPower);
            // FR.setPower(rightPower);
            // FL.setPower(rightPower);

            // For now, keep motor powers constant
            rearLeft.setPower(-power);
            rearRight.setPower(-power);
            frontRight.setPower(-power);
            frontLeft.setPower(-power);

            // Calculate elapsed time
            double elapsedTime = (System.currentTimeMillis() - startTime) / 1000.0;

            // Estimate distance traveled
            double estimatedDistance = elapsedTime * robotSpeed;

            // Display telemetry
            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Motor Power", power);
            telemetry.addData("Time Elapsed", elapsedTime);
            telemetry.addData("Estimated Distance", estimatedDistance);
            telemetry.update();

            sleep(10); // Adjust sleep time for telemetry updates
        }

        // Stop motors
        frontRight.setPower(0);
        frontLeft.setPower(0);
        rearRight.setPower(0);
        rearLeft.setPower(0);
    }

    private void spinRight(double distance, double power) {
        // Get initial heading
        angles   = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double initialHeading = angles.firstAngle;

        // Spin
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        rearLeft.setPower(-power);
        rearRight.setPower(power);

        // Get the current time in milliseconds
        long startTime = System.currentTimeMillis();

        // Move for estimated time
        while ((System.currentTimeMillis() - startTime) < (distance / robotSpeed) * 1000) {
            // Get current heading
            angles   = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentHeading = angles.firstAngle;

            // Calculate heading error
            double headingError = currentHeading - initialHeading;

            // Adjust motor powers based on heading error (optional)
            // double leftPower = power - headingError * kP;
            // double rightPower = power + headingError * kP;

            // Set motor powers (with optional heading correction)
            // BR.setPower(leftPower);
            // BL.setPower(leftPower);
            // FR.setPower(rightPower);
            // FL.setPower(rightPower);

            // For now, keep motor powers constant
            frontRight.setPower(-power);
            frontLeft.setPower(power);
            rearRight.setPower(-power);
            rearLeft.setPower(power);

            // Calculate elapsed time
            double elapsedTime = (System.currentTimeMillis() - startTime) / 1000.0;

            // Estimate distance traveled
            double estimatedDistance = elapsedTime * robotSpeed;

            // Display telemetry
            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Motor Power", power);
            telemetry.addData("Time Elapsed", elapsedTime);
            telemetry.addData("Estimated Distance", estimatedDistance);
            telemetry.update();

            sleep(10); // Adjust sleep time for telemetry updates
        }

        // Stop motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }

    private void strafeLeft (double distance,  double power) {

        // Get initial heading
        angles   = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double initialHeading = angles.firstAngle;

        // sets desired power for motors
        rearLeft.setPower(-power);
        rearRight.setPower(power);
        frontRight.setPower(power);
        frontLeft.setPower(-power);

        // Get the current time in milliseconds
        long startTime = System.currentTimeMillis();

        // Move for estimated time
        while ((System.currentTimeMillis() - startTime) < (distance / robotSpeed) * 1000) {
            // Get current heading
            angles = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentHeading = angles.firstAngle;

            // Calculate heading error
            double headingError = currentHeading - initialHeading;

            // Adjust motor powers based on heading error (optional)
            // double leftPower = power - headingError * kP;
            // double rightPower = power + headingError * kP;

            // Set motor powers (with optional heading correction)
            // BR.setPower(leftPower);
            // BL.setPower(leftPower);
            // FR.setPower(rightPower);
            // FL.setPower(rightPower);

            // For now, keep motor powers constant
            rearRight.setPower(power);
            rearLeft.setPower(-power);
            frontRight.setPower(power);
            frontLeft.setPower(-power);

            // Calculate elapsed time
            double elapsedTime = (System.currentTimeMillis() - startTime) / 1000.0;

            // Estimate distance traveled
            double estimatedDistance = elapsedTime * robotSpeed;

            // Display telemetry
            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Motor Power", power);
            telemetry.addData("Time Elapsed", elapsedTime);
            telemetry.addData("Estimated Distance", estimatedDistance);
            telemetry.update();

            sleep(10); // Adjust sleep time for telemetry updates
        }

        // stop motors
        rearLeft.setPower(0);
        rearRight.setPower(0);
        frontRight.setPower(0);
        frontLeft.setPower(0);
    }

    private void strafeRight (double distance,  double power) {

        // Get initial heading
        angles   = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double initialHeading = angles.firstAngle;

        // sets desired power for motors
        rearRight.setPower(-power);
        rearLeft.setPower(-power);
        frontRight.setPower(power);
        frontLeft.setPower(power);


        // Get the current time in milliseconds
        long startTime = System.currentTimeMillis();

        // Move for estimated time
        while ((System.currentTimeMillis() - startTime) < (distance / robotSpeed) * 1000) {
            // Get current heading
            angles = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentHeading = angles.firstAngle;

            // Calculate heading error
            double headingError = currentHeading - initialHeading;

            // Adjust motor powers based on heading error (optional)
            // double leftPower = power - headingError * kP;
            // double rightPower = power + headingError * kP;

            // Set motor powers (with optional heading correction)
            // BR.setPower(leftPower);
            // BL.setPower(leftPower);
            // FR.setPower(rightPower);
            // FL.setPower(rightPower);
            // For now, keep motor powers constant
            rearLeft.setPower(-power);
            rearRight.setPower(-power);
            frontRight.setPower(power);
            frontLeft.setPower(power);

            // Calculate elapsed time
            double elapsedTime = (System.currentTimeMillis() - startTime) / 1000.0;

            // Estimate distance traveled
            double estimatedDistance = elapsedTime * robotSpeed;

            // Display telemetry
            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Motor Power", power);
            telemetry.addData("Time Elapsed", elapsedTime);
            telemetry.addData("Estimated Distance", estimatedDistance);
            telemetry.update();

            sleep(10); // Adjust sleep time for telemetry updates

        }
        // stop motors
        rearRight.setPower(0);
        rearLeft.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
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

        wait(0.2);
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

        wait(0.2);
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

        wait(0.2);
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

}