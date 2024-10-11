package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp(name="WW_Teleop2024", group="WiredWoodmen")  //declares the name of the class and the
// group it is in.



public class WW_Teleop2024 extends OpMode {


    double  //declares all double variables and their values
            speedVariable = .8;

    //float[] hsvValuesRight = new float[3];
    //float[] hsvValuesLeft = new float[3];


    /*
     * Code will run ONCE when the driver hits INIT
     * INIT means initialize
     */
    DcMotorEx
            rearLeftDrive = null,
            rearRightDrive = null,
            frontLeftDrive = null,
            frontRightDrive = null,
            armMotor = null;

    // pixelArm = null;
    double hangerpos = 0.0;
    /* This constant is the number of encoder ticks for each degree of rotation of the arm.
   To find this, we first need to consider the total gear reduction powering our arm.
   First, we have an external 20t:100t (5:1) reduction created by two spur gears.
   But we also have an internal gear reduction in our motor.
   The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
   reduction of ~50.9:1. (more precisely it is 250047/4913:1)
   We can multiply these two ratios together to get our final reduction of ~254.47:1.
   The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
   counts per rotation of the arm. We divide that by 360 to get the counts per degree. */
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1 / 360.0; // we want ticks per degree, not per rotation
    /* These constants hold the position that the arm is commanded to run to.
   These are relative to where the arm was located when you start the OpMode. So make sure the
   arm is reset to collapsed inside the robot before you start the program.

   In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
   This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
   set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
   160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
   If you'd like it to move further, increase that number. If you'd like it to not move
   as far from the starting position, decrease it. */
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 0 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER = 15 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK = 110 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT = 10 * ARM_TICKS_PER_DEGREE;
    HardwareMap hwMap = null;
    ElapsedTime runTime = new ElapsedTime();
    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;
    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;
    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_LOW_BASKET = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_HIGH_BASKET = 480 * LIFT_TICKS_PER_MM;

    double liftPosition = LIFT_COLLAPSED;
    double armLiftComp = 0;

    @Override
    public void init() { //initialization class to be used at start of tele-op

        // these are our motors and what they are called
        rearLeftDrive = hardwareMap.get(DcMotorEx.class, "rear_left_drive");

        rearRightDrive = hardwareMap.get(DcMotorEx.class, "rear_right_drive");
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

//Direction?
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // pixelArm.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);
        armMotor.setPower(0);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Setting motors to run without encoders
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);

        //this will send a telemetry message to signify robot waiting;
        telemetry.addLine("AUTOBOTS ROLL OUT");
        telemetry.update();
        runTime.reset();

    }

    /*
     * Code will run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     *this code will run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code will run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //==========================================================
        //                        GamePad One
        //==========================================================

        //Controls Drive Train

        float FLspeed = -gamepad1.left_stick_y + gamepad1.left_stick_x;
        float BLspeed = -gamepad1.left_stick_y - gamepad1.left_stick_x;
        float FRspeed = -gamepad1.right_stick_y - gamepad1.right_stick_x;
        float BRspeed = -gamepad1.right_stick_y + gamepad1.right_stick_x;

        rearLeftDrive.setPower(Range.clip((-BLspeed * speedVariable), -1, 1));
        rearRightDrive.setPower(Range.clip((BRspeed * speedVariable), -1, 1));
        frontLeftDrive.setPower(Range.clip((FLspeed * speedVariable), -1, 1));
        frontRightDrive.setPower(Range.clip((-FRspeed * speedVariable), -1, 1));


        //DriveTrain Speed Controls
        if (gamepad1.dpad_left) speedVariable -= 0.05;
        if (gamepad1.dpad_right) speedVariable += 0.05;
        speedVariable = Range.clip(speedVariable, 0, 1);
        if (gamepad1.right_trigger > 0.5 && hangerpos < 2000)
            hangerpos += 50;
        if (gamepad1.left_trigger > 0.5 && hangerpos > 40)
            hangerpos -= 40;
        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);
         /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
         /* Here we implement a set of if else statements to set our arm to different scoring positions.
            We check to see if a specific button is pressed, and then move the arm (and sometimes
            intake and wrist) to match. For example, if we click the right bumper we want the robot
            to start collecting. So it moves the armPosition to the ARM_COLLECT position,
            it folds out the wrist to make sure it is in the correct orientation to intake, and it
            turns the intake on to the COLLECT mode.*/

        if (gamepad1.a) {
            /* This is the intaking/collecting arm position */
            armPosition = ARM_COLLECT;
            //liftPosition = LIFT_COLLAPSED;

        } else if (gamepad1.b) {
                    /* This is about 20° up from the collecting position to clear the barrier
                    Note here that we don't set the wrist position or the intake power when we
                    select this "mode", this means that the intake and wrist will continue what
                    they were doing before we clicked left bumper. */
            armPosition = ARM_CLEAR_BARRIER;
        } else if (gamepad1.x) {
            /* This is the correct height to score the sample in the HIGH BASKET */
            armPosition = ARM_SCORE_SAMPLE_IN_LOW;
            //liftPosition = LIFT_SCORING_IN_HIGH_BASKET;
        } else if (gamepad1.dpad_left) {
                    /* This turns off the intake, folds in the wrist, and moves the arm
                    back to folded inside the robot. This is also the starting configuration */
            armPosition = ARM_COLLAPSED_INTO_ROBOT;
            //liftPosition = LIFT_COLLAPSED;

        } else if (gamepad1.dpad_right) {
            /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
            armPosition = ARM_SCORE_SPECIMEN;

        } else if (gamepad1.dpad_up) {
            /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
            armPosition = ARM_ATTACH_HANGING_HOOK;

        } else if (gamepad1.dpad_down) {
            /* this moves the arm down to lift the robot up once it has been hooked */
            armPosition = ARM_WINCH_ROBOT;

        }

            /*
            This is probably my favorite piece of code on this robot. It's a clever little software
            solution to a problem the robot has.
            This robot has an extending lift on the end of an arm shoulder. That arm shoulder should
            run to a specific angle, and stop there to collect from the field. And the angle that
            the shoulder should stop at changes based on how long the arm is (how far the lift is extended)
            so here, we add a compensation factor based on how far the lift is extended.
            That comp factor is multiplied by the number of mm the lift is extended, which
            results in the number of degrees we need to fudge our arm up by to keep the end of the arm
            the same distance from the field.
            Now we don't need this to happen when the arm is up and in scoring position. So if the arm
            is above 45°, then we just set armLiftComp to 0. It's only if it's below 45° that we set it
            to a value.
             */

        if (armPosition < 45 * ARM_TICKS_PER_DEGREE) {
            armLiftComp = (0.25568 * liftPosition);
        } else {
            armLiftComp = 0;
        }

           /* Here we set the target position of our arm to match the variable that was selected
            by the driver. We add the armPosition Variable to our armPositionFudgeFactor, before adding
            our armLiftComp, which adjusts the arm height for different lift extensions.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/

        armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor + armLiftComp));

        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            /* Here we set the lift position based on the driver input.
            This is a.... weird, way to set the position of a "closed loop" device. The lift is run
            with encoders. So it kno*/

        //==========================================================//
        //                        Telemetry                           //
        //==========================================================//
       telemetrymotorprint();

    }
    public void telemetrymotorprint(){
        telemetry.clear();
        telemetry.addData("Drive Train Speed: " , speedVariable);
        telemetry.addData("BRMotor2", "Position : %2d, Power : %.2f", rearRightDrive.getCurrentPosition(), rearRightDrive.getPower());
        telemetry.addData("FRMotor2", "Position : %2d, Power : %.2f", frontRightDrive.getCurrentPosition(), frontRightDrive.getPower());

        telemetry.addData("FLMotor2", "Position : %2d, Power : %.2f", frontLeftDrive.getCurrentPosition(), frontLeftDrive.getPower());
        telemetry.addData("BLMotor2", "Position : %2d, Power : %.2f", rearLeftDrive.getCurrentPosition(), rearLeftDrive.getPower());
        telemetry.addLine("left joystick | ")
                .addData("x", gamepad1.left_stick_x)
                .addData("y", gamepad1.left_stick_y);
        telemetry.addLine("right joystick | ")
                .addData("x", gamepad1.right_stick_x)
                .addData("y", gamepad1.right_stick_y);

        // this will send a telemetry message to signify robot waiting
        telemetry.addLine("I 'm Ready");
        telemetry.update();

    }



      //Code will run ONCE after the driver hits STOP
        @Override
        public void stop() {
            // Sets all motors to zero power except Arms to keep pos
            frontLeftDrive.setPower(0);
            rearLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            rearRightDrive.setPower(0);
        }
        // pixelArm.setTargetPosition(pixelArm.getCurrentPosition());

    }
