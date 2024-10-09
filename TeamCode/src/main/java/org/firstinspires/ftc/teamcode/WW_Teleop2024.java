package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="WW_Teleop2024", group="WiredWoodmen")  //declares the name of the class and the
// group it is in.



public class WW_Teleop2024 extends OpMode{


    double  //declares all double variables and their values
            speedVariable = .8;

    //float[] hsvValuesRight = new float[3];
    //float[] hsvValuesLeft = new float[3];




    /*
     * Code will run ONCE when the driver hits INIT
     * INIT means initialize
     */
    DcMotorEx
            rearLeftDrive= null,
            rearRightDrive = null,
            frontLeftDrive= null,
            frontRightDrive = null,
            viperslide=null;

    // pixelArm = null;
    double hangerpos=0.0;
    HardwareMap hwMap =  null;
    ElapsedTime runTime = new ElapsedTime();


    @Override
    public void init() { //initialization class to be used at start of tele-op

        // these are our motors and what they are called
        rearLeftDrive=hardwareMap.get(DcMotorEx.class, "rear_left_drive");

        rearRightDrive = hardwareMap.get(DcMotorEx.class, "rear_right_drive");
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        viperslide=hardwareMap.get(DcMotorEx.class,"viperslide");

//Direction?
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);
        viperslide.setDirection(DcMotorSimple.Direction.REVERSE);
        // pixelArm.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);
        viperslide.setPower(0);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Setting motors to run without encoders
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        viperslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        rearLeftDrive.setPower(Range.clip((-BLspeed*speedVariable),-1,1));
        rearRightDrive.setPower(Range.clip((BRspeed*speedVariable),-1,1));
        frontLeftDrive.setPower(Range.clip((FLspeed*speedVariable),-1,1));
        frontRightDrive.setPower(Range.clip((-FRspeed*speedVariable),-1,1));


        //DriveTrain Speed Controls
        if (gamepad1.dpad_left) speedVariable -= 0.05;
        if (gamepad1.dpad_right) speedVariable += 0.05;
        speedVariable= Range.clip(speedVariable,0,1);
        if(gamepad1.right_trigger > 0.5 && hangerpos < 2000)
            hangerpos +=50;
        if(gamepad1.left_trigger>0.5 && hangerpos > 40)
            hangerpos -= 40;
        viperslide.setTargetPosition((int)hangerpos);
        viperslide.setPower(0.7);
        viperslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //==========================================================
        //                        GamePad Two
        //==========================================================

        //D-Pad stuff. none right noe


        //==========================================================//
        //                        Telemetry                           //
        //==========================================================/
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


    /*
     * Code will run ONCE after the driver hits STOP
     */
    @Override
    public void stop(){
        // Sets all motors to zero power except Arms to keep pos
        frontLeftDrive.setPower(0);
        rearLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearRightDrive.setPower(0);
        // pixelArm.setTargetPosition(pixelArm.getCurrentPosition());
    }
}