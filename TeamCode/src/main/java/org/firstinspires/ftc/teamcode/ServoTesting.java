package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class ServoTesting extends OpMode {
    ServoImplEx flapper;
    //clawRot;

    @Override
    public void init() {
        flapper= hardwareMap.get(ServoImplEx.class, "flapper");
       // clawRot = hardwareMap.get(ServoImplEx.class, "clawPivot");
    }

    @Override
    public void loop() {
        if (gamepad1.a) flapper.setPosition(0);
        if (gamepad1.b) flapper.setPosition(0.25);
        if (gamepad1.left_bumper) flapper.getController().pwmDisable();
        if (gamepad1.left_trigger > 0.75) flapper.getController().pwmEnable();

        /*if (gamepad1.dpad_down) clawRot.setPosition(0.5);
        if (gamepad1.dpad_right) clawRot.setPosition(1);
        if (gamepad1.dpad_left) clawRot.setPosition(0);
        if (gamepad1.right_bumper) clawRot.getController().pwmDisable();
        if (gamepad1.right_trigger > 0.75) clawRot.getController().pwmEnable();*/
        telemetry.addData("claw cont", flapper.getController());
        telemetry.addData("claw rot cont", flapper.getController());
        telemetry.addData("Claw pwm", flapper.isPwmEnabled());
        telemetry.addData("Claw Rot pwm", flapper.isPwmEnabled());
        telemetry.update();
    }
}
