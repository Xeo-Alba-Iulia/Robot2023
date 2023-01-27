package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "servotest")
public class Test extends OpMode {
    private ServoImplEx hei;
    private final PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(500, 2500);


    @Override
    public void init() {
        hei=hardwareMap.get(ServoImplEx.class,"boss");
        hei.setPwmRange(new PwmControl.PwmRange(500,2500));
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            hei.setPosition(1);
        } else if (gamepad1.b) {
            hei.setPosition(-1);
        }
        else hei.setPosition(0);
        telemetry.addData("pozitie", hei.getPosition());
        telemetry.update();

    }
}
