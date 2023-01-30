package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name = "servotest")
public class Servo extends OpMode {
    private ServoImplEx servo1;
    private ServoImplEx servo2;
    private final PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(500, 2500);
    ElapsedTime timer = new ElapsedTime();
    private int pos = 0;


    @Override
    public void init() {
        servo1 =hardwareMap.get(ServoImplEx.class,"vfb1");
        servo2 =hardwareMap.get(ServoImplEx.class,"vfb2");
        servo1.setPwmRange(new PwmControl.PwmRange(500,2500));
        servo2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        servo1.setPosition(1);
        servo2.setPosition(0);
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        if(timer.seconds() > 5 && timer.seconds() < 10) {
            servo1.setPosition(0.6);
            servo2.setPosition(1-0.6);
        } else if (timer.seconds() >= 10) {
            servo1.setPosition(0.3);
            servo2.setPosition(1-0.3);
        }

        telemetry.addData("poz1", servo1.getPosition());
        telemetry.addData("poz2", servo2.getPosition());

    }
}
