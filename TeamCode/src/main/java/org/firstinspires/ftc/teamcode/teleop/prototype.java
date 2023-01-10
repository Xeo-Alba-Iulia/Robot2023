package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "prototype")
public class prototype extends OpMode {
    RobotHardware robot = new RobotHardware(this);


    @Override
    public void init() {
        robot.init();
    }

    @Override
    public void loop() {
       robot.ridicare.setPower(-gamepad2.left_stick_y);
       telemetry.addData("Putere Ridicare", robot.ridicare.getPower());
       telemetry.update();
    }

}
