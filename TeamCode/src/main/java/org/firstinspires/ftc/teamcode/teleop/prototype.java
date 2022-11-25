package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class prototype extends OpMode {

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void init() {
        robot.init();
    }

    @Override
    public void loop() {
        robot.movement(gamepad1);
        if (gamepad1.a)
            robot.claw.setPower(1);
        else if (gamepad1.b)
            robot.claw.setPower(-1);
        else
            robot.claw.setPower(0);
        robot.setFourbarPower(gamepad1.right_trigger- gamepad1.left_trigger);
    }
}
