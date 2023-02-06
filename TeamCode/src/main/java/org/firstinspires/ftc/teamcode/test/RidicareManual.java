package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "ridicare manual", group = "test")
public class RidicareManual extends OpMode {
    private RobotHardware robot = new RobotHardware(this);
    @Override
    public void init() {
        robot.init();
    }

    @Override
    public void loop() {
        robot.ridicare1.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        telemetry.addData("pozitie",robot.ridicare1.getCurrentPosition());
        telemetry.addData("pozitie ridicare", robot.ridicare1.getCurrentPosition());
        telemetry.addData("target", robot.ridicare1.getTargetPosition());
        telemetry.update();
    }
}
