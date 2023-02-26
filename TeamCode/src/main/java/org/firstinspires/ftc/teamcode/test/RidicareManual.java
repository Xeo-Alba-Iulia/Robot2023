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
       //robot.setVFBPosition(robot.VFB_ALIGN_HIGH_POSE);
    }

    @Override
    public void loop() {
        robot.lift.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        telemetry.addData("pozitie",robot.lift.getCurrentPosition());
        telemetry.addData("pozitie ridicare", robot.lift.getCurrentPosition());
        telemetry.update();
    }
}
