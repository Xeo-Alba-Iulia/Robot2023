package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "prototype", group = "B")
public class prototype extends OpMode {
    RobotHardware robot = new RobotHardware(this);
    FtcDashboard dashboard;

    @Override
    public void init() {
        robot.init();
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        robot.setRidicarePos(gamepad1);
        telemetry.update();
        robot.ridicare.setPower(gamepad2.right_stick_x);

    }

}
