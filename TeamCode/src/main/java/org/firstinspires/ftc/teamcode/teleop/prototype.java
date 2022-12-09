package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "prototype")
public class prototype extends OpMode {

    RobotHardware robot = new RobotHardware(this);
    PIDController pid = new PIDController(0, 0 ,0, this);
    private double targetPos;

    @Override
    public void init() {
        robot.init();
        targetPos = 0;
    }

    @Override
    public void loop() {
        robot.movement(gamepad1);
        if(gamepad1.dpad_down) {
            targetPos = 0;
        } else if(gamepad1.dpad_left) {
            targetPos = robot.ridicarePos1;
        } else if(gamepad1.dpad_up) {
            targetPos = robot.ridicarePos2;
        } else if(gamepad1.dpad_right) {
            targetPos = robot.ridicarePos3;
        }
        if(gamepad1.right_bumper)
            robot.virtualFourbar.setPower(1);
        else if(gamepad1.left_bumper)
            robot.virtualFourbar.setPower(-1);
        else
            robot.virtualFourbar.setPower(0);

        if(robot.ridicare.getCurrentPosition() < targetPos - 200)
            robot.ridicare.setPower(pid.update(targetPos, robot.ridicare.getCurrentPosition()));
        else
            robot.timer.reset();
    }

}
