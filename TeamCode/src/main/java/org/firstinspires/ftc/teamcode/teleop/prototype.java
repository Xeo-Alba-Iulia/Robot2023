package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "prototype")
public class prototype extends OpMode {

    RobotHardware robot = new RobotHardware(this);
    PIDController pid = new PIDController(1, 0 ,0, this);
    private double targetPos;

    @Override
    public void init() {
        robot.init();
        targetPos = 0;
        telemetry.addData("ridicare", robot.ridicare.getCurrentPosition());
    }

    @Override
    public void loop() {
//        robot.movement(gamepad1);
//
//        if(gamepad1.a)
//            robot.claw.setPower(0.4);
//        else if(gamepad1.b)
//            robot.claw.setPower((-0.4));
//        else
//            robot.claw.setPower(0);

        if(gamepad1.right_bumper)
            robot.virtualFourbar.setPower(0.4);
        else if(gamepad1.left_bumper)
            robot.virtualFourbar.setPower(-0.4);
        else
            robot.virtualFourbar.setPower(0);

       robot.setRidicarePos(gamepad1);

        if(robot.ridicare.getCurrentPosition() < targetPos - 200)
            robot.ridicare.setPower(pid.update(robot.targetPos, robot.ridicare.getCurrentPosition()));
        else
            robot.timer.reset();
        telemetry.update();
    }

}
