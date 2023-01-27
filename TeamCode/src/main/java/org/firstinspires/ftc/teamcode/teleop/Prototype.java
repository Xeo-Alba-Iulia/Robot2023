package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utilities.PIDController;

@TeleOp(name = "prototype", group = "B")
public class Prototype extends OpMode {
    static final int RIDICARE_POS_1 = 14278;
    static final int RIDICARE_POS_2 = 30000;
    static final int RIDICARE_POS_3 = 55000;
    static final int VFB_OUTAKE_POS = 1500;
    static final int POS_1 = 14278;
    static final int POS_2 = 30000;
    static final int POS_3 = 55000;

    RobotHardware robot = new RobotHardware(this);
  //  PIDController ridicareController = new PIDController(3, 0, 4, this);
    PIDController vFBController = new PIDController(1, 0, 0, this);
    double target = 0;

    @Override
    public void init() {
        robot.init();
    }

    private void ridicare() {
        if (gamepad2.dpad_down) {
            target = 0;
        } else if (gamepad2.dpad_left) {
            target = POS_1;
        } else if (gamepad2.dpad_up) {
            target = POS_2;
        } else if (gamepad2.dpad_right) {
            target = POS_3;
        }

   //     robot.ridicare.setPower(ridicareController.update(target, robot.ridicare.getCurrentPosition()));
    }


    private void virtualFourBar() {
        if(gamepad2.x) {
            robot.vFB.setPosition(1);
        } else if(gamepad2.y) {
            robot.vFB.setPosition(0);
        } else if(gamepad2.left_bumper) {
            robot.vFB.setPosition(0.25);
        } else if(gamepad2.right_bumper) {
            robot.vFB.setPosition(0.75);
        } else {
            robot.vFB.setPosition(0.5);
        }
        telemetry.addData("poz VFb", robot.vFB.getPosition());

    }
    private void telemetry() {
        telemetry.update();
    }

    public void loop() {
        robot.movement(gamepad1);
       // ridicare();
        virtualFourBar();
        if (gamepad2.a) {
            robot.claw.setPower(0.4);
        } else if (gamepad2.b) {
            robot.claw.setPower(-0.4);
        } else robot.claw.setPower(0);
        robot.ridicare.setPower(gamepad2.right_trigger-gamepad2.left_trigger);

        telemetry();
    }


}