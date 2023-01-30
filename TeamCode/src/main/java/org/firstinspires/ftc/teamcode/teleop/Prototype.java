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
    static final double VFB_OUTTAKE_POS = 0.3;
    static final double VFB_INTAKE_POS = 0;

    RobotHardware robot = new RobotHardware(this);
    PIDController ridicareController = new PIDController(0.004, 0.008, 1, this);
    double ridicareTarget = 1;

    @Override
    public void init() {
        robot.init();
    }

    private void ridicare() {
        if (gamepad2.dpad_down) {
            ridicareTarget = 0;
        } else if (gamepad2.dpad_left) {
            ridicareTarget = RIDICARE_POS_1;
        } else if (gamepad2.dpad_up) {
            ridicareTarget = RIDICARE_POS_2;
        } else if (gamepad2.dpad_right) {
            ridicareTarget = RIDICARE_POS_3;
        }

    }


    private void vFBIntake() {
        robot.vFB1.setPosition(VFB_INTAKE_POS);
        robot.vFB2.setPosition(1 - VFB_INTAKE_POS);
    }

    private void vFBOuttake() {
        robot.vFB1.setPosition(VFB_OUTTAKE_POS);
        robot.vFB2.setPosition(1 - VFB_OUTTAKE_POS);
    }

    private void virtualFourBar() {
        if (gamepad2.left_bumper) {
            vFBIntake();
        } else if (gamepad2.right_bumper) {
            vFBOuttake();
        }
        telemetry.addLine("Virtual Four Bar");
        telemetry.addData("poz VFb1", robot.vFB1.getPosition());
        telemetry.addData("poz VFb2", robot.vFB1.getPosition());

    }

    private void multitask() {
        if (gamepad2.y) {
            vFBOuttake();
            ridicareTarget = RIDICARE_POS_3;
        } else if (gamepad2.x) {
            vFBIntake();
            ridicareTarget = ridicareTarget = 0;
        }

    }

    private void telemetry() {
        telemetry.update();
    }

    public void loop() {
        robot.movement(gamepad1);
//		ridicare();
        virtualFourBar();
        if (gamepad2.a) {
            robot.claw.setPower(0.4);
        } else if (gamepad2.b) {
            robot.claw.setPower(-0.4);
        } else robot.claw.setPower(0);
        robot.ridicare.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

        telemetry();
    }


}