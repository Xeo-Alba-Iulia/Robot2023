package org.firstinspires.ftc.teamcode.opmodes.teleop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "TeleOP", group = "A")
public class TeleOP extends OpMode {

    boolean stackToggle;

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void init() {
        stackToggle = false;
        robot.init();
        robot.claw.setPosition(robot.GHEARA_DESCHISA);
        robot.vFB1.setPosition(0);
        robot.vFB2.setPosition(1);
    }

    @Override
    public void start() {
        robot.start();
    }


    private void intakeOuttake() {
        if (gamepad2.dpad_down) {
            robot.lift.target = 600;
            robot.setVFBPosition(robot.VFB_ALIGN_POSE);
        } else if (gamepad2.dpad_left) {
            robot.lift.target = robot.RIDICARE_POS_1;
            robot.setVFBPosition(robot.VFB_OUTTAKE_POSE);
        } else if (gamepad2.dpad_up) {
            robot.lift.target = robot.RIDICARE_POS_2;
            robot.setVFBPosition(robot.VFB_OUTTAKE_POSE);
        } else if (gamepad2.dpad_right) {
            robot.lift.target = robot.RIDICARE_POS_3;
            robot.setVFBPosition(robot.VFB_ALIGN_HIGH_POSE);
        }

        robot.lift.update();


        if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0) {
            robot.lift.update();
        } else {
            robot.lift.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            robot.lift.target = robot.lift.getCurrentPosition();
        }
    }

    private void virtualFourBar() {
        if (gamepad2.left_bumper) {
            robot.setVFBPosition(robot.VFB_INTAKE_POSE);
        } else if (gamepad2.right_stick_button) {
            robot.setVFBPosition(robot.VFB_OUTTAKE_POSE);
        } else if (gamepad2.right_bumper) {
            robot.setVFBPosition(robot.VFB_ALIGN_POSE);
        }
    }

    private void claw() {
        if (gamepad2.a) {
            robot.claw.setPosition(robot.GHEARA_INCHISA);
        } else if (gamepad2.b) {
            robot.claw.setPosition(robot.GHEARA_DESCHISA);
        }
    }

    private void stack() {

        if (gamepad1.a && !stackToggle) {
            stackToggle = true;
            robot.vfb_stack_pose -= 0.07;
            robot.setVFBPosition(robot.vfb_stack_pose);
        } else if (gamepad1.y) {
            stackToggle = true;;
            robot.vfb_stack_pose += 0.07;
            robot.setVFBPosition(robot.vfb_stack_pose);
        } else if ((!gamepad1.a || gamepad1.b) && stackToggle) {
            stackToggle = false;
        }
    }

    private void telemetry() {
        telemetry.addLine("Ridicare");
        telemetry.addData("Pozitie", robot.lift.getCurrentPosition());
        telemetry.addLine("Poz Vfb");
        telemetry.addData("Stack", robot.vfb_stack_pose);
        telemetry.update();
    }


    public void loop() {
        robot.movement(gamepad1);
        intakeOuttake();
        virtualFourBar();
        claw();
//        stack();

        telemetry();
        telemetry.update();
    }

}
