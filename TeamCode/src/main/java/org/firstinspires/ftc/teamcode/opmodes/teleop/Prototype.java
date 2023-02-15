package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "TeleOP", group = "A")
public class Prototype extends OpMode {


    RobotHardware robot = new RobotHardware(this);
    double target = 0;
    boolean autoLift;
    boolean aPressedLastIteratoion;

    @Override
    public void init() {
        robot.init();
        robot.vFB1.setPosition(0);
        robot.vFB2.setPosition(1);
    }
    @Override
    public void start() {
        robot.start();
    }


    private void intakeOuttake() {
        if (gamepad2.dpad_down) {
            target = 600;
            robot.setVFBPosition(robot.VFB_ALIGN_POS);
        } else if (gamepad2.dpad_left) {
            target = robot.RIDICARE_POS_1;
            robot.setVFBPosition(robot.VFB_OUTTAKE_POS);
        } else if (gamepad2.dpad_up) {
            target = robot.RIDICARE_POS_2;
            robot.setVFBPosition(robot.VFB_OUTTAKE_POS);
        } else if (gamepad2.dpad_right) {
            target = robot.RIDICARE_POS_3;
            robot.setVFBPosition(robot.VFB_ALIGN_HIGH_POS);
        }




        if(gamepad1.left_trigger==0 && gamepad1.right_trigger==0) {
            robot.lift.update();
        } else {
            robot.lift.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            robot.lift.target=robot.lift.getCurrentPosition();
        }
    }

    private void virtualFourBar() {
        if (gamepad2.left_bumper) {
            robot.setVFBPosition(robot.VFB_INTAKE_POS);
        } else if (gamepad2.right_stick_button) {
            robot.setVFBPosition(robot.VFB_OUTTAKE_POS);
        } else if (gamepad2.right_bumper) {
            robot.setVFBPosition(robot.VFB_ALIGN_POS);
        }
    }

    private void claw() {
        if(gamepad2.b) {
            robot.claw.setPower(0.2);
        } else if(gamepad2.a) {
            robot.claw.setPower(-0.2);
        } else {
            robot.claw.setPower(0);
        }

    }

    private void telemetry() {
        telemetry.addLine("Ridicare");
        telemetry.addData("Pozitie", robot.lift.getCurrentPosition());
        telemetry.update();
    }


    public void loop() {
        robot.movement(gamepad1);
        intakeOuttake();
        virtualFourBar();
        claw();


        telemetry();
        telemetry.update();
    }

}
