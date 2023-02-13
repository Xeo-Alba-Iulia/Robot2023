package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utilities.PIDController;

@TeleOp(name = "TeleOP", group = "A")
public class Prototype extends OpMode {


    RobotHardware robot = new RobotHardware(this);
    PIDController ridicareController = new PIDController(0.001, 0.0002, 0.02975039, this);
    double ridicareTarget = 1;
    double target = 0;

    boolean aPressedLastIteratoion;
    boolean clawIn;

    @Override
    public void init() {
        robot.init();
        aPressedLastIteratoion = false;
        target = 0;
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
        robot.lift.setPower(ridicareController.update(target, robot.lift.getCurrentPosition()));
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
        if(gamepad2.a) {
            robot.claw.setPower(0.2);
        } else if(gamepad2.b) {
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
