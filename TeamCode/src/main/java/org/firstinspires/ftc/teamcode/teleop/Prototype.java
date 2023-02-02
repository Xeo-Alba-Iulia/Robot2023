package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utilities.PIDController;

@TeleOp(name = "prototype", group = "B")
public class Prototype extends OpMode {

    static final int RIDICARE_POS_1 = 5500;
    static final int RIDICARE_POS_2 = 10600;
    static final int RIDICARE_POS_3 = 17800;

    RobotHardware robot = new RobotHardware(this);
    PIDController ridicareController = new PIDController(0.0108, 0.00048129, 0.06058665, this);
    double ridicareTarget = 1;
    double target = 0;

    boolean aPressedLastIteratoion;
    boolean clawIn;

    @Override
    public void init() {
        robot.init();
        aPressedLastIteratoion = false;
        clawIn = false;
        target = 0;
    }

    private void ridicare() {
        if (gamepad2.dpad_down) {
            target = 1000;
        } else if (gamepad2.dpad_left) {
            target = RIDICARE_POS_1;
        } else if (gamepad2.dpad_up) {
            target = RIDICARE_POS_2;
        } else if (gamepad2.dpad_right) {
            target = RIDICARE_POS_3;
        }
        robot.ridicare.setPower(ridicareController.update(target, -robot.ridicare.getCurrentPosition()));
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
        if (gamepad1.a && !aPressedLastIteratoion) {
            clawIn = !clawIn;
            telemetry.addData("BANG", "BANG");
        }
        aPressedLastIteratoion = gamepad1.a;
        if (!clawIn) {
            robot.claw.setPower(0.4);
        } else {
            robot.claw.setPower(-0.1);
        }

    }


    @Override
    public void start() {
        robot.start();
    }


    private void telemetry() {
        telemetry.update();
    }


    public void loop() {
        robot.movement(gamepad1);
        ridicare();
        virtualFourBar();
        claw();


        telemetry();
        telemetry.update();
    }

}
