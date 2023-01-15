package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utilities.PIDController;

@TeleOp(name = "prototype", group = "B")
public class prototype extends OpMode {
    static final int POS_1 = 14278;
    static final int POS_2 = 42000;
    static final int POS_3 = 78000;
    RobotHardware robot = new RobotHardware(this);
    PIDController pid = new PIDController(3, 1, 2, this);
    int target = 0;

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
        boolean liftIsHome = target == 0 &&
                -robot.ridicare.getCurrentPosition() < 1000 &&
                -robot.ridicare.getCurrentPosition() > -1000;
        if (liftIsHome) {
            robot.ridicare.setPower(0);
        } else if (gamepad2.left_stick_y != 0) {
            robot.ridicare.setPower(-gamepad1.left_stick_y);
            target = robot.ridicare.getCurrentPosition();
        } else
            robot.ridicare.setPower(pid.update(target, robot.ridicare.getCurrentPosition()));
    }

    private void virtualFourBar() {
        robot.vfb1.setPower(gamepad2.left_stick_x);
        robot.vfb2.setPower(gamepad2.left_stick_x);
    }

    private void telemetry() {
        switch (target) {
            case POS_1:
                telemetry.addData("Target Ridicare", "Low Junction");
                break;
            case POS_2:
                telemetry.addData("Target Ridicare", "Medium Junction");
                break;
            case POS_3:
                telemetry.addData("Target Ridicare", "High Junction");
                break;
            default:
                telemetry.addData("Target Ridicare", "Ground Junction");
        }


        telemetry.addData("Ridicare Target", target);
        telemetry.addData("Gheara Puternica", robot.claw.getPower());
        telemetry.update();
    }

    public void loop() {
        robot.movement(gamepad1);
        ridicare();
        virtualFourBar();
        if (gamepad2.a) {
            robot.claw.setPower(0.4);
        } else if (gamepad2.b) {
            robot.claw.setPower(-0.4);
        } else robot.claw.setPower(0);


        telemetry();


    }


}
