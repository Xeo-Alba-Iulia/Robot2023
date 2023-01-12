package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utilities.PIDController;

@TeleOp(name = "prototype", group = "B")
public class prototype extends OpMode {
    RobotHardware robot = new RobotHardware(this);
    FtcDashboard dashboard;
    PIDController pid = new PIDController(3, 1, 2, this);
    int target = 0;
    static final int POS_1 = 14278;
    static final int POS_2 = 45000;
    static final int POS_3 = 69420;


    @Override
    public void init() {
        robot.init();
        dashboard = FtcDashboard.getInstance();
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
                -robot.ridicare.getCurrentPosition() < 100 &&
                -robot.ridicare.getCurrentPosition() > -100;
        if (liftIsHome)
            robot.ridicare.setPower(0);
        else
            robot.ridicare.setPower(pid.update(target, robot.ridicare.getCurrentPosition()));
    }

    private void virtualFourBar() {
        if (gamepad2.x) {
            robot.vfb1.setPower(1);
            robot.vfb2.setPower(1);
        } else if (gamepad2.y) {
            robot.vfb1.setPower(-1);
            robot.vfb2.setPower(-1);
        } else if (!robot.touch.getState()) {
            robot.vfb1.setPower(0);
            robot.vfb2.setPower(0);
        }

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
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.movement(gamepad1);
        ridicare();
        virtualFourBar();
        if (gamepad2.a) {
            robot.claw.setPower(0.2);
        } else if (gamepad2.b) {
            robot.claw.setPower(-0.2);
        } else robot.claw.setPower(0);

        telemetry();


    }

}
