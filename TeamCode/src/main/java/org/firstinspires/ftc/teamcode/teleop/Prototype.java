package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utilities.PIDController;

@TeleOp(name = "prototype", group = "B")
public class Prototype extends OpMode {
    static final int POS_1 = 14278;
    static final int POS_2 = 30000;
    static final int POS_3 = 55000;
    RobotHardware robot = new RobotHardware(this);
    PIDController ridicareController = new PIDController(3, 1, 2, this);
    PIDController vFBController = new PIDController(0, 0, 0, this);
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

        robot.ridicare.setPower(ridicareController.update(target, robot.ridicare.getCurrentPosition()));
    }

    private void virtualFourBar() {

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
        telemetry.addData("Vf power", robot.virtualFourBar.getPower());

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