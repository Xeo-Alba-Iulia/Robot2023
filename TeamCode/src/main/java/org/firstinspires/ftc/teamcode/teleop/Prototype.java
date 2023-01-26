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

    RobotHardware robot = new RobotHardware(this);
    PIDController ridicareController = new PIDController(3, 0, 4, this);
    PIDController vFBController = new PIDController(0.05, 0, 0.4, this);
    double ridicareTarget = 0;
    double vFBTarget = 0;

    @Override
    public void init() {
        robot.init();
    }

    private void outtake() {
        if (gamepad2.dpad_down) {
            ridicareTarget = 0;
            vFBTarget = 0;
        } else if (gamepad2.dpad_left) {
            ridicareTarget = RIDICARE_POS_1;
            vFBTarget = VFB_OUTAKE_POS;
        } else if (gamepad2.dpad_up) {
            ridicareTarget = RIDICARE_POS_2;
            vFBTarget = VFB_OUTAKE_POS;
        } else if (gamepad2.dpad_right) {
            ridicareTarget = RIDICARE_POS_3;
            vFBTarget = VFB_OUTAKE_POS;
        }
        double ridicarePower = ridicareController.update(ridicareTarget, -robot.ridicare.getCurrentPosition());
        double vFBPower = vFBController.update(vFBTarget, robot.virtualFourBar.getCurrentPosition());
        robot.ridicare.setPower(ridicarePower);
        robot.virtualFourBar.setPower(vFBPower);
    }



    private void telemetry() {

        telemetry.addLine("Ridicare");
        telemetry.addLine();
        telemetry.addData("Poz Ridicare",robot.ridicare.getCurrentPosition());
        telemetry.addData("Ridicare Target", ridicareTarget);
        telemetry.addData("Putere Ridicare", robot.ridicare.getPower());

        telemetry.addLine("Virtual Four Bar");
        telemetry.addLine();
        telemetry.addData("Vf power", robot.virtualFourBar.getPower());
        telemetry.addData("Vf pos", robot.virtualFourBar.getCurrentPosition());
        telemetry.addLine("Gheara");
        telemetry.addLine();
        telemetry.addData("Gheara Puternica", robot.claw.getPower());


        telemetry.update();
    }

    public void loop() {
        robot.movement(gamepad1);
        outtake();
    //        virtualFourBar();
        if (gamepad2.a) {
            robot.claw.setPower(0.4);
        } else if (gamepad2.b) {
            robot.claw.setPower(-0.4);
        } else robot.claw.setPower(0);


        telemetry();
    }


}