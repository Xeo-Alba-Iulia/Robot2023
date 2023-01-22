package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utilities.PIDController;

@TeleOp(name = "prototype", group = "B")
public class Prototype extends OpMode {
    static final double POS_1 = 13000;
    static final double POS_2 = 38000;
    static final double POS_3 = 65500;

    static final int vfbPOS=5;
    RobotHardware robot = new RobotHardware(this);
    PIDController ridicareController = new PIDController(3, 0, 4, this);
//    PIDController vFBController = new PIDController(0, 0, 0, this);
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

        robot.ridicare.setPower(
                ridicareController.update
                        (target, robot.ridicare.getCurrentPosition()));
    }

 //   private void virtualFourBar() {
 //       if (gamepad2.right_bumper) {
   //         target = vfbPOS;
     //   } else if (gamepad2.left_bumper) {
       //     target = 0;
        //}

        //robot.ridicare.setPower(vFBController.update(target, robot.ridicare.getCurrentPosition()));
   // }


    private void virtualFourBar() {
        robot.virtualFourBar.setPower(gamepad2.right_stick_x*0.7);
    }
    private void telemetry() {

        telemetry.addLine("Ridicare");
        telemetry.addLine();
        telemetry.addData("Poz Ridicare",robot.ridicare.getCurrentPosition());
        telemetry.addData("Ridicare Target", target);
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
//        ridicare();
        virtualFourBar();
        if (gamepad2.a) {
            robot.claw.setPower(0.4);
        } else if (gamepad2.b) {
            robot.claw.setPower(-0.4);
        } else robot.claw.setPower(0);
        robot.ridicare.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        telemetry();
    }


}