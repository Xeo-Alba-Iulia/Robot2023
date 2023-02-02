package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utilities.PIDController;

@TeleOp(name = "prototype", group = "B")
public class Prototype extends OpMode {
//<<<<<<< HEAD
	static final int RIDICARE_POS_1 = 5100;
	static final int RIDICARE_POS_2 = 9800;
	static final int RIDICARE_POS_3 = 16500;
	static final double VFB_OUTTAKE_POS = 0.3;
	static final double VFB_INTAKE_POS = 0.05;

	static final double VFB_POS3=0.8;

	static final double VFB_POS4=0.65;

	RobotHardware robot = new RobotHardware(this);
	PIDController ridicareController = new PIDController(0.012, 0.25, 0.000144, this);
	double ridicareTarget = 0;

//=======
    //static final int RIDICARE_POS_1 = 14278;
    //static final int RIDICARE_POS_2 = 30000;
    //static final int RIDICARE_POS_3 = 55000;
  //  static final double VFB_OUTTAKE_POS = 0.3;
   // static final double VFB_INTAKE_POS = 0;

  //  RobotHardware robot = new RobotHardware(this);
    //PIDController ridicareController = new PIDController(0.004, 0.008, 1, this);
    //double ridicareTarget = 1;


    @Override
    public void init() {
        robot.init();
    }

    public void ridicare() {
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

	private void vFBpos3() {
		robot.vFB1.setPosition(VFB_POS3);
		robot.vFB2.setPosition(1 - VFB_POS3);
	}

	private void vFBpos4() {
		robot.vFB1.setPosition(VFB_POS4);
		robot.vFB2.setPosition(1 - VFB_POS4);
	}

    private void virtualFourBar() {
        if (gamepad2.left_bumper) {
            vFBIntake();
        } else if (gamepad2.right_bumper) {
            vFBOuttake();
        } else if (gamepad2.left_trigger>0) {
	        vFBpos3();
        } else if (gamepad2.right_trigger>0) {
	        vFBpos4();
        }
        telemetry.addLine("Virtual Four Bar");
        telemetry.addData("poz VFb1", robot.vFB1.getPosition());
        telemetry.addData("poz VFb2", robot.vFB1.getPosition());
		telemetry.addData("power motor",robot.ridicare.getPower());
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

//<<<<<<< HEAD
	public void loop() {
		robot.movement(gamepad1);
		if (gamepad2.dpad_down) {
			ridicareTarget = 0;
		} else if (gamepad2.dpad_left) {
			ridicareTarget = RIDICARE_POS_1;
		} else if (gamepad2.dpad_up) {
			ridicareTarget = RIDICARE_POS_2;
		} else if (gamepad2.dpad_right) {
			ridicareTarget = RIDICARE_POS_3;
		}
		virtualFourBar();
		if (gamepad2.a) {
			robot.claw.setPower(0.4);
		} else if (gamepad2.b) {
			robot.claw.setPower(-0.4);
		} else robot.claw.setPower(0);



        telemetry();
    }


}