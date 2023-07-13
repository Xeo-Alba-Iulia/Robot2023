package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.sisteme.Ridicare;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utilities.PoseStorage;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "TeleOP", group = "A")
public class TeleOP extends OpMode {

    public LiftState liftState = LiftState.LIFT_START;
    boolean stackToggle;
    boolean intakeToggle;
    boolean clawToggle;
    boolean currentA;
    boolean currentX;
    boolean currentDdown;
    RobotHardware robot = new RobotHardware(this);
    StandardTrackingWheelLocalizer myLocalizer;
    SampleMecanumDrive drive;
    TrajectorySequence junction1;
    ElapsedTime liftTimer = new ElapsedTime();

    @Override
    public void init() {
        stackToggle = false;
        clawToggle = false;
        intakeToggle = false;
        currentA = false;
        currentX = false;
        currentDdown = false;
        robot.init();
        robot.virtualFourBar.setPosition(robot.VFB_INTAKE);
        robot.claw.setPosition(robot.GHEARA_INCHISA);
        robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_INTAKE);
        drive = new SampleMecanumDrive(hardwareMap);
        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();
        myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);
        myLocalizer.setPoseEstimate(PoseStorage.currentPose);

    }


    private void intakeOuttake(){
        if(gamepad1.dpad_down) {
           robot.lift.reference = 0;
           robot.virtualFourBar.setPosition(robot.VFB_INTAKE);
           robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_INTAKE);
           robot.claw.setPosition(robot.GHEARA_INCHISA);
        } else if(gamepad1.dpad_left) {
            robot.lift.reference = Ridicare.POS_1;
            robot.virtualFourBar.setPosition(robot.VFB_LOW);
            robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_LOW);
            robot.claw.setPosition(robot.GHEARA_INCHISA);
        } else if(gamepad1.dpad_up) {
            robot.lift.reference = Ridicare.POS_2;
            robot.virtualFourBar.setPosition(robot.VFB_MEDIUM);
            robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_MEDIUM);
            robot.claw.setPosition(robot.GHEARA_INCHISA);
        } else if(gamepad1.dpad_right) {
            robot.lift.reference =Ridicare.POS_3;
            robot.virtualFourBar.setPosition(robot.VFB_HIGH);
            robot.claw.setPosition(robot.GHEARA_INCHISA);
            robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_HIGH);

        }
        robot.lift.update();

    }
    private void virtualFourBar() {
        if (gamepad1.right_bumper) {
            robot.virtualFourBar.setPosition(robot.VFB_INTAKE);

        } else if (gamepad1.left_bumper) {
            robot.virtualFourBar.setPosition(robot.VFB_INTER);

        }
    }

    private void claw() {
        if (gamepad1.a != currentA) {
            currentA = gamepad1.a;
            if (gamepad1.a) {
                clawToggle = !clawToggle;
                if (clawToggle) {
                    robot.claw.setPosition(robot.GHEARA_INCHISA);
                } else {
                    robot.claw.setPosition(robot.GHEARA_DESCHISA);
                }
            }
        }
    }

    private void telemetry() {
//        telemetry.addData("Lift State", liftState);

//        telemetry.addData("Pozitie", robot.lift.getCurrentPosition());
//        telemetry.addData("Stack", robot.vfb_stack_pose);
        telemetry.addData("pozitie gheara", robot.claw.getPosition());
        telemetry.addData("pozitie vfb", robot.virtualFourBar.getPosition());
        telemetry.update();
    }

    public void loop() {
        myLocalizer.update();
        drive.update();
        robot.movement(gamepad1);
        intakeOuttake();
        virtualFourBar();
        claw();
//        allignJunction();
        //allignclaw();
//       cazut();
//        stackModifier();


        telemetry();
        telemetry.update();
    }

    public enum LiftState {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_EXTEND2,
        LIFT_DUMP,
        LIFT_RETRACT
    }

}








