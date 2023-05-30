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
        private double state_claw_align=robot.CLAW_ALLIGN_POS_INTAKE;
    private static int state_lift_pos=600;
    private double state_vfb_pos=robot.VFB_ALIGN_POSE;
    private double increasePosVFB;
    private double increasePosAlign;

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
//        robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_INTAKE);
        drive = new SampleMecanumDrive(hardwareMap);
        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();
        myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);
        myLocalizer.setPoseEstimate(PoseStorage.currentPose);

    }






    private void intakeOuttake() {

        if (gamepad2.dpad_left) {
            state_lift_pos = Ridicare.POS_1;
            state_claw_align = robot.CLAW_ALLIGN_POS_LOW;
            state_vfb_pos = robot.VFB_LOW;
        } else if (gamepad2.dpad_up) {
            state_lift_pos = Ridicare.POS_2;
            state_claw_align = robot.CLAW_ALLIGN_POS_MEDIUM;
            state_vfb_pos = robot.VFB_MEDIUM;
        } else if (gamepad2.dpad_right) {
            state_lift_pos = Ridicare.POS_3;
            state_claw_align = robot.CLAW_ALLIGN_POS_HIGH;
            state_vfb_pos = robot.VFB_HIGH;
        }

        switch (liftState) {
            case LIFT_START:
                if (state_lift_pos != 600) {
                    robot.lift.target = state_lift_pos;
                    robot.virtualFourBar.setPosition(state_vfb_pos);
                    robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_INTAKE);
                    robot.claw.setPosition(robot.GHEARA_INCHISA);
                    liftState = LiftState.LIFT_EXTEND;
                }
                virtualFourBar();
                break;

            case LIFT_EXTEND:
                if (Math.abs(-robot.lift.getCurrentPosition() - robot.lift.target) < 800) {
                    liftTimer.reset();
                    liftState = LiftState.LIFT_EXTEND2;
                }
                break;
            case LIFT_EXTEND2:
                if (liftTimer.milliseconds() > 200) {
                    robot.claw_alligner.setPosition(state_claw_align);
                    liftState = LiftState.LIFT_DUMP;
                }
                break;


            case LIFT_DUMP:
                if (gamepad2.dpad_down) {
                    robot.lift.target = 600;
                    state_lift_pos = robot.lift.target;
                    robot.virtualFourBar.setPosition(robot.VFB_ALIGN_POSE);
                    robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_INTAKE);
                    robot.claw.setPosition(robot.GHEARA_INCHISA);
                    liftState = LiftState.LIFT_RETRACT;
                }

            case LIFT_RETRACT:
                if (Math.abs(-robot.lift.getCurrentPosition() - 600) < 800) {
                    liftState = LiftState.LIFT_START;
                }
        }


        robot.lift.update();
    }

    private void virtualFourBar() {
        if (gamepad1.right_bumper) {
            robot.virtualFourBar.setPosition(robot.VFB_INTAKE);
        } else if (gamepad1.left_bumper) {
            robot.virtualFourBar.setPosition(robot.VFB_OUTTAKE);
        } else if (gamepad1.x) {
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
//        ridicareManuala();
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








