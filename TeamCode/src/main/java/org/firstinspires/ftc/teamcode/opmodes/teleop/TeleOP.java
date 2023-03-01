package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.sisteme.Ridicare;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utilities.PosStorage;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "TeleOP", group = "A")
public class TeleOP extends OpMode {

    boolean stackToggle;
    boolean intakeToggle;
    boolean clawToggle;
    boolean currentA;
    boolean currentX;
    boolean currentDdown;




    public enum LiftState {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_EXTEND2,
        LIFT_DUMP,
        LIFT_RETRACT
    }


    public LiftState liftState = LiftState.LIFT_START;


    RobotHardware robot = new RobotHardware(this);
    private double state_claw_align=robot.CLAW_ALLIGN_POS_INTAKE;
    private static int state_lift_pos=600;
    private double state_vfb_pos=robot.VFB_ALIGN_POSE;

    StandardTrackingWheelLocalizer myLocalizer;
    SampleMecanumDrive drive;
    TrajectorySequence junction1;

    @Override
    public void init() {
        stackToggle = false;
        clawToggle = false;
        intakeToggle = false;
        currentA = false;
        currentX = false;
        currentDdown = false;
        robot.init();
        robot.virtualFourBar.setPosition(robot.VFB_ALIGN_POSE);
        robot.claw.setPosition(robot.GHEARA_DESCHISA);
        robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_INTAKE);
        drive = new SampleMecanumDrive(hardwareMap);
        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();
        myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);
        myLocalizer.setPoseEstimate(PosStorage.currentPose);
    }

    public void allignJunction() {
        myLocalizer.setPoseEstimate(PosStorage.currentPose);
        drive.setPoseEstimate(PosStorage.currentPose);
        Pose2d myPose = drive.getPoseEstimate();
        //sau
//         Pose2d myPose = myLocalizer.getPoseEstimate();
        Pose2d posjunction1 = new Pose2d(-31, 8, 135);

        junction1 = drive.trajectorySequenceBuilder(myPose)
                .lineToLinearHeading(posjunction1)
                .build();

    }

    private void intake() {
        if (gamepad2.dpad_down != currentDdown) {
            currentDdown = gamepad1.a;
            if (gamepad2.dpad_down) {
                intakeToggle = !intakeToggle;
                if (intakeToggle) {
                    robot.virtualFourBar.setPosition(robot.VFB_ALIGN_POSE);
                } else {
                    robot.virtualFourBar.setPosition(robot.VFB_ALIGN_POSE);
                }
            }
        }
    }

    ///timp
    ElapsedTime liftTimer = new ElapsedTime();
    private void intakeOuttake() {

        if(gamepad2.dpad_left){
            state_lift_pos=Ridicare.POS_1;
            state_claw_align=robot.CLAW_ALLIGN_POS_LOW;
            state_vfb_pos=robot.VFB_LOW;
        }

        else if(gamepad2.dpad_up) {
            state_lift_pos = Ridicare.POS_2;
            state_claw_align=robot.CLAW_ALLIGN_POS_HIGH;
            state_vfb_pos=robot.VFB_MEDIUM;
        }
        else if(gamepad2.dpad_right) {
            state_lift_pos = Ridicare.POS_3;
            state_claw_align=robot.CLAW_ALLIGN_POS_HIGH;
            state_vfb_pos=robot.VFB_HIGH;
        }

        switch (liftState) {
            case LIFT_START:
                if(state_lift_pos!= 600) {
                    robot.lift.target = state_lift_pos;
                    robot.virtualFourBar.setPosition(state_vfb_pos);
                    robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_INTER);
                    robot.claw.setPosition(robot.GHEARA_INCHISA);
                    liftState = LiftState.LIFT_EXTEND;
                }
                virtualFourBar();
                break;

            case LIFT_EXTEND:
                if(Math.abs(-robot.lift.getCurrentPosition() - robot.lift.target) < 800) {
                    liftTimer.reset();
                    liftState = LiftState.LIFT_EXTEND2;
                }
                break;
            case LIFT_EXTEND2:
                if(liftTimer.milliseconds()>200){
                    robot.claw_alligner.setPosition(state_claw_align);
                    liftState = LiftState.LIFT_DUMP;
                }
                break;

            case LIFT_DUMP:
                if(gamepad2.dpad_down) {
                    robot.lift.target = 600;
                    state_lift_pos= robot.lift.target;
                    robot.virtualFourBar.setPosition(robot.VFB_ALIGN_POSE);
                    robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_INTAKE);
                    robot.claw.setPosition(robot.GHEARA_INCHISA);
                    liftState = LiftState.LIFT_RETRACT;
                }

            case LIFT_RETRACT:
                if(Math.abs(-robot.lift.getCurrentPosition() - 600) < 800) {
                    liftState = LiftState.LIFT_START;
                }
        }


        robot.lift.update();
    }

    private void virtualFourBar() {
        if (gamepad1.left_bumper) {
            robot.virtualFourBar.setPosition(robot.VFB_INTAKE_POSE);
        } else if (gamepad2.right_stick_button) {
            robot.virtualFourBar.setPosition(robot.VFB_OUTTAKE_POSE);
        } else if (gamepad2.left_stick_button) {
            robot.virtualFourBar.setPosition(robot.VFB_ALIGN_POSE);
        }
    }

    private void allignclaw() {
        if (gamepad2.x) {
            robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_UP);
        } else if (gamepad2.y) {
            robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_INTAKE);
        } else if (gamepad2.right_bumper)
            robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_FALLEN);

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

    private void cazut() {
        if (gamepad1.x != currentX) {
            currentX = gamepad1.x;
            if (gamepad1.x) {
                clawToggle = !clawToggle;
                if (clawToggle) {
                    robot.claw.setPosition(robot.VFB_FALLEN);
                    robot.virtualFourBar.setPosition(robot.VFB_FALLEN);
                    robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_FALLEN);
                    robot.lift.target = 600;
                } else {
                    robot.claw.setPosition(robot.VFB_ALIGN_POSE);
                    robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_INTAKE);

                }
            }
        }
        robot.lift.update();
    }

    private void stackModifier() {
        if (gamepad1.b) {
            robot.VFB_STACK_POSE += 0.005;
            robot.virtualFourBar.setPosition(robot.VFB_STACK_POSE);
        } else if (gamepad1.y) {
            robot.VFB_STACK_POSE -= 0.005;
            robot.virtualFourBar.setPosition(robot.VFB_STACK_POSE);
        }

    }

//    private void verify_lift_auto() {
//        switch () {
//            case 1: {
//                if (robot.lift.getCurrentPosition() == Ridicare.POS_1) {
//                    robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_LOW);
//                    dpad_pressed = false;
//                }
//            }
//            break;
//            case 2: {
//                if (robot.lift.getCurrentPosition() == Ridicare.POS_2) {
//                    robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_LOW);
//                    dpad_pressed = false;
//                }
//            }
//            break;
//            case 3: {
//                if (robot.lift.getCurrentPosition() == Ridicare.POS_3) {
//                    robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_HIGH);
//                    dpad_pressed = false;
//                }
//            }
//            break;
//        }
//
//    }

    private void telemetry() {
        telemetry.addData("Lift State", liftState);
        telemetry.addLine("Ridicare");
        telemetry.addData("Pozitie", robot.lift.getCurrentPosition());
        telemetry.addLine("Poz Vfb");
        telemetry.addData("Stack", robot.vfb_stack_pose);
        telemetry.update();
    }

    public void loop() {

        myLocalizer.update();
        drive.update();
        robot.movement(gamepad1);
        intakeOuttake();
       // virtualFourBar();
        claw();
        allignJunction();
        allignclaw();
        cazut();
        stackModifier();


        telemetry();
        telemetry.update();
    }

}
