//package org.firstinspires.ftc.teamcode.opmodes.auto;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.RobotHardware;
//import org.firstinspires.ftc.teamcode.camera.pipelines.AprilRecognition;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.opmodes.teleop.TeleOP;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//@Autonomous(group = "B", preselectTeleOp = "TeleOP")
//public class AutoBlind extends OpMode {
//
//    final static double TAGSIZE = 0.166;
//
//    private static final int CASE_1 = 7;
//
//    private static final int CASE_2 = 47;
//
//    private static final int CASE_3 = 333;
//
//    SampleMecanumDrive drive;
//
//    RobotHardware robot;
//
//    TrajectorySequence secventa1;
//
//    OpenCvCamera camera;
//
//    AprilRecognition aprilRecognition;
//
//    Pose2d startPose;
//
//    Pose2d firstJunction = new Pose2d(35,-21, Math.toRadians(180));
//    TrajectorySequence preload;
//
//    TrajectorySequence stack;
//
//    TrajectorySequence mediumJunction;
//
//    TrajectorySequence  allignStack;
//
//
//    double fx = 578.272;
//
//    double fy = 578.272;
//
//    double cx = 402.145;
//
//    double cy = 221.506;
//
//    int cameraMonitorViewid;
//
//    public enum Autonomous_traj{
//            PRELOAD,
//            STACK,
//            MEDIUMJUNCTION,
//            ALIGNSTACK
//    };
//    public Autonomous_traj autonomous_traj= Autonomous_traj.PRELOAD;
//
//    @Override
//
//    public void init() {
//       robot.claw.setPosition(robot.GHEARA_INCHISA);
//        cameraMonitorViewid = hardwareMap.appContext.getResources().getIdentifier(
//                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam
//                (hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewid);
//        aprilRecognition = new AprilRecognition(TAGSIZE, fx, fy, cx, cy);
//        camera.setPipeline(aprilRecognition);
//        camera.setPipeline(aprilRecognition);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_RIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                telemetry.addData("nu merge", errorCode);
//                telemetry.update();
//            }
//
//        });
//
//        startPose = new Pose2d(35, -63.5, Math.toRadians(270));
//        drive= new SampleMecanumDrive(hardwareMap);
//        preload = drive.trajectorySequenceBuilder(new Pose2d(32, -65.5, Math.toRadians(270)))
//                .lineToLinearHeading(new Pose2d(35, -21, Math.toRadians(0)))
//                .build();
//
//        stack = drive.trajectorySequenceBuilder(new Pose2d(35,-21, Math.toRadians(0)))
//                .strafeRight(7)
//                .lineTo(new Vector2d(57, -12))
//                .build();
//
//        mediumJunction  = drive.trajectorySequenceBuilder(new Pose2d(57,-12, Math.toRadians(0)))
//                .lineTo(new Vector2d(38,-13))
//                .lineToLinearHeading(new Pose2d(33,-15,Math.toRadians(25)))
//                .build();
//
//        allignStack = drive.trajectorySequenceBuilder(new Pose2d(33,-15,Math.toRadians(25)))
//                .lineToLinearHeading(new Pose2d(35,-12, Math.toRadians(0)))
//                .lineTo(new Vector2d(57,-12))
//                .build();
//    }
//
//
//
//    public void loop() {
//        switch (autonomous_traj) {
//            case PRELOAD:
//                if(state_lift_pos!= 600) {
//                    robot.lift.target = state_lift_pos;
//                    robot.virtualFourBar.setPosition(state_vfb_pos);
//                    robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_INTER);
//                    robot.claw.setPosition(robot.GHEARA_INCHISA);
//                    liftState = TeleOP.LiftState.LIFT_EXTEND;
//                }
//                virtualFourBar();
//                break;
//
//            case LIFT_EXTEND:
//                if(Math.abs(-robot.lift.getCurrentPosition() - robot.lift.target) < 800) {
//                    liftTimer.reset();
//                    liftState = TeleOP.LiftState.LIFT_EXTEND2;
//                }
//                break;
//            case LIFT_EXTEND2:
//                if(liftTimer.milliseconds()>200){
//                    robot.claw_alligner.setPosition(state_claw_align);
//                    liftState = TeleOP.LiftState.LIFT_DUMP;
//                }
//                break;
//
//            case LIFT_DUMP:
//                if(gamepad2.dpad_down) {
//                    robot.lift.target = 600;
//                    state_lift_pos= robot.lift.target;
//                    robot.virtualFourBar.setPosition(robot.VFB_ALIGN_POSE);
//                    robot.claw_alligner.setPosition(robot.CLAW_ALLIGN_POS_INTAKE);
//                    robot.claw.setPosition(robot.GHEARA_INCHISA);
//                    liftState = TeleOP.LiftState.LIFT_RETRACT;
//                }
//
//            case LIFT_RETRACT:
//                if(Math.abs(-robot.lift.getCurrentPosition() - 600) < 800) {
//                    liftState = TeleOP.LiftState.LIFT_START;
//                }
//        }
//        drive.update();
//    }
//}
