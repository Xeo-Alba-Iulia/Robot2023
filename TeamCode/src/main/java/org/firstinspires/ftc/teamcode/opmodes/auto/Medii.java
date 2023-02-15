package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.camera.AprilRecognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Medii", group = "A", preselectTeleOp = "TeleOP")
public class Medii extends OpMode {

    SampleMecanumDrive drive;

    Pose2d startPose;
    RobotHardware robot;
    TrajectorySequence secventa;
    TrajectorySequence secventa2;

    OpenCvCamera camera;
    AprilRecognition aprilRecognition;
    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    final double TAGSIZE = 0.166;

    private static final int CASE_1 = 7;
    private static final int CASE_2 = 47;
    private static final int CASE_3 = 333;


    int cameraMonitorViewid;



    //Trajectory inceput, align, cone, mid;

    @Override
    public void init(){
        cameraMonitorViewid= hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam
                (hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewid);
        aprilRecognition = new AprilRecognition(TAGSIZE, fx, fy, cx, cy);
        camera.setPipeline(aprilRecognition);
        camera.setPipeline(aprilRecognition);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("n-o mers camera că", errorCode);
                telemetry.update();

            }
        });
        robot = new RobotHardware(this);
        startPose = new Pose2d(35, -60.355, Math.toRadians(270));
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(32, -65.5, Math.toRadians(270.00)));
        secventa = drive.trajectorySequenceBuilder(new Pose2d(32, -65.5, Math.toRadians(270.00)))
                .setReversed(true)
                .splineTo(new Vector2d(56, -39.00),0)
                .build();
        secventa2= drive.trajectorySequenceBuilder(new Pose2d(40,-65.5, Math.toRadians(270))).
                setReversed(true).
                splineTo(new Vector2d(9,-40),60).
                build();
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width

    }

    @Override
    public void start(){
        robot.start();

        drive.followTrajectorySequence(secventa2);
    }
    public void loop() {
         drive.update();
         robot.lift.update();

    }
}
