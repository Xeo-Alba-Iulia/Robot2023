package org.firstinspires.ftc.teamcode.sisteme;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utilities.PIDController;

public class Ridicare {
    DcMotorEx ridicare1;
    DcMotorEx ridicare2;
    PIDController controller;
    public int target;
    public Ridicare (DcMotorEx ridicare1, DcMotorEx ridicare2, OpMode myOpMode) {
        this.ridicare1 = ridicare1;
        this.ridicare2 = ridicare2;
        target = 0;
        controller = new PIDController(0.001, 0.0012, 0.02975039, myOpMode);
    }

    /**
     * Give power to both lift motors
     * @param power power given to both motors
     */
    public void setPower(double power) {
        ridicare1.setPower(power);
        ridicare2.setPower(power);
    }

    /**
     * get position of the motor with the encoder attached
     * @return ridicare1's current encoder's ticks
     */
    public int getCurrentPosition() {
        return ridicare1.getCurrentPosition();
    }

    /**
     * get the average velocity between the 2 motors
     * @return (speed of motor1 + speed of motor 2) / 2
     */
    public double getAverageVelocity() {
        double speed1 = ridicare1.getVelocity();
        double speed2 = ridicare2.getVelocity();
        return (speed1 + speed2) / 2.0;
    }



    /**
     * reset both encoder's ticks
     * even if one of them is redundant, for safety
     */
    public void resetEncoder() {
        ridicare1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ridicare2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * update the power given by the pid controller
     */
    public void update() {
        ridicare1.setPower(controller.update(target, ridicare1.getCurrentPosition()));
        ridicare2.setPower(controller.update(target, ridicare1.getCurrentPosition()));
    }

}
