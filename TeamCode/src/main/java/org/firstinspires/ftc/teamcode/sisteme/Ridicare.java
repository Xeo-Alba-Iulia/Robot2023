package org.firstinspires.ftc.teamcode.sisteme;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Ridicare {
    PIDCoefficients coefficients;

    public PIDFController controller;
    public static final int POS_1 = 1250;
    public static final int POS_2 = 3100;
    public static final int POS_3 = 4100;
    public int target = 0;
    public double maxVelo =20;
    public double maxAccel=50;
    DcMotorEx ridicare1;
    DcMotorEx ridicare2;
    public MotionProfile profile;
    public ElapsedTime profileTimer = new ElapsedTime();


    public Ridicare(DcMotorEx ridicare1, DcMotorEx ridicare2) {
        this.ridicare1 = ridicare1;
        this.ridicare2 = ridicare2;
        target = 0;
        coefficients = new PIDCoefficients(0.5,0,0);
        controller = new PIDFController(coefficients);
        controller.setTargetPosition(target);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(ridicare1.getCurrentPosition(), 0, 0),
                new MotionState(target, 0, 0),
                maxVelo,
                maxAccel,
                80,
                true
        );
        profileTimer.reset();

    }

    /**
     * Power given to the motors
     *
     * @return average power given to motors
     */
    public double getPower() {

        return (ridicare1.getPower() + ridicare2.getPower()) / 2.0;
    }

    /**
     * Give power to both lift motors
     *
     * @param power power given to both motors
     */
    public void setPower(double power) {
        ridicare1.setPower(power);
        ridicare2.setPower(power);
    }

    /**
     * Sets a velocity to both motors
     *
     * @param velocity given to both motors
     */
    public void setVelocity(double velocity) {
        ridicare1.setVelocity(velocity);
        ridicare2.setVelocity(velocity);

    }


    public double getVelocity(double velocity) {
        return ridicare1.getVelocity();

    }

    public void printVelocity(Telemetry telemetry) {
        telemetry.addData("lift1 velocity", ridicare1.getVelocity());
        telemetry.addData("lift2 velocity", ridicare2.getVelocity());
    }

    /**
     * Get position of the motor with the encoder attached
     *
     * @return ridicare1's current encoder's ticks
     */
    public int getCurrentPosition() {

        return ridicare1.getCurrentPosition();
    }


    /**
     * Reset both encoder's ticks
     * even if one of them is redundant, for safety
     */
    public void resetEncoders() {
        ridicare1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ridicare2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    double lastTarget = target;
    /**
     * Update the power given by the pid controller
     */
    public void update() {
        if (target != lastTarget) {
            //restart timer for motion profile
            profileTimer.reset();
            profile = MotionProfileGenerator.generateSimpleMotionProfile(
                    new MotionState(getCurrentPosition(), 0, 0),
                    new MotionState(target, 0, 0),
                    maxVelo,
                    maxAccel,
                    80,
                    true
            );


        }
        //the pid controller gets the reference given by the motion profile
        controller.setTargetPosition(profile.get(profileTimer.time()).getX());
        double power = controller.update(ridicare1.getCurrentPosition());

        ridicare1.setPower(power);
        ridicare2.setPower(power);
        lastTarget = target;
    }


}
