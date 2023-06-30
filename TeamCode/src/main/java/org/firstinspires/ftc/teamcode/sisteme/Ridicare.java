package org.firstinspires.ftc.teamcode.sisteme;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.PIDController;

public class Ridicare {
    DcMotorEx ridicare1;
    DcMotorEx ridicare2;
    public PIDController controller;



    public static final int POS_1 = 1250;
    public static final int POS_2 = 3100;
    public static final int POS_3 = 4100;

    public int target;



    public Ridicare (DcMotorEx ridicare1, DcMotorEx ridicare2) {
        this.ridicare1 = ridicare1;
        this.ridicare2 = ridicare2;
        target = 0;
        controller = new PIDController(0.004, 0, 0);
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
     * Power given to the motors
     * @return average power given to motors
     */
    public double getPower() {
        return (ridicare1.getPower() + ridicare2.getPower()) / 2.0;
    }

    /**
     * Sets a velocity to both motors
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

    /**
     * Update the power given by the pid controller
     */
    public void update() {
        double power = controller.update(target, ridicare1.getCurrentPosition());
        ridicare1.setPower(power);
        ridicare2.setPower(power);
    }


 }
