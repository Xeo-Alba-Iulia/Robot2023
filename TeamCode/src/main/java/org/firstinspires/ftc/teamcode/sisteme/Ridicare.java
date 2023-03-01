package org.firstinspires.ftc.teamcode.sisteme;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utilities.PIDController;

public class Ridicare {
    DcMotorEx ridicare1;
    DcMotorEx ridicare2;
    public PIDController controller;

    public static final int POS_1 = 9300;
    public static final int POS_2 = 16200;
    public static final int POS_3 = 19500;
    public int target;

    public Ridicare (DcMotorEx ridicare1, DcMotorEx ridicare2) {
        this.ridicare1 = ridicare1;
        this.ridicare2 = ridicare2;
        target = 0;
        controller = new PIDController(0.0007, 0, 0.000001723790323);
    }

    /**
     * Give power to both lift motors
     * @param power power given to both motors
     */
    public void setPower(double power) {
        ridicare1.setPower(power);
        ridicare2.setPower(power);
    }

    public double getPower() {
        return (ridicare1.getPower() + ridicare2.getPower()) / 2.0;
    }

    /**
     * Get position of the motor with the encoder attached
     * @return ridicare1's current encoder's ticks
     */
    public int getCurrentPosition() {
        return ridicare1.getCurrentPosition();
    }

    /**
     * Get the average velocity between the 2 motors
     * @return (speed of motor1 + speed of motor 2) / 2
     */
    public double getAverageVelocity() {
        double speed1 = ridicare1.getVelocity();
        double speed2 = ridicare2.getVelocity();
        return (speed1 + speed2) / 2.0;
    }

    /**
     * Reset both encoder's ticks
     * even if one of them is redundant, for safety
     */
    public void resetEncoder() {
        ridicare1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ridicare2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Update the power given by the pid controller
     */
    public void update() {
        double power = controller.update(target, -ridicare1.getCurrentPosition());
        ridicare1.setPower(power);
        ridicare2.setPower(power);
    }
}
