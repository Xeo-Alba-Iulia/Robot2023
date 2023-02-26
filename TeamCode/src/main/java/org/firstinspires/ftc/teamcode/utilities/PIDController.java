package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class PIDController {
    private static final double integralSumLimit = 1000;
    private static final double MARGIN_OF_ERROR = 0;
    RobotHardware robot;
    double Kp, Ki, Kd;
    public ElapsedTime timer;
    double lastError = 0;
    double integralSum = 0;
    double lastTime = 0;
    double lastReference = 0;
    double lastTarget = 0;
    OpMode opMode;

    /**
     * construct PID controller
     *
     * @param Kp Proportional coefficient
     * @param Ki Integral coefficient
     * @param Kd Derivative coefficient
     */
    public PIDController(double Kp, double Ki, double Kd, OpMode opMode) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.opMode = opMode;
        robot = new RobotHardware(opMode);
        timer = new ElapsedTime();
    }

    /**
     * update the PID controller output
     *
     * @param target where we would like to be, also called the reference
     * @param state  where we currently are, I.E. motor position
     * @return the command to our motor, I.E. motor power
     */
    public double update(double target, double state) {
        double error = target - state;
        double derivative = (error - lastError) / timer.seconds();
        integralSum += error * timer.seconds();
        // reset integral sum upon setpoint changes
        if (target != lastReference) {
            integralSum = 0;
        }
        if (integralSum > integralSumLimit) {
            integralSum = integralSumLimit;
        }
        if (integralSum < -integralSumLimit) {
            integralSum = -integralSumLimit;
        }
        double output = Kp * error + Ki * integralSum + Kd * derivative;
        lastError = error;
        lastTime = timer.seconds();
        lastReference = target;
        timer.reset();
        return output;
    }

}
