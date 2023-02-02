package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class PIDController {
    RobotHardware robot;
    double Kp, Ki, Kd;

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
        robot = new RobotHardware(opMode);
    }

    double lastError = 0;
    double integralSum = 0;
    private static final double integralSumLimit = 1;

    /**
     * update the PID controller output
     *
     * @param target where we would like to be, also called the reference 
     * @param state  where we currently are, I.E. motor position
     * @return the command to our motor, I.E. motor power
     */
    public double update( double target, double state) {
        double error = target - state;
        double derivative = (error - lastError) / robot.timer.seconds();
        integralSum += error * robot.timer.seconds();
        if (integralSum > integralSumLimit) {
            integralSum = integralSumLimit;
        }
        if (integralSum < -integralSumLimit) {
            integralSum = -integralSumLimit;
        }
        double output = Kp * error + Ki * integralSum + Kd * derivative;
        lastError = error;
        return output;
    }

}
