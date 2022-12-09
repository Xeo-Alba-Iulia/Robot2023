package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class PIDController {
    double Kp, Ki, Kd;
    RobotHardware robot;
    /**
     * construct PID controller
     * @param Kp Proportional coefficient
     * @param Ki Integral coefficient
     * @param Kd Derivative coefficient
     */
    public PIDController(double Kp, double Ki, double Kd, OpMode opmode) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        robot = new RobotHardware(opmode);
    }

    double lastError = 0;
    double integralSum = 0;
    private static final double integralSumLimit = 0.25;
    double output;
    /**
     * update the PID controller output
     * @param target where we would like to be, also called the reference
     * @param state where we currently are, I.E. motor position
     * @return the command to our motor, I.E. motor power
     */
    public double update(double target, double state) {
        double error = target - state;
        double derivative = (error - lastError) / robot.timer.seconds();
        integralSum += error * robot.timer.seconds();
        if(integralSum > integralSumLimit) {
            integralSum = integralSumLimit;
        }
        if(integralSum < -integralSumLimit){
            integralSum = -integralSumLimit;
        }
        output = Kp * error + Ki * integralSum + Kd * derivative;
        robot.telemetry.update();
        lastError = error;
        return output;
    }

}
