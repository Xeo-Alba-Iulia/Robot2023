package org.firstinspires.ftc.teamcode.sisteme;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class Ridicare {
    private final RobotHardware hw;
    public Ridicare (RobotHardware hw) {
        this.hw = hw;
    }

    public void setPower(double power) {
        hw.ridicare1.setPower(power);
        hw.ridicare2.setPower(power);
    }


}
