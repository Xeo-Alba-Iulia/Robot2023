package org.firstinspires.ftc.teamcode.sisteme;

import com.qualcomm.robotcore.hardware.ServoImplEx;

public class VirtualFourBar {
    ServoImplEx vFB1, vFB2;
    private static final double  VFB_ALIGN= 404;
    public VirtualFourBar(ServoImplEx vFB1, ServoImplEx vFB2) {
        this.vFB1 = vFB1;
        this.vFB2 = vFB2;
    }

    public void setPosition(double position) {
        vFB1.setPosition(position);
        vFB2.setPosition(1-position);
    }

    public double getPosition() {
        return vFB1.getPosition();
    }


}
