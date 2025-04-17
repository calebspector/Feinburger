package pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Motors {
    public List<DcMotorEx> all;
    public DcMotorEx left, left2, right, rightArm;

    public Motors(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotorEx.class, "leftM");
        left2 = hardwareMap.get(DcMotorEx.class, "leftM2");
        right = hardwareMap.get(DcMotorEx.class, "rightM");
        rightArm = hardwareMap.get(DcMotorEx.class, "rightArm");

        all.add(left);
        all.add(left2);
        all.add(right);
        all.add(rightArm);
    }

    void setMode(DcMotor.RunMode mode) {
        for (DcMotor i : all) {
            i.setMode(mode);
        }
    }

    void setDirection(DcMotorSimple.Direction dir) {
        for (DcMotor i : all) {
            i.setDirection(dir);
        }
    }

    void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zpb) {
        for (DcMotor i : all) {
            i.setZeroPowerBehavior(zpb);
        }
    }
}
