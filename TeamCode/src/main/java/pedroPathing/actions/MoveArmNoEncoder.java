package pedroPathing.actions;

import com.qualcomm.robotcore.hardware.DcMotor;

import pedroPathing.Action;
import pedroPathing.ActionStatus;
import pedroPathing.Motors;
import pedroPathing.Servos;
import pedroPathing.constants.CalebConstants;

public class MoveArmNoEncoder implements Action {
    Motors motors;
    Servos servos;
    double power;

    public MoveArmNoEncoder(Motors motors, Servos servos, double power) {
        this.motors = motors;
        this.servos = servos;
        this.power = power;
    }

    @Override
    public ActionStatus run() {
        if (motors.rightArm.getCurrentPosition() > CalebConstants.armMax + CalebConstants.armCushion && servos.shoulder.getPosition() < 0.9 || (servos.shoulder.getPosition() > 0.9 && motors.rightArm.getCurrentPosition() > CalebConstants.armWithShoulderInFront + CalebConstants.armCushion)) {
            power = Math.min(power, -0.5);
        } else if (motors.rightArm.getCurrentPosition() > CalebConstants.armMax && servos.shoulder.getPosition() < 0.9 || (servos.shoulder.getPosition() > 0.9 && motors.rightArm.getCurrentPosition() > CalebConstants.armWithShoulderInFront)) {
            power = Math.min(power, 0);
        }

        if (motors.rightArm.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION) || armZeroing) {
            if (Math.abs(power) > 0.5 /*||(!armZeroing&&Math.abs(rightArm.getCurrentPosition()-rightArm.getTargetPosition())<50)*/ ) {
                armZeroing = false;
                motors.rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.rightArm.setPower(power);
            }
        } else {
            motors.rightArm.setPower(power);
        }

        return ActionStatus.COMPLETED;
    }
}
