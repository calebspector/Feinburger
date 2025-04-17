package pedroPathing.actions;

import com.qualcomm.robotcore.hardware.DcMotor;

import pedroPathing.Action;
import pedroPathing.ActionStatus;
import pedroPathing.Motors;

public class MoveArm implements Action {
    Motors motors;
    int position;
    double power;

    public MoveArm(Motors motors, int pos, double pow) {
        position = pos;
        power = pow;
    }

    @Override
    public ActionStatus run() {
        motors.rightArm.setTargetPosition(position);
        motors.rightArm.setTargetPositionTolerance(30);
        motors.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors.rightArm.setPower(power);
        return ActionStatus.COMPLETED;
    }
}