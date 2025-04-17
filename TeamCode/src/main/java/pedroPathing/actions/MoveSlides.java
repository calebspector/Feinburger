package pedroPathing.actions;

import com.qualcomm.robotcore.hardware.DcMotor;

import pedroPathing.Action;
import pedroPathing.ActionStatus;
import pedroPathing.Motors;
import pedroPathing.constants.CalebConstants;

public class MoveSlides implements Action {
    Motors motors;
    int position;

    public MoveSlides(Motors motors, int position) {
        this.motors = motors;
        this.position = position;
    }

    @Override
    public ActionStatus run() {
        motors.left.setTargetPosition((int)(position * CalebConstants.ARM_MULTIPLIER));
        motors.left2.setTargetPosition((int)(position * CalebConstants.ARM_MULTIPLIER));
        motors.right.setTargetPosition((int)(position * CalebConstants.ARM_MULTIPLIER));

        motors.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors.left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motors.left.setPower(1);
        motors.left2.setPower(1);
        motors.right.setPower(1);

        return ActionStatus.COMPLETED;
    }
}