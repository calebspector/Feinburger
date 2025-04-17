package pedroPathing.actions;

import pedroPathing.Action;
import pedroPathing.ActionStatus;
import pedroPathing.Servos;

public class MoveWrist implements Action {
    Servos servos;
    double position;

    public MoveWrist(Servos servos, double pos) {
        position = pos;
    }

    @Override
    public ActionStatus run() {
        servos.wrist.setPosition(position);
        return ActionStatus.COMPLETED;
    }
}