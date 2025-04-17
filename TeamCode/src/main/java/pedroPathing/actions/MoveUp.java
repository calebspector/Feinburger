package pedroPathing.actions;

import pedroPathing.Action;
import pedroPathing.ActionStatus;
import pedroPathing.Servos;

public class MoveUp implements Action {
    Servos servos;
    double position;

    public MoveUp(Servos servos, double pos) {
        position = pos;
    }

    @Override
    public ActionStatus run() {
        servos.up.setPosition(position);
        return ActionStatus.COMPLETED;
    }
}