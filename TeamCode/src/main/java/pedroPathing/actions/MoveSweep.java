package pedroPathing.actions;

import pedroPathing.Action;
import pedroPathing.ActionStatus;
import pedroPathing.Servos;

public class MoveSweep implements Action {
    Servos servos;
    double position;

    public MoveSweep(Servos servos, double pos) {
        this.servos = servos;
        position = pos;
    }

    @Override
    public ActionStatus run() {
        servos.sweep.setPosition(position);
        return ActionStatus.COMPLETED;
    }
}