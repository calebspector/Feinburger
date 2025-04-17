package pedroPathing.actions;

import pedroPathing.Action;
import pedroPathing.ActionStatus;
import pedroPathing.Servos;

public class MoveShoulder implements Action {
    Servos servos;
    double position;

    public MoveShoulder(Servos servos, double pos) {
        this.servos = servos;
        position = pos;
    }

    @Override
    public ActionStatus run() {
        servos.shoulder.setPosition(position);
        servos.shoulder2.setPosition(position);
        return ActionStatus.COMPLETED;
    }
}