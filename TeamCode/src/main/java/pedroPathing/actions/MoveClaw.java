package pedroPathing.actions;

import pedroPathing.Action;
import pedroPathing.ActionStatus;
import pedroPathing.Servos;

public class MoveClaw implements Action {
    Servos servos;
    double position;

    public MoveClaw(Servos servos, double pos) {
        this.servos = servos;
        position = pos;
    }

    @Override
    public ActionStatus run() {
        servos.claw.setPosition(position);
        return ActionStatus.COMPLETED;
    }
}