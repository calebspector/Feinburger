package pedroPathing.actions;

import pedroPathing.Action;
import pedroPathing.ActionStatus;
import pedroPathing.Servos;

public class MoveHang implements Action {
    Servos servos;
    double position;

    public MoveHang(Servos servos, double pos) {
        this.servos = servos;
        position = pos;
    }

    @Override
    public ActionStatus run() {
        servos.hang.setPosition(position);
        return ActionStatus.COMPLETED;
    }
}