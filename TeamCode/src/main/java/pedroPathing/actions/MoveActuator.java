package pedroPathing.actions;

import pedroPathing.Action;
import pedroPathing.ActionStatus;
import pedroPathing.Servos;

public class MoveActuator implements Action {
    Servos servos;
    double position;
    double pullUpPower;

    public MoveActuator(Servos servos, double pos, double pow) {
        this.servos = servos;
        position = pos;
        pullUpPower = pow;
    }

    @Override
    public ActionStatus run() {
        servos.actuator.setPosition(position);
        servos.pullUp.setPosition(pullUpPower);
        return ActionStatus.COMPLETED;
    }
}