package pedroPathing.actions;

import pedroPathing.Action;
import pedroPathing.ActionStatus;
import pedroPathing.Motors;
import pedroPathing.Servos;
import pedroPathing.constants.CalebConstants;

public class ActionManager {
    final Motors motors;
    final Servos servos;

    public ActionManager(Motors motors, Servos servos) {
        this.motors = motors;
        this.servos = servos;
    }

    public Action moveArm(int position, double power, ActionStatus action) {
        Action a = new MoveArm(motors, position, power);
        if (action == ActionStatus.IN_PROGRESS) {
            a.run();
        }
        return a;
    }

    public Action moveSlides(int position, ActionStatus action) {
        Action a = new MoveSlides(motors, position);
        if (action == ActionStatus.IN_PROGRESS) {
            a.run();
        }
        return a;
    }

    public Action moveSlidesNoEncoder(double power, ActionStatus action) {
        Action a = new MoveSlidesNoEncoder(motors, servos, power);
        if (action == ActionStatus.IN_PROGRESS) {
            a.run();
        }
        return a;
    }

    public Action moveArmNoEncoder(double power, ActionStatus action) {
        Action a = new MoveArmNoEncoder(motors, servos, power);
        if (action == ActionStatus.IN_PROGRESS) {
            a.run();
        }
        return a;
    }

    public Action moveSweep(double pos, ActionStatus action) {
        Action a = new MoveSweep(servos, pos);
        if (action == ActionStatus.IN_PROGRESS) {
            a.run();
        }
        return a;
    }

    public Action moveUp(double pos, ActionStatus action) {
        Action a = new MoveUp(servos, pos);
        if (action == ActionStatus.IN_PROGRESS) {
            a.run();
        }
        return a;
    }

    public Action moveHang(double pos, ActionStatus action) {
        Action a = new MoveHang(servos, pos);
        if (action == ActionStatus.IN_PROGRESS) {
            a.run();
        }
        return a;
    }

    public Action moveClaw(double pos, ActionStatus action) {
        Action a = new MoveClaw(servos, pos);
        if (action == ActionStatus.IN_PROGRESS) {
            a.run();
        }
        return a;
    }

    public Action moveShoulder(double pos, ActionStatus action) {
        Action a = new MoveShoulder(servos, pos);
        if (action == ActionStatus.IN_PROGRESS) {
            a.run();
        }
        return a;
    }

    public Action moveWrist(double pos, ActionStatus action) {
        Action a = new MoveWrist(servos, pos);
        if (action == ActionStatus.IN_PROGRESS) {
            a.run();
        }
        return a;
    }

    public Action disengageActuator(ActionStatus action) {
        Action a = new MoveActuator(servos, CalebConstants.ACTUATORDISENGAGE, 0.5);
        if (action == ActionStatus.IN_PROGRESS) {
            a.run();
        }
        return a;
    }

    public Action engageActuator(ActionStatus action) {
        Action a = new MoveActuator(servos, CalebConstants.ACTUATORENGAGE, 1);
        if (action == ActionStatus.IN_PROGRESS) {
            a.run();
        }
        return a;
    }
}
