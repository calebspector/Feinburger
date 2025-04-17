package pedroPathing;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@FunctionalInterface
interface Action {
    boolean run();
}

class SequentialAction implements Action{
    List<Action> actions;
    SequentialAction(List<Action> initialActions){
        this.actions=initialActions;
    }

    SequentialAction(Action... actions) {
        this(Arrays.asList(actions));
    }

    @Override
    public boolean run() {
        if (actions.isEmpty()) {
            return false;
        }

        if (actions.get(0).run()) {
            return true;
        } else {
            actions = actions.subList(1, actions.size());
            return run();
        }
    }
}

class ParallelAction implements Action {
    private final List<Action> actions;

    ParallelAction(List<Action> initialActions) {
        this.actions = initialActions;
    }

    ParallelAction(Action... actions) {
        this(Arrays.asList(actions));
    }

    @Override
    public boolean run() {
        actions.removeIf(action -> !action.run());
        return !actions.isEmpty();
    }
}

class RaceAction implements Action {
    private final List<Action> actions;

    RaceAction(List<Action> actions) {
        this.actions = actions;
    }

    RaceAction(Action... actions) {
        this(Arrays.asList(actions));
    }

    @Override
    public boolean run() {
        return actions.stream().allMatch(Action::run);
    }
}

class SleepAction implements Action {
    private double beginTs = -1d;
    private final double dt;

    SleepAction(double dt) {
        this.dt = dt;
    }

    boolean init=false;
    @Override
    public boolean run() {
        if (!init){
            beginTs=System.currentTimeMillis();
            init=true;
        }
        return System.currentTimeMillis()-beginTs < dt;
    }
}

@FunctionalInterface
interface InstantFunction {
    void run();
}

class InstantAction implements Action {
    private final InstantFunction f;

    InstantAction(InstantFunction f) {
        this.f = f;
    }

    @Override
    public boolean run() {
        f.run();
        return false;
    }
}

class ImplicitAction {
    public static double now() {
        return System.nanoTime() - 1e-9;
    }
    private Action seqCons(Action hd, Action tl) {
        if (tl instanceof SequentialAction) {
            List<Action> newActions = new ArrayList<>();
            newActions.add(hd);
            newActions.addAll(((SequentialAction) tl).actions);
            return new SequentialAction(newActions);
        }
        return new SequentialAction(hd, tl);
    }
}