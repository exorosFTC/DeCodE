package org.firstinspires.ftc.teamcode.Hardware.Util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class TriggerManager {
    private final List<AbsTrigger> triggers = new ArrayList<>();
    private final List<Runnable> actions = new ArrayList<>();

    public TriggerManager addTrigger(BooleanSupplier condition, Runnable action) {
        triggers.add(new Trigger(condition, action));
        return this;
    }

    public TriggerManager addTriggerSequence(BooleanSupplier condition, List<Runnable> actions) {
        triggers.add(new TriggerSequence(condition, actions));
        return this;
    }

    public TriggerManager addAction(Runnable action) {
        actions.add(action);
        return this;
    }


    public void check() {
        for (AbsTrigger t : triggers) t.check();
        for (Runnable r: actions) r.run();
    }






    static abstract class AbsTrigger {
        public abstract void check();
    }

    private static class Trigger extends AbsTrigger {
        private final BooleanSupplier condition;
        private final Runnable action;

        public Trigger(BooleanSupplier condition, Runnable action) {
            this.condition = condition;
            this.action = action;
        }

        public void check() {
            if (condition.getAsBoolean()) action.run();
        }
    }

    private static class TriggerSequence extends AbsTrigger {
        private final BooleanSupplier condition;
        private final List<Runnable> actions;
        private int index = 0;

        private TriggerSequence(BooleanSupplier condition, List<Runnable> actions) {
            this.condition = condition;
            this.actions = actions;
        }

        public void check() {
            if (condition.getAsBoolean()) {
                if (index == actions.size()) index = 0;

                actions.get(index).run();
                index += 1;
            }
        }
    }
}
