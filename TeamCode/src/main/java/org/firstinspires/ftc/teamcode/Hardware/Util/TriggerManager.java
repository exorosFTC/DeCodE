package org.firstinspires.ftc.teamcode.Hardware.Util;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.function.Supplier;

public class TriggerManager {
    private List<AbsTrigger> triggers = new ArrayList<>();
    private List<Runnable> actions = new ArrayList<>();

    public TriggerManager addTrigger(Supplier<Boolean> condition, Runnable action) {
        triggers.add(new Trigger(condition, action));
        return this;
    }

    public TriggerManager addTriggerSequence(Supplier<Boolean> condition, List<Runnable> actions) {
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
        private final Supplier<Boolean> condition;
        private final Runnable action;

        public Trigger(Supplier<Boolean> condition, Runnable action) {
            this.condition = condition;
            this.action = action;
        }

        public void check() {
            if (condition.get()) action.run();
        }
    }

    private static class TriggerSequence extends AbsTrigger {
        private final Supplier<Boolean> condition;
        private final List<Runnable> actions;
        private int index = 0;

        private TriggerSequence(Supplier<Boolean> condition, List<Runnable> actions) {
            this.condition = condition;
            this.actions = actions;
        }

        public void check() {
            if (condition.get()) {
                if (index == actions.size()) index = 0;

                actions.get(index).run();
                index += 1;
            }
        }
    }
}
