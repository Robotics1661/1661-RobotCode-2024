package frc.robot.util;

import java.util.function.Supplier;

public class MutableObject<T> implements Supplier<T> {
    public MutableObject(T val) {
        this.val = val;
    }

    private T val;

    public void set(T val) {
        this.val = val;
    }

    public T get() {
        return val;
    }
}
