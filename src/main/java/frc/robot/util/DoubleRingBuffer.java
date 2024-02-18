package frc.robot.util;

public class DoubleRingBuffer {
    private final int max_size;
    private final double[] backing;
    private int size;
    private int idx;

    public DoubleRingBuffer(int size) {
        this.max_size = size;
        this.size = 0;
        this.idx = 0;
        this.backing = new double[size];
    }

    public void push(double v) {
        backing[idx++] = v;
        idx %= max_size;
        if (size < max_size)
            size += 1;
    }

    public double getAverage() {
        if (size == 0) {
            return 0;
        } else {
            double sum = 0;
            for (int i = 0; i < size; i++) {
                sum += backing[i];
            }
            return sum / size;
        }
    }

    public double smooth(double v) {
        push(v);
        return getAverage();
    }
}
