package org.firstinspires.ftc.teamcode.Solvers.Subsystems;

import com.pedropathing.geometry.Pose;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;

public class Filter {
    private final int MEDIAN_FILTER_SIZE = 9;
    private final Queue<Double> XBuffer = new LinkedList<>();
    private final double[] XArray = new double[MEDIAN_FILTER_SIZE];
    double xValue = 0;
    public Filter(){
        initializeFilterBuffers();
    }
    private void initializeFilterBuffers() {
        for (int i = 0; i < MEDIAN_FILTER_SIZE; i++) {
            XBuffer.add(0.0);
        }
    }
    private double applyMedianFilter(Queue<Double> buffer, double newValue, double[] array) {
        buffer.poll();
        buffer.add(newValue);

        int index = 0;
        for (Double value : buffer) {
            array[index++] = value;
        }

        Arrays.sort(array);

        return array[MEDIAN_FILTER_SIZE / 2];
    }
    public double updateFilteredVelocities(Double value) {
        double rawX = value;
        xValue = applyMedianFilter(XBuffer, rawX, XArray);
        return xValue;
    }
    public void updateFilteredVelocities(Double value, boolean empty) {
        double rawX = value;
        xValue = applyMedianFilter(XBuffer, rawX, XArray);
    }
    public double getX(){
        return xValue;
    }
}
