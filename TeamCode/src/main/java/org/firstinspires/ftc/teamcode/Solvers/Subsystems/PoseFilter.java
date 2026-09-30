package org.firstinspires.ftc.teamcode.Solvers.Subsystems;

import com.pedropathing.geometry.Pose;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;

public class PoseFilter {
    private int MEDIAN_FILTER_SIZE = 9;
    private final Queue<Double> XBuffer = new LinkedList<>();
    private final Queue<Double> YBuffer = new LinkedList<>();
    private final Queue<Double> headingBuffer = new LinkedList<>();
    private final double[] XArray = new double[MEDIAN_FILTER_SIZE];
    private final double[] YArray = new double[MEDIAN_FILTER_SIZE];
    private final double[] headingArray = new double[MEDIAN_FILTER_SIZE];
    public  Pose filteredPose = new Pose(0,0,0);
    private void initializeFilterBuffers() {
        for (int i = 0; i < MEDIAN_FILTER_SIZE; i++) {
            XBuffer.add(0.0);
            YBuffer.add(0.0);
            headingBuffer.add(0.0);
        }
    }
    public PoseFilter(){
        initializeFilterBuffers();
    }
    public PoseFilter(int FILTER_SIZE){
        MEDIAN_FILTER_SIZE = FILTER_SIZE;
        initializeFilterBuffers();
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
    public Pose updateFilteredVelocities(Pose pose) {
        double rawX = pose.getX();
        double rawY = pose.getY();
        double rawHeadingVel = pose.getHeading();
        return new Pose(
                applyMedianFilter(XBuffer, rawX, XArray),
                applyMedianFilter(YBuffer, rawY, YArray),
                applyMedianFilter(headingBuffer, rawHeadingVel, headingArray)
        );
    }
}
