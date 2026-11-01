package org.firstinspires.ftc.teamcode.helpers;

import com.pedropathing.math.Pose;

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
    public Pose filteredPose = new Pose(0,0,0);
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
    private void applyMedianFilter(Queue<Double> buffer, double newValue, double[] array) {
        buffer.poll();
        buffer.add(newValue);

        int index = 0;
        for (Double value : buffer) {
            array[index++] = value;
        }

        Arrays.sort(array);

    }
    public void updateFilteredPose(Pose pose) {
        double rawX = pose.x();
        double rawY = pose.y();
        double rawHeadingVel = pose.heading();
        applyMedianFilter(XBuffer, rawX, XArray);
        applyMedianFilter(YBuffer, rawY, YArray);
        applyMedianFilter(headingBuffer, rawHeadingVel, headingArray);
    }
}