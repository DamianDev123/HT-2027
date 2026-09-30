package org.firstinspires.ftc.teamcode.Solvers.StateMachine.Definitions;

import android.util.Log;


import org.firstinspires.ftc.teamcode.Globals.Robot;

import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

public interface StateMachine {
    Map<StateMachine, MachineData> MACHINES = new HashMap<>();
    Robot robot = Robot.getInstance();
    class MachineData {
        Map<String, Method> states = new HashMap<>();
        Method currentState;
        Method firstState;
        long stateStartTime;
        boolean justEntered = false;
    }

    default void initStateMachine() {
        MachineData data = new MachineData();

        for (Method method : getClass().getDeclaredMethods()) {

            boolean isState =
                    method.isAnnotationPresent(State.class) ||
                            method.isAnnotationPresent(TimedState.class);

            if (!isState) {
                continue;
            }

            data.states.put(method.getName(), method);

            boolean first = false;

            if (method.isAnnotationPresent(State.class)) {
                first = Objects.requireNonNull(method.getAnnotation(State.class)).first();
            }

            if (method.isAnnotationPresent(TimedState.class)) {
                first = Objects.requireNonNull(method.getAnnotation(TimedState.class)).first();
            }

            if (first) {
                data.currentState = method;
                data.firstState = method;
            }
        }

        if (data.currentState == null) {
            throw new RuntimeException(
                    "No first state defined."
            );
        }

        data.stateStartTime = System.currentTimeMillis();
        data.justEntered = true;
        MACHINES.put(this, data);
    }
    default void update() {

        MachineData data = MACHINES.get(this);



        assert data != null;
        robot.telemetry.addData("In State: ", data.currentState.getName());
        if (data.currentState == null) {
            return;
        }

        try {

            data.currentState.setAccessible(true);
            data.currentState.invoke(this);

            // Handle timed states
            if (data.currentState.isAnnotationPresent(TimedState.class)) {

                TimedState timed =
                        data.currentState.getAnnotation(TimedState.class);

                double elapsed =
                        (System.currentTimeMillis()
                                - data.stateStartTime) / 1000.0;

                assert timed != null;
                if (elapsed >= timed.time()) {
                    nextState(timed.next());
                }
            }

        } catch (Exception e) {
            Log.i("F", Objects.requireNonNull(e.getMessage()));
        }
        data.justEntered = false;
    }
    default void nextState() {

        MachineData data = MACHINES.get(this);

        if (data.firstState == null) {
            throw new RuntimeException("No first state defined.");
        }

        data.currentState = data.firstState;

        data.stateStartTime = System.currentTimeMillis();

        data.justEntered = true;
    }
    default void nextState(String name){
        MachineData data = MACHINES.get(this);

        assert data != null;
        Method next = data.states.get(name);
        if (next == null) {
            throw new RuntimeException("State not found: " + name);
        }
        data.currentState = next;
        data.stateStartTime = System.currentTimeMillis();

        data.justEntered = true;
    }
    default void onUpdate(Method method, boolean justEntered) throws Exception {
        method.invoke(this, justEntered);
    }
    default boolean justEntered() {return  Objects.requireNonNull(MACHINES.get(this)).justEntered;};

    default String getCurrentStateName() {
        MachineData data = MACHINES.get(this);
        assert data != null;
        if (data.currentState == null) {
            return "NONE";
        }
        return data.currentState.getName();
    }
}
