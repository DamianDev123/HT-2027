package org.firstinspires.ftc.teamcode.Solvers.StateMachine.Definitions;

import org.firstinspires.ftc.teamcode.Solvers.CommandBase.Command;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.lang.reflect.Executable;
import java.lang.reflect.Method;

@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.METHOD)
public @interface TimedState {
    double time();
    String next();
    boolean first() default false;
}


