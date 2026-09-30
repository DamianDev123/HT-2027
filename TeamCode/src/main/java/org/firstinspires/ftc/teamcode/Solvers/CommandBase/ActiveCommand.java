package org.firstinspires.ftc.teamcode.Solvers.CommandBase;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Retention(RetentionPolicy.RUNTIME)   // Available at runtime
@Target(ElementType.TYPE)             // Can be placed on classes
public @interface ActiveCommand {

}
