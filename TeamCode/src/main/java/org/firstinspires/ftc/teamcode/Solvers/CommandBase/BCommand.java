package org.firstinspires.ftc.teamcode.Solvers.CommandBase;

public interface BCommand {
    default boolean isFinished() {
        return false;
    }
    default void schedule() {}
    default void initialize() {}
    default void execute() {}
    default void end() {}
}
