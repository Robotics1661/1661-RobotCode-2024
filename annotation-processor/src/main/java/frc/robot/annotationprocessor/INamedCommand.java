package frc.robot.annotationprocessor;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Registers a named command for PathPlanner
 * 
 * Should only annotate static methods implementing {@link frc.robot.commands.named.INamedCommandBuilder#build}
 * When run in a simulation, will throw on errors.
 */
@Target({ElementType.METHOD, ElementType.CONSTRUCTOR})
@Retention(RetentionPolicy.RUNTIME)
public @interface INamedCommand {
    /** PathPlanner named command name */
    String value();
}
