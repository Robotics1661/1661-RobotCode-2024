package frc.robot.annotationprocessor;

import java.util.Set;

import com.google.auto.service.AutoService;
import javax.annotation.processing.*;
import javax.annotation.processing.AbstractProcessor;
import javax.annotation.processing.RoundEnvironment;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.TypeElement;

@AutoService(Processor.class)
public class INamedCommandProcessor extends AbstractProcessor {

    @Override
    public Set<String> getSupportedAnnotationTypes() {
        return Set.of("frc.robot.annotationprocessor.INamedCommand");
    }

    @Override
    public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv) {
        System.err.println("Testing 123");
        if (true) throw new RuntimeException("I broke something!!"); // TODO: actual processing
        return false;
    }

    @Override
    public SourceVersion getSupportedSourceVersion() {
        return SourceVersion.RELEASE_17;
    }
}
