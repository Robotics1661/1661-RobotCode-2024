package frc.robot.annotationprocessor;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import javax.annotation.processing.AbstractProcessor;
import javax.annotation.processing.Processor;
import javax.annotation.processing.RoundEnvironment;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.Element;
import javax.lang.model.element.ElementKind;
import javax.lang.model.element.ExecutableElement;
import javax.lang.model.element.Modifier;
import javax.lang.model.element.TypeElement;
import javax.lang.model.element.VariableElement;
import javax.lang.model.type.DeclaredType;
import javax.lang.model.type.TypeKind;
import javax.lang.model.type.TypeMirror;
import javax.tools.Diagnostic;

import com.google.auto.service.AutoService;

@AutoService(Processor.class)
public class INamedCommandProcessor extends AbstractProcessor {

    private static Set<String> registeredCommands = new HashSet<>();

    @Override
    public Set<String> getSupportedAnnotationTypes() {
        return Set.of("frc.robot.annotationprocessor.INamedCommand");
    }

    private static boolean isKindOk(Element element) {
        ElementKind kind = element.getKind();
        return kind == ElementKind.METHOD || kind == ElementKind.CONSTRUCTOR;
    }

    @Override
    public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv) {
        Set<? extends Element> annotatedHandlers = roundEnv.getElementsAnnotatedWith(INamedCommand.class);

        for (Element e : annotatedHandlers) {
            if (!isKindOk(e))
                processingEnv.getMessager().printMessage(Diagnostic.Kind.ERROR, "@INamedCommand can only be applied to static methods and constructors, %s does not comply"
                    .formatted(repr(e)));
        }

        annotatedHandlers.stream()
            .filter(INamedCommandProcessor::isKindOk)
            .map((e) -> (ExecutableElement) e)
            .forEach((e) -> {
                try {
                    processElement(e);
                } catch (CompileException ex) {
                    processingEnv.getMessager().printMessage(Diagnostic.Kind.ERROR, ex.getMessage());
                }
            });
        return true;
    }

    private void processElement(ExecutableElement e) throws CompileException {
        INamedCommand metadata = e.getAnnotation(INamedCommand.class);
        if (!registeredCommands.add(metadata.value())) {
            throw new CompileException("@INamedCommand: duplicate registry of command '%s' by %s"
                .formatted(metadata.value(), repr(e)));
        }
        if (!e.getModifiers().contains(Modifier.PUBLIC)) {
            throw new CompileException("@INamedCommand can only be applied to public methods and constructors, %s is not public"
                .formatted(repr(e)));
        }
        if (e.getReceiverType().getKind() != TypeKind.NONE) {
            throw new CompileException("@INamedCommand can only be applied to static methods and constructors, %s has a receiver of type %s"
                .formatted(repr(e), e.getReceiverType().getKind()));
        }
        if (e.getReturnType().getKind() == TypeKind.DECLARED) {
            DeclaredType returnType = (DeclaredType) e.getReturnType();
            TypeElement returnElement = (TypeElement) returnType.asElement();
            if (!isOrExtends(returnElement, "edu.wpi.first.wpilibj2.command.Command")) {
                throw new CompileException("@INamedCommand can only be applied to methods returning Command or constructors of Command subclasses, %s returns %s"
                    .formatted(repr(e), returnElement.getQualifiedName()));
            }
        } else if (e.getKind() == ElementKind.CONSTRUCTOR) {
            if (!isOrExtends((TypeElement) e.getEnclosingElement(), "edu.wpi.first.wpilibj2.command.Command")) {
                throw new CompileException("@INamedCommand can only be applied to methods returning Command or constructors of Command subclasses, %s does not subclass Command"
                    .formatted(repr(e)));
            }
        } else {
            throw new CompileException("@INamedCommand can only be applied to methods returning Command or constructors of Command subclasses, %s returns type of kind %s"
                .formatted(repr(e), e.getReturnType().getKind()));
        }
        List<? extends VariableElement> params = e.getParameters();
        String[] expectedParams = new String[] {"frc.robot.subsystems.DrivetrainSubsystem", "frc.robot.subsystems.FourBarSubsystem", "frc.robot.subsystems.IntakeSubsystem"};
        if (params.size() != expectedParams.length) {
            throw new CompileException("@INamedCommand expects a method with the signature `(DriveTrainSubsystem, FourBarSubsystem, IntakeSubsystem) -> Command`, %s has the wrong number of parameters"
                .formatted(repr(e)));
        }
        for (int i = 0; i < expectedParams.length; i++) {
            String expectedName = expectedParams[i];
            VariableElement param = params.get(i);
            TypeMirror type = param.asType();
            if (type.getKind() != TypeKind.DECLARED) {
                throw new CompileException("@INamedCommand expects a method with the signature `(DriveTrainSubsystem, FourBarSubsystem, IntakeSubsystem) -> Command`, %s does not match"
                    .formatted(repr(e)));
            }
            TypeElement typeElement = (TypeElement) ((DeclaredType) type).asElement();
            if (!typeElement.getQualifiedName().contentEquals(expectedName)) {
                throw new CompileException("@INamedCommand expects a method with the signature `(DriveTrainSubsystem, FourBarSubsystem, IntakeSubsystem) -> Command`, %s does not match"
                    .formatted(repr(e)));
            }
        }

        // Parameters checked, return type checked, should be good to go.
        System.out.println("Sucessfully registered named command '%s' to %s"
            .formatted(metadata.value(), repr(e)));
    }

    @Override
    public SourceVersion getSupportedSourceVersion() {
        return SourceVersion.RELEASE_17;
    }

    private static String repr(Element e) {
        String className = ((TypeElement) e.getEnclosingElement()).getQualifiedName().toString();
        return className + "#" + e.getSimpleName().toString();
    }

    private static boolean isOrExtends(TypeElement clazz, String qualifiedName) {
        if (clazz.getQualifiedName().contentEquals(qualifiedName)) {
            return true;
        }
        TypeMirror zuper = clazz.getSuperclass();
        if (zuper.getKind() == TypeKind.DECLARED) {
            return isOrExtends((TypeElement) ((DeclaredType) zuper).asElement(), qualifiedName);
        }
        return false;
    }

    private static class CompileException extends Exception {
        public CompileException(String message) {
            super(message);
        }
    }
}
