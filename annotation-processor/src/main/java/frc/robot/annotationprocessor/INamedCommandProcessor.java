package frc.robot.annotationprocessor;

import java.io.Writer;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

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
import javax.tools.FileObject;
import javax.tools.StandardLocation;

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

        if (annotations.isEmpty() || annotatedHandlers.isEmpty())
            return false;

        for (Element e : annotatedHandlers) {
            if (!isKindOk(e))
                processingEnv.getMessager().printMessage(Diagnostic.Kind.ERROR, "@INamedCommand can only be applied to static methods and constructors, %s does not comply"
                    .formatted(repr(e)));
        }

        List<String> entries = new ArrayList<>();

        annotatedHandlers.stream()
            .filter(INamedCommandProcessor::isKindOk)
            .map((e) -> (ExecutableElement) e)
            .forEach((e) -> {
                try {
                    processElement(e, entries);
                } catch (CompileException ex) {
                    processingEnv.getMessager().printMessage(Diagnostic.Kind.ERROR, ex.getMessage());
                }
            });
        String commandsXml = entries.stream().collect(Collectors.joining());
        StringBuilder finalDescriptor = new StringBuilder()
            .append("<?xml version=\"1.0\" encoding=\"utf-8\" ?>")
            .append("\n<named-commands>")
            .append(commandsXml)
            .append("\n</named-commands>\n");            

        try {
            /*System.out.println("hello1");
            System.out.println(annotations.toString());
            System.out.println(annotatedHandlers.toString());*/
            FileObject file = processingEnv.getFiler().createResource(StandardLocation.SOURCE_OUTPUT, "resources.frc.robot", "named_commands.xml");
            //System.out.println("hello2");
            Writer writer = file.openWriter();
            //System.out.println("hello3");
            writer.write(finalDescriptor.toString());
            writer.close();
        } catch (Exception ex) {
            ex.printStackTrace();
            processingEnv.getMessager().printMessage(Diagnostic.Kind.ERROR, "Failed to write named commands file");
        }

        return true;
    }

    private void processElement(ExecutableElement e, List<String> entries) throws CompileException {
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
        String[] expectedParams = new String[] {"frc.robot.subsystems.AutonomousInput"};
        if (params.size() != expectedParams.length) {
            throw new CompileException("@INamedCommand expects a method with the signature `(AutonomousInput) -> Command`, %s has the wrong number of parameters"
                .formatted(repr(e)));
        }
        for (int i = 0; i < expectedParams.length; i++) {
            String expectedName = expectedParams[i];
            VariableElement param = params.get(i);
            TypeMirror type = param.asType();
            if (type.getKind() != TypeKind.DECLARED) {
                throw new CompileException("@INamedCommand expects a method with the signature `(AutonomousInput) -> Command`, %s does not match"
                    .formatted(repr(e)));
            }
            TypeElement typeElement = (TypeElement) ((DeclaredType) type).asElement();
            if (!typeElement.getQualifiedName().contentEquals(expectedName)) {
                throw new CompileException("@INamedCommand expects a method with the signature `(AutonomousInput) -> Command`, %s does not match"
                    .formatted(repr(e)));
            }
        }

        // Create entries
        if (e.getKind() == ElementKind.METHOD) {
            /*
    <command name="test_command_method">
        <class>frc.robot.commands.named.TestCommand</class>
        <method>
            <name>build</name>
            <return>frc.robot.commands.named.TestCommand</return>
            <parameters>
                <parameter>frc.robot.subsystems.DrivetrainSubsystem</parameter>
                <parameter>frc.robot.subsystems.FourBarSubsystem</parameter>
                <parameter>frc.robot.subsystems.IntakeSubsystem</parameter>
            </parameters>
        </method>
    </command>
            */
            StringBuilder entry = new StringBuilder()
                .append("\n\t<command name=\"").append(metadata.value()).append("\">")
                .append("\n\t\t<class>").append(getEnclosingClassName(e)).append("</class>")
                .append("\n\t\t<method>")
                .append("\n\t\t\t<name>").append(e.getSimpleName()).append("</name>")
                .append("\n\t\t\t<return>").append(((DeclaredType) e.getReturnType()).toString()).append("</return>")
                .append("\n\t\t\t<parameters>");
            for (VariableElement param : params) {
                TypeElement paramType = (TypeElement) ((DeclaredType) param.asType()).asElement();
                entry.append("\n\t\t\t\t<parameter>").append(paramType.getQualifiedName()).append("</parameter>");
            }
            entry
                .append("\n\t\t\t</parameters>")
                .append("\n\t\t</method>")
                .append("\n\t</command>");
            entries.add(entry.toString());
        } else if (e.getKind() == ElementKind.CONSTRUCTOR) {
            /*
    <command name="test_command">
        <class>frc.robot.commands.named.TestCommand</class>
        <constructor>
            <parameter>frc.robot.subsystems.DrivetrainSubsystem</parameter>
            <parameter>frc.robot.subsystems.FourBarSubsystem</parameter>
            <parameter>frc.robot.subsystems.IntakeSubsystem</parameter>
        </constructor>
    </command>
            */
            StringBuilder entry = new StringBuilder()
                .append("\n\t<command name=\"").append(metadata.value()).append("\">")
                .append("\n\t\t<class>").append(getEnclosingClassName(e)).append("</class>")
                .append("\n\t\t<constructor>");
            for (VariableElement param : params) {
                TypeElement paramType = (TypeElement) ((DeclaredType) param.asType()).asElement();
                entry.append("\n\t\t\t<parameter>").append(paramType.getQualifiedName()).append("</parameter>");
            }
            entry
                .append("\n\t\t</constructor>")
                .append("\n\t</command>");
            entries.add(entry.toString());
        }

        // Parameters checked, return type checked, should be good to go. NOTE: runtime annotation processing is still needed for actual registration
        System.out.println("Sucessfully registered named command '%s' to %s"
            .formatted(metadata.value(), repr(e)));
    }

    @Override
    public SourceVersion getSupportedSourceVersion() {
        return SourceVersion.RELEASE_17;
    }

    private static String getEnclosingClassName(Element e) {
        return ((TypeElement) e.getEnclosingElement()).getQualifiedName().toString();
    }

    private static String repr(Element e) {
        String className = getEnclosingClassName(e);
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

/*
File example:
```xml
<?xml version="1.0" encoding="utf-8" ?>
<named-commands>
    <command name="test_command">
        <class>frc.robot.commands.named.TestCommand</class>
        <constructor>
            <parameter>frc.robot.subsystems.DrivetrainSubsystem</parameter>
            <parameter>frc.robot.subsystems.FourBarSubsystem</parameter>
            <parameter>frc.robot.subsystems.IntakeSubsystem</parameter>
        </constructor>
    </command>
    <command name="test_command_method">
        <class>frc.robot.commands.named.TestCommand</class>
        <method>
            <name>build</name>
            <return>frc.robot.commands.named.TestCommand</return>
            <parameters>
                <parameter>frc.robot.subsystems.DrivetrainSubsystem</parameter>
                <parameter>frc.robot.subsystems.FourBarSubsystem</parameter>
                <parameter>frc.robot.subsystems.IntakeSubsystem</parameter>
            </parameters>
        </method>
    </command>
</named-commands>
```
*/