package net.polarbub;

import org.gradle.api.Plugin;
import org.gradle.api.Project;
import org.gradle.api.Task;

//Warning. This does mess up the configuration cache. To run it use the `--no-configuration-cache` option with gradle
public class StubPlugin implements Plugin<Project> {
    @Override
    public void apply(Project target) {
        target.getPlugins().forEach((plugin) -> {
            System.out.println("Plugin: " + plugin);
        });

        target.getTasks().register("testTask", (Task t) -> {
//            System.out.println("Hello from stub plugin!");

            t.doFirst((Task t2) -> {
                System.out.println("Hello \"" + target.getName() + "\" from testTask!");
            });

        });
    }
}
