package org.ducktapeandaprayer;

import com.android.build.api.dsl.AndroidLibrarySourceSet;
import com.android.build.api.dsl.LibraryExtension;

import org.gradle.api.Plugin;
import org.gradle.api.Project;
import org.gradle.api.Task;
import org.gradle.api.file.FileTree;
import org.gradle.api.tasks.Delete;
import org.gradle.api.tasks.TaskProvider;

import java.io.File;
import java.util.Set;

//Warning. This does mess up the configuration cache. To run it use the `--no-configuration-cache` option with gradle
public class StubPlugin implements Plugin<Project> {
    @Override
    public void apply(Project target) {
//        target.getPlugins().forEach((plugin) -> {
//            System.out.println("Plugin: " + plugin);
//        });
//
//        target.getTasks().register("testTask", (Task t) -> {
////            System.out.println("Hello from stub plugin!");
//
//            t.doFirst((Task t2) -> {
//                System.out.println("Hello \"" + target.getName() + "\" from testTask!");
//            });
//
//        });

         TaskProvider<Delete> cleanWebAssetsTask = target.getTasks().register("cleanWebAssets", Delete.class, (Delete t) -> {
            LibraryExtension extension = target.getExtensions().getByType(LibraryExtension.class);
            AndroidLibrarySourceSet sourceSet = extension.getSourceSets().getByName("main");

            Set<String> directories = sourceSet.getAssets().getDirectories();
            if(directories.size() > 1) {
                throw new RuntimeException("too many asset directories in sourceSet");
            }

            FileTree fileTree = target.fileTree(directories.stream().findFirst().get() + "/web/");
            fileTree.visit((f) -> {
                t.delete(f.getFile());
            });
        });

        target.getTasks().getByName("clean").dependsOn(cleanWebAssetsTask);
    }
}
