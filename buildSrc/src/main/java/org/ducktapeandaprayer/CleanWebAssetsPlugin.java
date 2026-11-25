package org.ducktapeandaprayer;

import com.android.build.api.dsl.AndroidLibrarySourceSet;
import com.android.build.api.dsl.LibraryExtension;

import org.gradle.api.Plugin;
import org.gradle.api.Project;
import org.gradle.api.file.FileTree;
import org.gradle.api.tasks.Delete;
import org.gradle.api.tasks.TaskProvider;

import java.io.File;
import java.util.Set;

//Warning. This does mess up the configuration cache. To run it use the `--no-configuration-cache` option with gradle
public class CleanWebAssetsPlugin implements Plugin<Project> {
    @Override
    public void apply(Project target) {
        TaskProvider<Delete> cleanWebAssetsTask = target.getTasks().register("cleanWebAssets", Delete.class, (Delete t) -> {
            LibraryExtension extension = target.getExtensions().getByType(LibraryExtension.class);
            AndroidLibrarySourceSet sourceSet = extension.getSourceSets().getByName("main");

            Set<String> directories = sourceSet.getAssets().getDirectories();
            if(directories.size() > 1) {
                throw new RuntimeException("too many asset directories in sourceSet");
            }

            for(File file : target.file(directories.stream().findFirst().get() + "/web/").listFiles()) {
                t.delete(file);
            }
            //FileTree fileTree = target.fileTree(directories.stream().findFirst().get() + "/web/");
            //fileTree.visit((f) -> {
            //    t.delete(f.getFile());
            //});
        });

        target.getTasks().getByName("clean").dependsOn(cleanWebAssetsTask);
    }
}
