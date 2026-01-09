package org.ducktapeandaprayer;

import org.gradle.api.Plugin;
import org.gradle.api.Project;
import org.gradle.api.artifacts.dsl.RepositoryHandler;

public class RepositoriesPlugin implements Plugin<Project> {
    @Override
    public void apply(Project target) {
        RepositoryHandler repos = target.getRepositories();
        repos.google();
        repos.mavenCentral();
        repos.maven((repo) -> {
           repo.setUrl("https://maven.brott.dev/");
        });
    }
}
