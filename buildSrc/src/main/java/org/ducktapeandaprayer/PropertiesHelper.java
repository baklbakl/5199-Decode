package org.ducktapeandaprayer;

import org.gradle.api.Project;

public class PropertiesHelper {
    public static String getStringProperty(Project target, String name) {
        return target.property(name).toString();
    }

    public static int getIntProperty(Project target, String name) {
        String prop = getStringProperty(target, name);
        try {
            return Integer.parseInt(prop);
        } catch (NumberFormatException e) {
            throw new RuntimeException("ERROR: Property \"" + name + "\" should be a int, but it is \"" + prop + "\"", e);
        }
    }
}
