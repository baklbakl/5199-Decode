package org.ducktapeandaprayer.ftc.dashConfig;

import com.acmerobotics.dashboard.config.variable.ConfigVariable;
import com.acmerobotics.dashboard.config.variable.CustomVariable;
import com.acmerobotics.dashboard.config.variable.VariableType;

import java.lang.reflect.Field;
import java.util.Map;

public class CustomVariableEx<T> extends CustomVariable {
    public final T objectReference;
    public final T nonNullT;

    public CustomVariableEx(T objReference, T nonNullT) {
        super();
        this.nonNullT = nonNullT;
        this.objectReference = objReference;
    }

    public CustomVariableEx(Map<String, ConfigVariable> variables, T objReference, T nonNullT) {
        super(variables);
        this.nonNullT = nonNullT;
        this.objectReference = objReference;
    }

    public void checkChildrenUpToDate() {
        Map<String, ConfigVariable> variables = ((Map<String, ConfigVariable>) super.getValue());

        for (String key : variables.keySet()) {
            ConfigVariable variable = super.getVariable(key);
            if(variable.getType() == VariableType.CUSTOM) {
                if(!(variable instanceof CustomVariableEx)) {
                    throw new RuntimeException("Nested Custom Variables must be of type CustomVariableEx");
                }

                CustomVariableEx customVar = (CustomVariableEx) variable;

                try {
                    Field field = objectReference.getClass().getField(key);

                    if(customVar.objectReference != field.get(objectReference)) {
                        super.removeVariable(key);
                        super.putVariable(key, ListConfigVariable.createVariableFromFieldKnownType(field, objectReference, nonNullT));
                    } else {
                        customVar.checkChildrenUpToDate();
                    }
                } catch (IllegalAccessException | NoSuchFieldException e) {
                    throw new RuntimeException(e);
                }
            }
        }
    }
}
