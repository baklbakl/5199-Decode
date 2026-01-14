package org.ducktapeandaprayer.ftc.dashConfig;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.config.reflection.FieldProvider;
import com.acmerobotics.dashboard.config.variable.BasicVariable;
import com.acmerobotics.dashboard.config.variable.ConfigVariable;
import com.acmerobotics.dashboard.config.variable.CustomVariable;
import com.acmerobotics.dashboard.config.variable.VariableType;
import com.qualcomm.robotcore.util.RobotLog;

import java.lang.reflect.*;
import java.util.List;
import java.util.function.Supplier;

public class ListConfigVariable<T> extends CustomVariable {
    private final transient List<T> list;
    private final transient T nonNullT;
    private int lastListLen = 0;

    private static <TT> ConfigVariable<?> createVariableGeneric(TT value, TT nonNullObj, ValueProvider<TT> basicProvider) {
        Class<TT> klass = (Class<TT>) nonNullObj.getClass();
        VariableType type = VariableType.fromClass(klass);
        switch (type) {
            case BOOLEAN:
            case INT:
            case LONG:
            case FLOAT:
            case DOUBLE:
            case STRING:
            case ENUM:
                return new BasicVariable<>(type, basicProvider);
            case CUSTOM:
                if (value == null) {
                    return new CustomVariableEx<>(null, null, nonNullObj);
                }

                CustomVariableEx<TT> customVariable = new CustomVariableEx<>(value, nonNullObj);

                //ADD: Array support to list config variable
                /*if (fieldClass.isArray()) {
                    for (int i = 0; i < Array.getLength(value); i++) {
                        customVariable.putVariable(Integer.toString(i),
                                createVariableFromArrayField(field,
                                        field.getType().getComponentType(), parent, new int[] {i}));
                    }
                } else*/

                if(!klass.isArray())  {
                    for (Field nestedField : klass.getFields()) {
                        int modifiers = nestedField.getModifiers();
                        if (Modifier.isFinal(modifiers) || Modifier.isStatic(modifiers)) {
                            continue;
                        }

                        String name = nestedField.getName();

                        try {
                            Object nonNullNestedValue = nestedField.get(nonNullObj);
                            if(nonNullNestedValue == null) {
                                throw new NullPointerException("No component of the non null object may be null");
                            }

                            customVariable.putVariable(name, createVariableGeneric(nestedField.get(value), nonNullNestedValue, new FieldProvider<>(nestedField, value)));

                        } catch (IllegalAccessException e) {
                            throw new RuntimeException(e);
                        }
                    }
                }
                return customVariable;

            default:
                throw new RuntimeException("Unsupported field type: " +
                        klass.getName());
        }
    }

    public static ConfigVariable<?> createVariableFromFieldKnownType(Field field, Object parent, Object nonNullParent) {
        try {
            return createVariableGeneric(field.get(parent), field.get(nonNullParent), new FieldProvider<>(field, parent));
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        }
    }

    private ConfigVariable<?> createVariableFromListIndex(int i) {
        return createVariableGeneric(list.get(i), nonNullT, new ValueProvider<T>() {
            @Override
            public T get() {
                return list.get(i);

            }

            @Override
            public void set(T value) {
                list.set(i, value);
            }
        });
    }

    private void createVariables() {
        synchronized (list) {
            lastListLen = list.size();

            for(int i = list.size() - 1; i >= 0; i--) {
                putVariable(String.valueOf(i), createVariableFromListIndex(i));
            }
        }
    }

    private void refresh() {
        synchronized (list) {
            if (lastListLen > list.size()) {
                for (int i = list.size(); i < lastListLen; i++) {
                    super.removeVariable(String.valueOf(i));
                }

            } else if (lastListLen < list.size()) {
                for (int i = lastListLen; i < list.size(); i++) {
                    super.putVariable(String.valueOf(i), createVariableFromListIndex(i));
                }
            }

            for(int i = 0; i < list.size(); i++) {
                ConfigVariable<?> variable = super.getVariable(String.valueOf(i));
                if(variable.getType() == VariableType.CUSTOM) {
                    if(!(variable instanceof CustomVariableEx)) {
                        throw new RuntimeException("Nested Custom Variables must be of type CustomVariableEx");
                    }

                    CustomVariableEx<?> customVar = (CustomVariableEx<?>) variable;

                    if(customVar.objectReference != list.get(i)) {
                        super.removeVariable(String.valueOf(i));
                        super.putVariable(String.valueOf(i), createVariableFromListIndex(i));

                    } else {
                        customVar.checkChildrenUpToDate();
                    }
                }
            }

            lastListLen = list.size();
        }
    }

    public ListConfigVariable(List<T> l, Supplier<T> newElementSupplier, T nonNullTArg) {
        list = l;
        this.nonNullT = nonNullTArg;

        putVariable("Change element count", new BasicVariable<>(new ValueProvider<Integer>() {
            private int resultToSend = 0;

            @Override
            public Integer get() {
                return resultToSend;
            }

            @Override
            public void set(Integer value) {
                RobotLog.ii("ListConfigVariable", "Change element count: " + value);
                if(value < 0) {
                    int absValue = -value;
                    if(list.size() < absValue) {
                        absValue = list.size();

                    }

                    for(int i = 0; i < absValue; i++) {
                        list.remove(list.size() - 1);
                    }

                } else if(value > 0) {
                    for(int i = 0; i < value; i++) {
                        list.add(newElementSupplier.get());
                    }
                }

                refresh();

                resultToSend = value;
                FtcDashboard.getInstance().updateConfig();
                resultToSend = 0;

            }
        }));

        putVariable("Refresh", new BasicVariable<>(new ButtonValueProvider(this::refresh)));

//        putVariable("Log", new BasicVariable<>(new ButtonValueProvider(() -> {
//            synchronized (list) {
//                RobotLog.ii("ListConfigVariable", "{" + list.stream().map(Object::toString).collect(Collectors.joining(", ")) + "}");
//            }
//        })));
//
//        putVariable("Add Element", new BasicVariable<>(new ButtonValueProvider(() -> {
//            synchronized (list) {
//                list.add(newElementSupplier.get());
//            }
//        })));
//
//        putVariable("Clear Element", new BasicVariable<>(new ButtonValueProvider(() -> {
//            synchronized (list) {
//                list.set(0, newElementSupplier.get());
//            }
//        })));
//
//        putVariable("Clear Field", new BasicVariable<>(new ButtonValueProvider(() -> {
//            synchronized (list) {
//                ((MutablePair<MutablePair<Double, Boolean>,Integer>) list.get(0)).setRight(0);
//                ((MutablePair<MutablePair<Double, Boolean>,Integer>) list.get(0)).setLeft(new MutablePair<>(0.0, true));
//            }
//        })));

        createVariables();
    }
}
