package frc.robot.functions.io.networktable;

import java.lang.constant.Constable;
import java.lang.reflect.ParameterizedType;

public class FasterNetworkTableValue<T extends Constable> extends FasterNetworkTableEntry {

    public enum SupportValueType {
        String ("String"),
        Int ("Int"),
        Double ("Double"),
        Boolean ("Boolean"),
        Unknown ("Unknown");

        final String name;

        SupportValueType(String tmp) {
            name = tmp;
        }

        public String toString() {
            return name;
        }
    }

    private T defaultValue;
    private T value;
    private SupportValueType type;

    public FasterNetworkTableValue(String name, T _defaultValue) {
        super(name);

        defaultValue = _defaultValue;

        if(_defaultValue.getClass() == String.class) {
            type = SupportValueType.String;
        } else if(_defaultValue.getClass() == Integer.class) {
            type = SupportValueType.Int;
        } else if(_defaultValue.getClass() == Double.class) {
            type = SupportValueType.Double;
        } else if(_defaultValue.getClass() == Boolean.class) {
            type = SupportValueType.Boolean;
        }
    }

    public FasterNetworkTableValue(String name, T _defaultValue, T _value) {
        this(name, _defaultValue);
        setValue(_value);
    }

    public void setValue(T _value) {
        value = _value;
    }

    public T getValue() {
        return value;
    }

    public T getDefaultValue() {
        return defaultValue;
    }

    public SupportValueType getType() {
        return type;
    }

    private String getJsonValueString() {
        switch(type) {
            case Int:
            case Double:
            case Boolean:
                return value.toString();
            case String:
                return "\"" + value + "\"";
        }

        return "\"" + value + "\"";
    }

    public void convertToJson(StringBuilder builder) {
        builder.append("{\n");
        builder.append("\"Name\": \"").append(getName()).append("\",\n");
        builder.append("\"Id\": ").append(getID()).append(",\n");
        builder.append("\"ValueType\": \"").append(type.toString()).append("\",\n");
        builder.append("\"DefaultValue\": ").append(getDefaultValue()).append("\n");
        builder.append("}");
    }
}
