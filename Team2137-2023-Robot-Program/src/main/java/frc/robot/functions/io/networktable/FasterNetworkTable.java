package frc.robot.functions.io.networktable;

import java.util.*;

public class FasterNetworkTable extends FasterNetworkTableEntry {
    private final Hashtable<Integer, FasterNetworkTableEntry> children = new Hashtable<Integer, FasterNetworkTableEntry>();

    public FasterNetworkTable(String _name) {
        super(_name);
    }

    protected boolean add(FasterNetworkTableEntry addEntry) {
        FasterNetworkTableServer.registerChange();

        if(children.containsKey(addEntry.hashCode())) {
            return false;
        } else {
            addEntry.setPath(this.getPath() + "/" + this.getName());
            children.put(addEntry.hashCode(), addEntry);
            return true;
        }
    }

    public FasterNetworkTableValue<?> getEntry(int hashCode) throws ClassCastException {
        FasterNetworkTableEntry entry = children.get(hashCode);

        if(entry instanceof FasterNetworkTableValue) {
            return (FasterNetworkTableValue<?>) entry;
        } else {
            throw new ClassCastException();
        }
    }

    public FasterNetworkTable getTable(int hashCode) throws ClassCastException {
        FasterNetworkTableEntry entry = children.get(hashCode);

        if(entry instanceof FasterNetworkTable) {
            return (FasterNetworkTable) entry;
        } else {
            throw new ClassCastException();
        }
    }

    public FasterNetworkTableEntry get(int hashCode) throws IndexOutOfBoundsException {
        if(children.containsKey(hashCode))
            return children.get(hashCode);
        else
            throw new IndexOutOfBoundsException();
    }

    public void convertToJson(StringBuilder builder) {
        builder.append("{\n");
        builder.append("\"Name\": \"").append(getName()).append("\",\n");
        builder.append("\"Id\": ").append(getID()).append(",\n");
        builder.append("\"Tables\": [\n");

        children.forEach((a, b) -> {
            if(b instanceof FasterNetworkTable) {
                ((FasterNetworkTable) b).convertToJson(builder);
                builder.append(",\n");
            }
        });

        builder.append("],\n");
        builder.append("\"Entries\": [\n");

        children.forEach((a, b) -> {
            if(b instanceof FasterNetworkTableValue) {
                ((FasterNetworkTableValue<?>) b).convertToJson(builder);
                builder.append(",\n");
            }
        });

        builder.append("]\n");
        builder.append("}");
    }
}
