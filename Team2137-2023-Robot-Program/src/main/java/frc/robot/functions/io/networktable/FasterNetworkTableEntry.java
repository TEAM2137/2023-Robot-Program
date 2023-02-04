package frc.robot.functions.io.networktable;


public class FasterNetworkTableEntry {
    private String name;
    private String path = "";
    private final int localID;

    public FasterNetworkTableEntry(String _name) {
        name = _name;
        localID = this.hashCode();
    }

    public int getID() {
        return localID;
    }

    public String getName() {
        return name;
    }

    public String getPath() {
        return path;
    }

    public void setPath(String _path) {
        path = _path;
    }

    @Override
    public String toString() {
        return getPath() + "/" + getName();
    }
}
