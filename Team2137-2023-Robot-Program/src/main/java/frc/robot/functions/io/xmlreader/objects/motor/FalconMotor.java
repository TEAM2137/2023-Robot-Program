package frc.robot.functions.io.xmlreader.objects.motor;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.functions.io.xmlreader.Entity;
import frc.robot.functions.io.xmlreader.data.PID;
import frc.robot.library.units.AngleUnits.Angle;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.TranslationalUnits.Velocity;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.FOOT;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.INCH;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.CTRE_VELOCITY;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.FEET_PER_SECOND;


public class FalconMotor extends TalonFX implements Entity, SimpleMotorControl {

    private NetworkTable savedNetworkTableInstance;
    private final org.w3c.dom.Element savedElement;
    private String name;
    private Runnable onImplement;

    public static final int NUMBEROFPIDSLOTS = 2;

    private String canLoopName;
    private int id = 0;
    private final Motor.MotorTypes type = Motor.MotorTypes.NEO;
    private boolean inverted = false;
    private int currentLimit = 0;
    private double gearRatio = 0;
    private double rampRate = 0;
    private final PID[] pidValues = new PID[NUMBEROFPIDSLOTS];
    private CANSparkMax.IdleMode idleMode = CANSparkMax.IdleMode.kCoast;

    private Distance distancePerRevolution;

    private static final int CountsPerRevolution = 2048;

    public FalconMotor(Element element) {
        super(Integer.parseInt(Entity.getOrDefault(element, "ID", "0")), Entity.getOrDefault(element, "CANLoop", "rio"));

        name = getNodeOrAttribute(element, "Name", "none");
        canLoopName = getNodeOrAttribute(element, "CANLoop", "rio");

        savedElement = element;

        NodeList tmpList = element.getElementsByTagName("PID");

        for (int i = 0; i < tmpList.getLength(); i++) {
            if(!((Element) tmpList.item(i)).hasAttribute("Slot")) {
                this.pidValues[0] = new PID((Element) tmpList.item(i));
                continue;
            }

            String value = ((Element) tmpList.item(i)).getAttribute("Slot");
            int slotNumber = 0;

            if (!value.equals(""))
                slotNumber = Integer.parseInt(value);

            if (slotNumber >= NUMBEROFPIDSLOTS) {
                DriverStation.reportError("PID Slot too large", false);
            } else {
                this.pidValues[slotNumber] = new PID((Element) tmpList.item(i));
            }
        }

        configFactoryDefault();

        setID(Integer.parseInt(Entity.getOrDefault(element, "ID", "0")));
        setMotorType(Motor.MotorTypes.valueOf(Entity.getOrDefault(element, "Type", "FALCON").toUpperCase()));
        setInverted(Boolean.parseBoolean(Entity.getOrDefault(element, "Inverted", "false").toLowerCase()));
        setCurrentLimit(Integer.parseInt(Entity.getOrDefault(element, "CurrentLimit", "-1")));
        setGearRatio(Double.parseDouble(Entity.getOrDefault(element, "GearRatio", "1")));
        setRampRate(Double.parseDouble(Entity.getOrDefault(element, "RampRate", "0")));
    }

    @Override
    public double get() {
        return super.getMotorOutputPercent();
    }

    @Override
    public void set(double val) {
        set(ControlMode.PercentOutput, val);
    }
    @Override
    public void setPosition(Distance distance) {
        set(TalonFXControlMode.Position, (distance.getValue(Distance.DistanceUnits.INCH) / distancePerRevolution.getValue(Distance.DistanceUnits.INCH)) * getGearRatio() *  getCountPerRevolution());
    }
    @Override
    public void setPosition(Angle angle) {
        set(TalonFXControlMode.Position, (angle.getValue(Angle.AngleUnits.DEGREE) / 360.0) * getGearRatio() * getCountPerRevolution());
    }
    @Override
    public Distance getPosition() {
        return new Distance(super.getSelectedSensorPosition() / getGearRatio() / getCountPerRevolution() * distancePerRevolution.getValue(INCH), INCH);
    }


    @Override
    public void configureForwardLimit(Distance d) {
        super.configForwardSoftLimitThreshold((d.getValue(INCH) / distancePerRevolution.getValue(INCH)) * getGearRatio() * getCountPerRevolution());
        super.configForwardSoftLimitEnable(true);
    }
    @Override
    public void enableForwardLimit() {
        super.configForwardSoftLimitEnable(true);
    }
    @Override
    public void disableForwardLimit() {
        super.configForwardSoftLimitEnable(false);
    }

    @Override
    public void configureReverseLimit(Distance d) {
        super.configReverseSoftLimitThreshold((d.getValue(INCH) / distancePerRevolution.getValue(INCH)) * getGearRatio() * getCountPerRevolution());
        super.configReverseSoftLimitEnable(true);
    }
    @Override
    public void enableReverseLimit() {
        super.configReverseSoftLimitEnable(true);
    }
    @Override
    public void disableReverseLimit() {
        super.configReverseSoftLimitEnable(false);
    }

    @Override
    public void setVelocity(Velocity velocity) {
        Velocity vel = velocity.getCTREVelocityUnit(new Distance(1.0 / distancePerRevolution.getValue(FOOT), FOOT));
        set(TalonFXControlMode.Velocity, vel.getValue(CTRE_VELOCITY));
    }

    @Override
    public Distance getDistancePerRevolution() {
        return distancePerRevolution;
    }
    @Override
    public void setDistancePerRevolution(Distance distance) {
        distancePerRevolution = distance;
    }


    @Override
    public int getCountPerRevolution() {
        return CountsPerRevolution;
    }

    @Override
    public void setIntegratedSensorPosition(double val) {
        setSelectedSensorPosition(val);
    }

    @Override
    public int getID() {
        return id;
    }

    @Override
    public void setNeutralMode(CANSparkMax.IdleMode mode) {
        super.setNeutralMode(mode == CANSparkMax.IdleMode.kCoast ? NeutralMode.Coast : NeutralMode.Brake);
        idleMode = mode;
    }

    @Override
    public void setID(int _id) {
        id = _id;
    }

    @Override
    public void follow(SimpleMotorControl other) {
        if(other instanceof FalconMotor) {
            follow(((TalonFX) other));
        }
    }

    //--------------------------------------------Gear Ratio----------------------------------------------------------//
    @Override
    public double getGearRatio() {
        return gearRatio;
    }
    @Override
    public void setGearRatio(double ratio) {
        gearRatio = ratio;
    }

    //--------------------------------------------Motor Type----------------------------------------------------------//
    @Override
    public Motor.MotorTypes getType() {
        return Motor.MotorTypes.FALCON;
    }
    @Override
    public void setMotorType(Motor.MotorTypes _type) {}

    //----------------------------------------------Inverted----------------------------------------------------------//
    @Override
    public boolean inverted() {
        return inverted;
    }
    @Override
    public void setInverted(boolean _inverted) {
        super.setInverted(_inverted);
        inverted = _inverted;
    }

    //--------------------------------------------Current Limit-------------------------------------------------------//
    @Override
    public int getCurrentLimit() {
        return currentLimit;
    }
    @Override
    public void setCurrentLimit(int limit) {
        super.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, limit, limit, 0));
        currentLimit = limit;
    }
    @Override
    public ErrorCode configSupplyCurrentLimit(SupplyCurrentLimitConfiguration limit) {
        currentLimit = (int) limit.currentLimit;
        return super.configSupplyCurrentLimit(limit);
    }

    //------------------------------------------------PID-------------------------------------------------------------//
    @Override
    public PID getPID(int slotID) {
        return pidValues[slotID];
    }
    @Override
    public void setPID(int soltID, PID pid) {
        pidValues[soltID] = pid;
    }

    @Override
    public PID getPID() {
        return pidValues[0];
    }
    @Override
    public void setPID(PID pid) {
        pidValues[0] = pid;
        super.config_kP(0, pid.getP());
        super.config_kI(0, pid.getI());
        super.config_kD(0, pid.getD());
        super.config_kF(0, pid.getFF());
    }

    @Override
    public ErrorCode config_kP(int slotIdx, double p) {
        pidValues[slotIdx].setP(p);
        return super.config_kP(slotIdx, p);
    }
    @Override
    public ErrorCode config_kI(int slotIdx, double i) {
        pidValues[slotIdx].setI(i);
        return super.config_kI(slotIdx, i);
    }
    @Override
    public ErrorCode config_kD(int slotIdx, double d) {
        pidValues[slotIdx].setP(d);
        return super.config_kD(slotIdx, d);
    }
    @Override
    public ErrorCode config_kF(int slotIdx, double f) {
        pidValues[slotIdx].setFF(f);
        return super.config_kF(slotIdx, f);
    }

    //---------------------------------------------Ramp Rate----------------------------------------------------------//
    @Override
    public double getRampRate() {
        return rampRate;
    }
    @Override
    public void setRampRate(double _rampRate) {
        super.configClosedloopRamp(_rampRate);
        rampRate = _rampRate;
    }
    @Override
    public ErrorCode configClosedloopRamp(double _rampRate) {
        rampRate = _rampRate;
        return super.configClosedloopRamp(_rampRate);
    }

    //--

    /**
     * Gets the set name of the Entity
     *
     * @return - If no name is present return "Default"
     */
    @Override
    public String getName() {
        return name;
    }

    @Override
    public void constructTreeItemPrintout(StringBuilder builder, int depth) {
        Entity.buildStringTabbedData(builder, depth, "Name", name);
        Entity.buildStringTabbedData(builder, depth, "ID", String.valueOf(getID()));
        Entity.buildStringTabbedData(builder, depth, "Type", type.toString());
        Entity.buildStringTabbedData(builder, depth, "Inverted", String.valueOf(inverted));
        Entity.buildStringTabbedData(builder, depth, "Current Limit", String.valueOf(currentLimit));
        Entity.buildStringTabbedData(builder, depth, "Gear Ratio", String.valueOf(gearRatio));
        Entity.buildStringTabbedData(builder, depth, "Ramp Rate", String.valueOf(rampRate));

        for(int i = 0; i < NUMBEROFPIDSLOTS; i++) {
            if(pidValues[i] != null)
                pidValues[i].constructTreeItemPrintout(builder, depth + 1);
        }
    }

    @Override
    public void setOnImplementCallback(Runnable run) {
        onImplement = run;
    }

    @Override
    public Runnable getOnImplementCallback() {
        return onImplement;
    }

    @Override
    public Element updateElement() {
        getSavedElement().getElementsByTagName("Name").item(0).setTextContent(name);
        getSavedElement().getElementsByTagName("ID").item(0).setTextContent(String.valueOf(getID()));
        getSavedElement().getElementsByTagName("GearRatio").item(0).setTextContent(String.valueOf(gearRatio));
        getSavedElement().getElementsByTagName("Type").item(0).setTextContent(type.toString());
        getSavedElement().getElementsByTagName("Inverted").item(0).setTextContent(String.valueOf(inverted));
        getSavedElement().getElementsByTagName("CurrentLimit").item(0).setTextContent(String.valueOf(currentLimit));
        getSavedElement().getElementsByTagName("RampRate").item(0).setTextContent(String.valueOf(rampRate));

        for (PID pid : pidValues) {
            if(pid != null) pid.updateElement();
        }

        return getSavedElement();
    }

    @Override
    public void OnImplement() {
        onImplement.run();
    }

    @Override
    public NetworkTable addToNetworkTable(NetworkTable dashboard) {
        NetworkTable table = dashboard.getSubTable(getName());

        //TODO read PID
        for (PID pid : pidValues) {
            if(pid != null) pid.addToNetworkTable(getCurrentNetworkInstance());
        }

        NetworkTableEntry entryGearRatio = table.getEntry("GearRatio");
        entryGearRatio.setDouble(gearRatio);

        NetworkTableEntry entryRampRate = table.getEntry("RampRate");
        entryRampRate.setDouble(rampRate);

        NetworkTableEntry entryCurrentLimit = table.getEntry("CurrentLimit");
        entryCurrentLimit.setDouble(currentLimit);

        NetworkTableEntry entryInverted = table.getEntry("Inverted");
        entryInverted.setBoolean(inverted);

        //None mutable
        NetworkTableEntry entryType = table.getEntry("Type");
        entryType.setString(type.toString());

        NetworkTableEntry entryID = table.getEntry("ID");
        entryID.setDouble(getID());

        NetworkTableEntry entryName = table.getEntry("Name");
        entryName.setString(getName());

        return table;
    }

    @Override
    public NetworkTable pullFromNetworkTable() {
        NetworkTable table = getCurrentNetworkInstance();

        //TODO add type
        name = table.getEntry("Name").getString(getName());
        setInverted(table.getEntry("Inverted").getBoolean(inverted()));
        setID((int) table.getEntry("ID").getDouble(getID()));
        setCurrentLimit((int) table.getEntry("CurrentLimit").getDouble(getCurrentLimit())); //TODO convert current limit to double
        setRampRate(table.getEntry("RampRate").getDouble(getRampRate()));
        setGearRatio(table.getEntry("GearRatio").getDouble(getGearRatio()));

        for(PID pid : pidValues) {
            if(pid != null) pid.pullFromNetworkTable();
        }

        return table;
    }

    @Override
    public NetworkTable removeFromNetworkTable() {
        NetworkTable table = getCurrentNetworkInstance();

        table.getEntry("Name").unpublish();
        table.getEntry("Inverted").unpublish();
        table.getEntry("ID").unpublish();
        table.getEntry("CurrentLimit").unpublish();
        table.getEntry("RampRate").unpublish();
        table.getEntry("GearRatio").unpublish();
        table.getEntry("Type").unpublish();

        return table;
    }

    @Override
    public NetworkTable getCurrentNetworkInstance() {
        return savedNetworkTableInstance;
    }

    @Override
    public void setCurrentNetworkInstance(NetworkTable instance) {
        savedNetworkTableInstance = instance;
    }

    /**
     * Returns the linked Element object
     *
     * @return - Linked Element object
     */
    @Override
    public Element getSavedElement() {
        return savedElement;
    }
}
