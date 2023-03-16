package frc.robot.functions.io.xmlreader.objects.motor;

import com.revrobotics.*;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.functions.io.xmlreader.Entity;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.data.PID;
import frc.robot.functions.io.xmlreader.objects.Encoder;
import frc.robot.library.units.AngleUnits.Angle;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.TranslationalUnits.Velocity;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import java.sql.Driver;

import static com.revrobotics.CANSparkMax.ControlType.kVelocity;
import static frc.robot.library.units.AngleUnits.Angle.AngleUnits.REVOLUTIONS;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.FOOT;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.INCH;

public class NeoMotor extends CANSparkMax implements Entity, SimpleMotorControl {

    private NetworkTable savedNetworkTableInstance;
    private final org.w3c.dom.Element savedElement;
    private String name;
    private Runnable onImplement;

    public static final int NUMBEROFPIDSLOTS = 2;

    private int id = 0;
    private final Motor.MotorTypes type = Motor.MotorTypes.NEO;
    private boolean inverted = false;
    private int currentLimit = 0;
    private double gearRatio = 0;
    private double rampRate = 0;
    private final PID[] pidValues = new PID[NUMBEROFPIDSLOTS];
    private IdleMode idleMode = IdleMode.kBrake;

    private RelativeEncoder relativeEncoder;
    private AbsoluteEncoder absoluteEncoder;
    private final SparkMaxPIDController pidController;

    private Distance distancePerRevolution;

    private static final int CountsPerRevolution = 1;

    public NeoMotor(Element element) {
        super(Integer.parseInt(Entity.getOrDefault(element, "ID", "0")), MotorType.kBrushless);
        savedElement = element;

        name = getNodeOrAttribute(element, "Name", "none");

        restoreFactoryDefaults();

        NodeList tmpEncoderList = element.getElementsByTagName("Encoder");

        if(tmpEncoderList.getLength() > 0) {
            Element encoderElement = (Element) tmpEncoderList.item(0);
            Encoder coder = new Encoder(encoderElement);

            setGearRatio(1);

            if(coder.isAbsolute()) {
                DriverStation.reportWarning("Created Coder for Neo Motor ABS", false);
                absoluteEncoder = getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
                absoluteEncoder.setInverted(coder.inverted());
                absoluteEncoder.setZeroOffset(coder.getOffset() / 360.0);
            } else {
                relativeEncoder = getEncoder(coder.getEncoderType().getType(), coder.getCPR());
            }
        } else {
            relativeEncoder = getEncoder();
            setGearRatio(Double.parseDouble(Entity.getOrDefault(element, "GearRatio", "1")));
        }

        pidController = getPIDController();

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

        PID pid = getPID();
        pidController.setP(pid.getP());
        pidController.setI(pid.getI());
        pidController.setD(pid.getD());
        pidController.setFF(pid.getFF());

        if(absoluteEncoder != null) {
            pidController.setOutputRange(-1, 1);
            pidController.setPositionPIDWrappingEnabled(true);
            pidController.setPositionPIDWrappingMinInput(0);
            pidController.setPositionPIDWrappingMaxInput(360);
            pidController.setFeedbackDevice(absoluteEncoder);

            super.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 100);
        } else {
            pidController.setFeedbackDevice(relativeEncoder);
        }

        setID(Integer.parseInt(Entity.getOrDefault(element, "ID", "0")));
        setMotorType(Motor.MotorTypes.valueOf(Entity.getOrDefault(element, "Type", "FALCON").toUpperCase()));
        setInverted(Boolean.parseBoolean(Entity.getOrDefault(element, "Inverted", "false").toLowerCase()));
        setCurrentLimit(Integer.parseInt(Entity.getOrDefault(element, "CurrentLimit", "-1")));
        setRampRate(Double.parseDouble(Entity.getOrDefault(element, "RampRate", "0")));

        super.setClosedLoopRampRate(rampRate);

        if(Entity.getOrDefault(element, "IdleMode", "Brake").equalsIgnoreCase("Brake")) {
            setNeutralMode(IdleMode.kBrake);
        } else {
            setNeutralMode(IdleMode.kCoast);
        }
    }

    //---------------------------------------------------Soft Limit---------------------------------------------------//
    @Override
    public void configureForwardLimit(Distance d) {
        //DriverStation.reportWarning(getName() + "-SoftLimit" + ((float) ((d.getValue(INCH) / distancePerRevolution.getValue(INCH)) * getGearRatio() * getCountPerRevolution()), false);
        super.enableSoftLimit(SoftLimitDirection.kForward, true);
        super.setSoftLimit(SoftLimitDirection.kForward, (float) ((d.getValue(INCH) / distancePerRevolution.getValue(INCH)) * getGearRatio() * getCountPerRevolution()));
    }
    @Override
    public void enableForwardLimit() {
        super.enableSoftLimit(SoftLimitDirection.kForward, true);
    }
    @Override
    public void disableForwardLimit() {
        super.enableSoftLimit(SoftLimitDirection.kForward, false);
    }

    @Override
    public void configureReverseLimit(Distance d) {
        super.enableSoftLimit(SoftLimitDirection.kReverse, true);
        super.setSoftLimit(SoftLimitDirection.kReverse, (float) ((d.getValue(INCH) / distancePerRevolution.getValue(INCH)) * getGearRatio() * getCountPerRevolution()));
    }
    @Override
    public void enableReverseLimit() {
        super.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }
    @Override
    public void disableReverseLimit() {
        super.enableSoftLimit(SoftLimitDirection.kReverse, false);
    }

    //---------------------------------------------------Set & Get----------------------------------------------------//

    @Override
    public void set(double val) {
        super.set(val);
    }
    @Override
    public double get() {
        return super.getAppliedOutput();
    }

    //------------------------------------------Position Control------------------------------------------------------//
    @Override
    public void setPosition(Distance distance) {
        pidController.setReference((distance.getValue(INCH) / distancePerRevolution.getValue(INCH)) * getGearRatio(), CANSparkMax.ControlType.kPosition);
    }
    public void setPosition(Angle angle, double feedforward) {
        pidController.setReference((angle.getValue(Angle.AngleUnits.DEGREE) / 360.0) * getGearRatio(), ControlType.kPosition, 0, feedforward, SparkMaxPIDController.ArbFFUnits.kPercentOut);
    }
    @Override
    public void setPosition(Angle angle) {
        pidController.setReference((angle.getValue(Angle.AngleUnits.DEGREE) / 360.0) * getGearRatio(), ControlType.kPosition);
    }
    @Override
    public Distance getPosition() {
        return new Distance(relativeEncoder.getPosition() / getGearRatio() * distancePerRevolution.getValue(INCH), INCH);
    }
    @Override
    public Angle getAnglePosition() {
        if(relativeEncoder != null) {
            return new Angle(relativeEncoder.getPosition() / getGearRatio() / getCountPerRevolution(), REVOLUTIONS);
        } else {
            SmartDashboard.putNumber(getName() + "-TestOutput", getGearRatio());
            return new Angle(absoluteEncoder.getPosition() / getGearRatio() / getCountPerRevolution(), REVOLUTIONS);
        }
    }

    //------------------------------------------Velocity Control------------------------------------------------------//
    @Override
    public void setVelocity(Velocity velocity) {
        pidController.setReference((velocity.getValue(Velocity.VelocityUnits.FEET_PER_SECOND) / distancePerRevolution.getValue(FOOT)) * getGearRatio(), kVelocity);
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

    //-----------------------------------------------Integrated Sensor------------------------------------------------//

    @Override
    public void setIntegratedSensorPosition(double val) {
        if(relativeEncoder != null)
            relativeEncoder.setPosition(val);
        else
            absoluteEncoder.setZeroOffset(val / 360);
    }
    @Override
    public void setIntegratedSensorDistance(Distance dis) {
        relativeEncoder.setPosition(dis.getValue(INCH) * distancePerRevolution.getValue(INCH) * getGearRatio() * getCountPerRevolution());
    }

    //-----------------------------------------------------CAN ID-----------------------------------------------------//

    @Override
    public void setID(int _id) {
        id = _id;
    }
    @Override
    public int getID() {
        return id;
    }

    //---------------------------------------------------Idle Mode----------------------------------------------------//

    @Override
    public void setNeutralMode(IdleMode mode) {
        super.setIdleMode(IdleMode.kBrake);
        idleMode = mode;
    }

    //-----------------------------------------------------Follow-----------------------------------------------------//

    @Override
    public void follow(SimpleMotorControl other) {
        if(other instanceof NeoMotor) {
            DriverStation.reportWarning("Successfully made follow relationship NEO: " + ((NeoMotor) other).id, false);
            super.follow(((CANSparkMax) other));
        } else if(other instanceof FalconMotor){
            super.follow(ExternalFollower.kFollowerPhoenix, other.getID());
        } else {
            DriverStation.reportWarning("Could not make follow relationship", true);
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
        return Motor.MotorTypes.NEO;
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
        super.setSmartCurrentLimit(limit);
        currentLimit = limit;
    }
    @Override
    public REVLibError setSmartCurrentLimit(int limit) {
        currentLimit = limit;
        return super.setSmartCurrentLimit(limit);
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
        pidController.setP(pid.getP());
        pidController.setI(pid.getI());
        pidController.setD(pid.getD());
        pidController.setFF(pid.getFF());
    }

    //---------------------------------------------Ramp Rate----------------------------------------------------------//
    @Override
    public double getRampRate() {
        return rampRate;
    }
    @Override
    public void setRampRate(double _rampRate) {
        super.setClosedLoopRampRate(_rampRate);
        rampRate = _rampRate;
    }
    @Override
    public REVLibError setClosedLoopRampRate(double rate) {
        rampRate = rate;
        return super.setClosedLoopRampRate(rate);
    }
    @Override
    public REVLibError setOpenLoopRampRate(double rate) {
        rampRate = rate;
        return super.setOpenLoopRampRate(rate);
    }

    @Override
    public double getOutputAmperage() {
        return super.getOutputCurrent();
    }

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
    public void setOnImplementCallback(Runnable run) {
        onImplement = run;
    }

    @Override
    public Runnable getOnImplementCallback() {
        return onImplement;
    }

    @Override
    public NetworkTable getCurrentNetworkInstance() {
        return savedNetworkTableInstance;
    }

    @Override
    public void setCurrentNetworkInstance(NetworkTable instance) {
        savedNetworkTableInstance = instance;
    }

    @Override
    public NetworkTable addToNetworkTable(NetworkTable dashboard) {
        NetworkTable table = dashboard.getSubTable(getName());

        //TODO read PID
        for (PID pid : pidValues) {
            if(pid != null) pid.addToNetworkTable(table);
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
