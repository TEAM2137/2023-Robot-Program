package frc.robot.functions.io.xmlreader.objects.motor;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.functions.io.xmlreader.Entity;
import frc.robot.functions.io.xmlreader.data.PID;
import frc.robot.library.units.AngleUnits.Angle;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.TranslationalUnits.Velocity;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import static com.revrobotics.CANSparkMax.ControlType.kVelocity;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.FOOT;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.INCH;

public class NeoMotor extends CANSparkMax implements Entity, SimpleMotorControl {

    private NetworkTable savedNetworkTableInstance;
    private final org.w3c.dom.Element savedElement;
    private final String name;
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

    private final RelativeEncoder relativeEncoder;
    private final SparkMaxPIDController pidController;

    private Distance distancePerRevolution;

    private static final int CountsPerRevolution = 2048;

    public NeoMotor(Element element) {
        super(Integer.parseInt(Entity.getOrDefault(element, "ID", "0")), MotorType.kBrushless);
        savedElement = element;

        name = getNodeOrAttribute(element, "Name", "none");

        restoreFactoryDefaults();

        relativeEncoder = getEncoder();

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

        setID(Integer.parseInt(Entity.getOrDefault(element, "ID", "0")));
        setMotorType(Motor.MotorTypes.valueOf(Entity.getOrDefault(element, "Type", "FALCON").toUpperCase()));
        setInverted(Boolean.parseBoolean(Entity.getOrDefault(element, "Inverted", "false").toLowerCase()));
        setCurrentLimit(Integer.parseInt(Entity.getOrDefault(element, "CurrentLimit", "-1")));
        setGearRatio(Double.parseDouble(Entity.getOrDefault(element, "GearRatio", "1")));
        setRampRate(Double.parseDouble(Entity.getOrDefault(element, "RampRate", "0")));
    }

    @Override
    public void set(double val) {
        super.set(val);
    }

    //------------------------------------------Position Control------------------------------------------------------//
    @Override
    public void setPosition(Distance distance) {
        pidController.setReference((distance.getValue(INCH) / distancePerRevolution.getValue(INCH)) * getGearRatio(), CANSparkMax.ControlType.kPosition);
    }
    @Override
    public void setPosition(Angle angle) {
        pidController.setReference((angle.getValue(Angle.AngleUnits.DEGREE) / 360.0) * getGearRatio(), ControlType.kPosition);
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

    @Override
    public void setIntegratedSensorPosition(double val) {
        relativeEncoder.setPosition(val);
    }

    @Override
    public int getID() {
        return id;
    }

    @Override
    public void setNeutralMode(IdleMode mode) {
        super.setIdleMode(IdleMode.kBrake);
        idleMode = mode;
    }

    @Override
    public void setID(int _id) {
        id = _id;
    }

    @Override
    public void follow(SimpleMotorControl other) {
        if(other instanceof NeoMotor) {
            follow(((CANSparkMax) other));
        } else if(other instanceof FalconMotor){
            follow(ExternalFollower.kFollowerPhoenix, other.getID());
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
