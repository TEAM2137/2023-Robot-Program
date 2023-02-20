//              Copyright 2022 Wyatt Ashley
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package frc.robot.functions.io.xmlreader;

import edu.wpi.first.networktables.NetworkTable;
import frc.robot.Robot;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.data.Binding;
import frc.robot.functions.io.xmlreader.data.PID;
import frc.robot.functions.io.xmlreader.data.mappings.CANifierMapping;
import frc.robot.functions.io.xmlreader.data.mappings.ControllerMapping;
import frc.robot.functions.io.xmlreader.data.Step;
import frc.robot.functions.io.xmlreader.data.mappings.DIOMapping;
import frc.robot.functions.io.xmlreader.objects.canifier.DIO;
import frc.robot.functions.io.xmlreader.objects.motor.FalconMotor;
import frc.robot.functions.io.xmlreader.objects.motor.NeoMotor;
import frc.robot.functions.io.xmlreader.objects.solenoid.DoubleSolenoid;
import frc.robot.library.hardware.LazySusan;
import frc.robot.library.hardware.elevators.StringElevator;
import frc.robot.library.units.Number;
import frc.robot.functions.io.xmlreader.data.Threshold;
import frc.robot.functions.io.xmlreader.objects.Camera;
import frc.robot.functions.io.xmlreader.objects.Encoder;
import frc.robot.functions.io.xmlreader.objects.Gyro;
import frc.robot.functions.io.xmlreader.objects.motor.Motor;
import frc.robot.library.hardware.swerve.SwerveDrivetrain;
import frc.robot.library.hardware.swerve.module.SwerveFALCONDriveModule;
import frc.robot.library.hardware.swerve.module.SwerveNEODriveModule;
import frc.robot.library.hardware.swerve.module.SwerveSimulationDriveModule;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import java.lang.reflect.InvocationTargetException;
import java.util.*;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

/**
 * EntityGroup is a class implemented by the XMLSettings reader and acts like a container for all of the
 * child EntityGroups and also devices its self.
 *
 * Created By... Wyatt Ashley 1/19/2022
 */
public class EntityGroup extends EntityImpl {

    private final String type; //Type of EntityGroup (Intake, Climber, etc.)
    private final ArrayList<Entity> childEntities = new ArrayList<>(); //Child devices
    private final HashMap<String, EntityGroup> childSubsystem = new HashMap<>(); //Child device groups

    private String entityPath = "root";

    private FileLogger logger;

    public static HashMap<String, HashMap<String, Class<? extends EntityGroup>>> entityGroupClassMappings = new HashMap<>();
    public static HashMap<String, HashMap<String, Class<? extends Entity>>> entityClassMappings = new HashMap<>();

    static {
        //Only single encapsulation is supported right now
        HashMap<String, Class<? extends EntityGroup>> swerveModuleImplements = new HashMap<>();
        swerveModuleImplements.put("NEO", SwerveNEODriveModule.class);
        swerveModuleImplements.put("FALCON", SwerveFALCONDriveModule.class);
        swerveModuleImplements.put("SIMULATION", SwerveSimulationDriveModule.class);
        entityGroupClassMappings.put("SWERVEMODULE", swerveModuleImplements);

        HashMap<String, Class<? extends EntityGroup>> elevatorImplements = new HashMap<>();
        elevatorImplements.put("STRING", StringElevator.class);
        entityGroupClassMappings.put("ELEVATOR", elevatorImplements);

        HashMap<String, Class<? extends EntityGroup>> lazySusanImplements = new HashMap<>();
        lazySusanImplements.put("DEFAULT", LazySusan.class);
        entityGroupClassMappings.put("LazySusan", lazySusanImplements);

        HashMap<String, Class<? extends EntityGroup>> bindingGroup = new HashMap<>();
        bindingGroup.put("DEFAULT", Binding.class);
        entityGroupClassMappings.put("BINDING", bindingGroup);

        HashMap<String, Class<? extends EntityGroup>> driveTrainImplements = new HashMap<>();
        driveTrainImplements.put("SWERVE", SwerveDrivetrain.class);
        entityGroupClassMappings.put("DRIVETRAIN", driveTrainImplements);

        //---------------------------------------------Entity Types---------------------------------------------------//
        HashMap<String, Class<? extends Entity>> usbCameraImplements = new HashMap<>();
        usbCameraImplements.put("DEFAULT", Camera.class);
        entityClassMappings.put("USBCAMERA", usbCameraImplements);

        HashMap<String, Class<? extends Entity>> encoderImplements = new HashMap<>();
        encoderImplements.put("DEFAULT", Encoder.class);
        entityClassMappings.put("ENCODER", encoderImplements);

        HashMap<String, Class<? extends Entity>> solenoidImplements = new HashMap<>();
        solenoidImplements.put("DEFAULT", DoubleSolenoid.class);
        entityClassMappings.put("DOUBLESOLENOID", solenoidImplements);

        HashMap<String, Class<? extends Entity>> dioImplements = new HashMap<>();
        dioImplements.put("DEFAULT", DIO.class);
        entityClassMappings.put("DIO", dioImplements);

        HashMap<String, Class<? extends Entity>> gyroImplements = new HashMap<>();
        gyroImplements.put("DEFAULT", Gyro.class);
        entityClassMappings.put("GYRO", gyroImplements);

        HashMap<String, Class<? extends Entity>> motorImplements = new HashMap<>();
        motorImplements.put("DEFAULT", Motor.class);
        motorImplements.put("FALCON", FalconMotor.class);
        motorImplements.put("NEO", NeoMotor.class);
        entityClassMappings.put("MOTOR", motorImplements);

        HashMap<String, Class<? extends Entity>> thresholdImplements = new HashMap<>();
        thresholdImplements.put("DEFAULT", Threshold.class);
        entityClassMappings.put("THRESHOLD", thresholdImplements);

        HashMap<String, Class<? extends Entity>> numberImplements = new HashMap<>();
        numberImplements.put("DEFAULT", Number.class);
        entityClassMappings.put("NUMBER", numberImplements);

        HashMap<String, Class<? extends Entity>> pidImplements = new HashMap<>();
        pidImplements.put("DEFAULT", PID.class);
        entityClassMappings.put("PID", pidImplements);

        HashMap<String, Class<? extends Entity>> mapImplements = new HashMap<>();
        mapImplements.put("DEFAULT", ControllerMapping.class);
        mapImplements.put("CONTROLLER", ControllerMapping.class);
        mapImplements.put("DIO", DIOMapping.class);
        mapImplements.put("CANIFIER", CANifierMapping.class);
        entityClassMappings.put("MAP", mapImplements);

        HashMap<String, Class<? extends Entity>> stepImplements = new HashMap<>();
        stepImplements.put("DEFAULT", Step.class);
        entityClassMappings.put("STEP", stepImplements);
    }

    public EntityGroup(String _name, String _type) {
        super(_name);
        type = _type;
        Robot.subSystemCallList.add(this);
    }

    /**
     * Takes in a part of the xml file and parses it into variables and subtypes using recursion
     * @param element - Portion of the XML File
     */
    public EntityGroup(Element element, EntityGroup parent, FileLogger fileLogger) {
        super(element);
        type = element.getTagName(); //Record the Tag Name or the Type
        logger = fileLogger;

        Robot.subSystemCallList.add(this);

        if(parent != null)
            entityPath = parent.entityPath + "/" + getName();

        findEntities(element, logger);
    }

    public Class<? extends EntityGroup> createEntityGroup(Element tmp) {
        if(tmp.hasAttribute("type"))
            return entityGroupClassMappings.get(tmp.getTagName().toUpperCase()).get(tmp.getAttribute("type").toUpperCase());
        else if (entityGroupClassMappings.containsKey(tmp.getTagName().toUpperCase()))
            return entityGroupClassMappings.get(tmp.getTagName().toUpperCase()).get("default".toUpperCase());
        else
            return EntityGroup.class;
    }

    public Class<? extends Entity> createEntity(Element tmp) {
        if(tmp.hasAttribute("type"))
            return entityClassMappings.get(tmp.getTagName().toUpperCase()).get(tmp.getAttribute("type").toUpperCase());
        else if (entityClassMappings.containsKey(tmp.getTagName().toUpperCase()))
            return entityClassMappings.get(tmp.getTagName().toUpperCase()).get("default".toUpperCase());
        else
            return Entity.class;
    }

    protected void findEntities(Node element, FileLogger fileLogger) {
        NodeList list = element.getChildNodes();

        if (list.getLength() > 1) {
            for(int i = 0; i < list.getLength(); i++) {
                if(list.item(i).getNodeType() == Node.ELEMENT_NODE) {
                    Element tmp = (Element) list.item(i);

                    if(entityClassMappings.containsKey(tmp.getTagName().toUpperCase())) {
                        Class<? extends Entity> entityClass = createEntity(tmp);

                        try {
                            Entity returner = entityClass.getDeclaredConstructor(Element.class).newInstance(tmp);

                            childEntities.add(returner);
                        } catch (InvocationTargetException invocationTargetException) {
                            invocationTargetException.printStackTrace();
                            fileLogger.writeLine(invocationTargetException.getTargetException().getMessage());
                        } catch (NoSuchMethodException e) {
                            fileLogger.writeLine("Could not find constructor for " + tmp.getTagName() + " device");
                        } catch (Exception e) {
                            fileLogger.writeLine("An exception occurred in the Entity's constructor");
                            fileLogger.writeLine(e.toString());
                            e.printStackTrace();
                            fileLogger.writeLine("Proceeding with out...");
                        }
                    } else {
                        Class<? extends EntityGroup> entityGroupClass = createEntityGroup(tmp);

                        try {
                            if(entityGroupClass == null) {
                                logger.writeEvent(0, FileLogger.EventType.Error, tmp.getTagName());
                            }

                            EntityGroup returner = entityGroupClass.getDeclaredConstructor(Element.class, EntityGroup.class, FileLogger.class).newInstance(tmp, this, logger);

                            childSubsystem.put(returner.getName().toUpperCase(), returner);
                        } catch (NoSuchMethodException e) {
                            fileLogger.writeLine("Could not find constructor for " + tmp.getTagName() + " device");
                        } catch (Exception e) {
                            fileLogger.writeLine("An exception occurred in the EntityGroup's constructor");
                            fileLogger.writeLine(e.toString());
                            e.printStackTrace();
                            fileLogger.writeLine("Proceeding with out...");
                        }
                    }
                }
            }
        }
    }

    @Override
    public void constructTreeItemPrintout(StringBuilder builder, int depth) {
        super.constructTreeItemPrintout(builder, depth);

        childEntities.forEach((b) -> b.constructTreeItemPrintout(builder, depth + 1));
        childSubsystem.forEach((a, b) -> b.constructTreeItemPrintout(builder, depth + 1));
    }

    /**
     * Add a child device to this deviceGroup
     * @param _device - An child class of Entity type
     */
    public void addEntity(Entity _device) {
        childEntities.add(_device);
    }

    /**
     * Retrieve all the child device groups
     * @return - HashMap with Name then EntityGroup Object
     */
    public HashMap<String, EntityGroup> getChildEntityGroups() {
        return childSubsystem;
    }

    /**
     * Gets ALL groups that are a certain type
     * @param type - Type to find (Tag Name on XML)
     * @return - ArrayList of matching Groups
     */
    public ArrayList<EntityGroup> getEntityGroupsByType(String type) {
        ArrayList<EntityGroup> returningList = new ArrayList<>();

        for (Map.Entry<String, EntityGroup> sub : childSubsystem.entrySet()) {
            if (sub.getValue().getGroupType().equals(type)) {
                returningList.add(sub.getValue());
            }
        }

        return returningList;
    }

    /**
     * Same as getEntityGroupsByType but returns only one result
     * @param type - Type to search for
     * @return - First Match found
     */
    public EntityGroup getEntityGroupByType(String type) {
        for (Map.Entry<String, EntityGroup> sub : childSubsystem.entrySet()) {
            if (sub.getValue().getGroupType().equalsIgnoreCase(type)) {
                return sub.getValue();
            }
        }

        return null;
    }

    /**
     * Gets this groups type
     * @return - String of type (Tag Name of XML)
     */
    public String getGroupType() {
        return type;
    }

    /**
     * Returns a device based on a name
     * @param name - String name of the entity
     * @return - Entity type
     */
    public Entity getEntity(String name) {
        name = name.toUpperCase();

        for (Entity entity : childEntities) {
            if(entity.getName().equalsIgnoreCase(name))
                return entity;
        }

        return null;
    }

    public ArrayList<Entity> getEntities() {
        return childEntities;
    }

    /**
     * Find all the Child EntityGroups that have a certain name and type
     * @param name - Name of Child EntityGroup
     * @param type - Type of Child EntityGroup (Tag Name in XML)
     * @return - Returns first EntityGroup found (null if not found)
     */
    public EntityGroup getChildEntityGroup(String name, String type) {
        name = name.toUpperCase();

        if(childSubsystem.containsKey(name) && childSubsystem.get(name).getGroupType().equals(type))
            return childSubsystem.get(name);
        else
            System.out.println("Did not find " + name + " subsystem");

        return null;
    }

    /**
     * Find a Child EntityGroup that has a certain name only
     * @param name - Name of Child EntityGroup
     * @return - Single EntityGroup with given name (null if not found)
     */
    public EntityGroup getChildEntityGroup(String name) {
        name = name.toUpperCase(); //Move to all uppercase

        if(childSubsystem.containsKey(name))
            return childSubsystem.get(name);
        else
            System.out.println("Did not find " + name + " subsystem");

        return null;
    }

    /**
     * Add a child EntityGroup to this current group
     * @param deviceGroup - EntityGroup to add
     */
    public void addEntityGroup(EntityGroup deviceGroup) {
        childSubsystem.put(deviceGroup.getName().toUpperCase(), deviceGroup);
    }

    /**
     * Function that runs over all child device groups (IS NOT RECURSIVE)
     * @param consumer - Function to run (a, b) -> {}
     */
    public void forEachSubSystem(BiConsumer<String, EntityGroup> consumer) {
        getChildEntityGroups().forEach(consumer);
    }

    @Override
    public NetworkTable addToNetworkTable(NetworkTable instance) {
        NetworkTable subInstance;
        subInstance = instance.getSubTable(getName());

        childEntities.forEach((b) -> b.addToNetworkTable(subInstance));
        childSubsystem.forEach((a, b) -> b.addToNetworkTable(subInstance));

        return subInstance;
    }

    @Override
    public NetworkTable removeFromNetworkTable() {
        NetworkTable subInstance = super.removeFromNetworkTable();

        childEntities.forEach(Entity::removeFromNetworkTable);
        childSubsystem.forEach((a, b) -> b.removeFromNetworkTable());

        return subInstance;
    }

    /**
     * Returns the linked Element object
     *
     * @return - Linked Element object
     */
    @Override
    public Element getSavedElement() {
        return null;
    }

    @Override
    public NetworkTable pullFromNetworkTable() {
        NetworkTable subInstance = super.pullFromNetworkTable();

        childEntities.forEach(Entity::pullFromNetworkTable);
        childSubsystem.forEach((a, b) -> b.pullFromNetworkTable());

        return subInstance;
    }

    @Override
    public Element updateElement() {
        super.updateElement();

        childEntities.forEach(Entity::updateElement);
        childSubsystem.forEach((a, b) -> b.updateElement());

        return getSavedElement();
    }

    @Override
    public void OnImplement() {
        super.OnImplement();

        childEntities.forEach((b) -> b.OnImplement());
        childSubsystem.forEach((a, b) -> b.OnImplement());
    }

    public String getEntityPath() {
        return entityPath + "/";
    }

    public void addSubsystemCommand(String name, Consumer<Step> command) {
        Robot.subSystemCommandList.put(name, command);
    }

    public void periodic() {
//        Robot.subSystemCallList.remove(this);
    }
}
