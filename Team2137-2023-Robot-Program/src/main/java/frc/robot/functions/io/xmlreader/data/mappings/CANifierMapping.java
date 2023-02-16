package frc.robot.functions.io.xmlreader.data.mappings;

import com.ctre.phoenix.CANifier;
import frc.robot.Robot;
import frc.robot.functions.io.xmlreader.Entity;
import frc.robot.functions.io.xmlreader.EntityImpl;
import frc.robot.functions.io.xmlreader.objects.canifier.CANifierObj;
import org.w3c.dom.Element;

import java.util.ArrayList;

public class CANifierMapping extends EntityImpl implements Mapping {

    private CANifierObj canifier;
    private CANifier.GeneralPin pin;
    private String mCANifierName;

    /**
     * Constructs a new Entity with Element linkage
     * Usage: Entity is the main type of the XML File and represents any of the elements whether it is hardware or
     * just a number
     *
     * @param element
     */
    public CANifierMapping(Element element) {
        super(element);

        mCANifierName = getNodeOrAttribute(element, "CANifierName", "none");
        pin = CANifier.GeneralPin.valueOf(getNodeOrAttribute(element, "id", "LIMF"));

        for(Entity entity : Robot.allEntities) {
            if(entity instanceof CANifierObj && entity.getName().equalsIgnoreCase(mCANifierName)){
                canifier = (CANifierObj) entity;
                break;
            }
        }
    }

    @Override
    public double getValue() {
        return 0;
    }

    @Override
    public boolean getBooleanValue() {
        return false;
    }

    @Override
    public boolean isBooleanValue() {
        return false;
    }

    @Override
    public String getPseudoName() {
        return null;
    }
}
