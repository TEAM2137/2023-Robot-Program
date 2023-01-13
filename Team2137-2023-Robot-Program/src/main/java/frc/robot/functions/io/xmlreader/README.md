## Basics

The main idea is there are two different "Entities" in XML files. Which can be thought as blocks and containers.
  - Blocks &rarr; Entity --- People use blocks/entities to build structures and system
  - Container &rarr; EntityGroup --- Blocks can be sorted in groups like block on the front wall 


### Types of Entities

  - In the "data" folder there is entities that represent only data with regular numbers.
    - These are mainly for settings that are likely to change
    - They are mutable on the SmartDashboard icons

  - In the "objects" folder there is entities that represent actual objects
    - These are motors, encoders, etc.
    - Soft settings like gear ratio are mutable but ID is not


# XML Reader Types...
Motor.class is an object that holds all the values for an actual motor. The XMLSetting reader then populates the class. 
The Motor class 



### How to make a type...

This example is for a camera
```javascript
public class Camera extends Entity {
    
    //Must have - Elements are sections taken from the XML reading library
    public Camera(Element element) {
        super(element);
        //Store values
    }

    //Suggested but not required is a back up constructor
    public Camera(String name, int value1, int value2) {
        super(name);
        //Store values
    }

    //Getter and Setter methods

    //Function to update the XML file elements
    //It is called recursivly so call this function for stored Entitis
    @Override
    public Element updateElement() {
        super.updateElement();
        //Update items in XML file other than Name
        return getSavedElement();
    }

    //Function to display all of the values to the NetworkTable
    //It is called recursivly
    @Override
    public NetworkTable addToNetworkTable(NetworkTable dashboard, boolean mutable) {
        NetworkTable table = super.addToNetworkTable(dashboard, mutable);

        //Post all the values not from super on the Network Table

        return table;
    }

    @Override
    public void removeFromNetworkTable(NetworkTable instance) {
        NetworkTable table = instance.getSubTable(getName());
        //Remove the NetworkTable Listeners and disable editing
    }
}
```
After creating the type it must be registered in the EntityGroup.class in the EntityTypes enum. This is how the class is called when XML is read and the name MUST be in all caps.

