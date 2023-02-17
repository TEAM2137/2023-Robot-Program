# Elevator Subsystem Instructions

## XML Instructions

Create a motor instances with the naming convention of "Lift Motor x". In code this will automatically create a motor instance that is controllable no matter the type Falcon or Neo. The only change needed to switch motors is to change the "type" block to the correct motor version.

```xml
<Motor type="NEO">
    <Name>Lift Motor 1</Name>
    <ID>1</ID>
    <Type>NEO</Type>
    <Inverted>false</Inverted>
    <GearRatio>1.0</GearRatio>
    <CurrentLimit>120</CurrentLimit>
    <PID>
        <P>0.10</P>
        <I>0.0</I>
        <D>0.0</D>
        <FF>0.0</FF>
    </PID>
</Motor>
```

Inorder to home the Elevator a Map must be made to the sensor. An example using a CANifier is shown below. Note that the name for the Map MUST be HomingMap for it to work correctly.

```xml
<Map type="CANifier" id="LIMB" value="HomingMap" name="HomingMap"/>
```

Yet another nessiary function is setting the spool diameter so the elevator can go to the correct position. This is a basic Number element with the spool diameter in inches,

```xml
<Number name="SpoolDiameter">1.5</Number>
```

Creation done!!

```xml
<Elevator type="String" name="HorizontalElevator">
    <Number name="SpoolDiameter">1.5</Number>
    <Map type="CANifier" id="LIMB" value="HomingMap" name="HomingMap"/>
    
    <Motor type="NEO">
        <Name>Lift Motor 1</Name>
        <ID>1</ID>
        <Type>NEO</Type>
        <Inverted>false</Inverted>
        <GearRatio>1.0</GearRatio>
        <CurrentLimit>120</CurrentLimit>
        <PID>
            <P>0.1</P>
            <I>0.0</I>
            <D>0.0</D>
            <FF>0.0</FF>
        </PID>
    </Motor>
</Elevator>
```

## Control Instructions

To control the elevator there is three major functions: SetPosition, HomeElevator, SetSpeed. Below is the implementation of all three.

Here is an example of the SetPosition command. The naming convention for the step is "NAME-SetRawPosition". By adding the Raw key word this skip the auto added drive step to the (X, Y, R) position on the drive train. Furthermore only a single button Map is placed in the Binding parent because the OnClick event calls all steps contained in the Binding.

```xml
<Binding>
    <Map controller="0" id="AButton" value="A"/>

    <Step>
        <command>HorizontalElevator-SetRawPosition</command>
        <timeout>1000</timeout>
        <speed>1.0</speed>
        <parallel>False</parallel>
        <parm1>12</parm1> <Note>Position Value in Inches</Note>
        <parm2>0</parm2>
        <parm3>0</parm3>
        <parm4>0</parm4>
        <parm5>0</parm5>
        <parm6>0</parm6>
        <parm7>0</parm7>
    </Step>
</Binding>
```

For the driver to be able to override the set position there is a function to get input from the controller. Below is an example.

```xml
<Binding>
    <Map controller="1" id="LeftY" value="YValue"/>

    <Step>
        <command>HorizontalElevator-SetSpeed</command>
        <timeout>1000</timeout>
        <speed>1.0</speed>
        <parallel>False</parallel>
        <parm1><YValue/></parm1> <Note>Speed value -1 to 1</Note>
        <parm2>0.2</parm2> <Note>Deadband value</Note>
        <parm3>0</parm3>
        <parm4>0</parm4>
        <parm5>0</parm5>
        <parm6>0</parm6>
        <parm7>0</parm7>
    </Step>
</Binding>
```

Finally an example of a Homing command is...

```xml
<Binding>
    <Map controller="0" id="AButton" value="A"/>

    <Step>
        <command>HorizontalElevator-HomeElevator</command>
        <timeout>1000</timeout>
        <speed>1.0</speed>
        <parallel>False</parallel>
        <parm1>0</parm1>
        <parm2>0</parm2>
        <parm3>0</parm3>
        <parm4>0</parm4>
        <parm5>0</parm5>
        <parm6>0</parm6>
        <parm7>0</parm7>
    </Step>
</Binding>
```

