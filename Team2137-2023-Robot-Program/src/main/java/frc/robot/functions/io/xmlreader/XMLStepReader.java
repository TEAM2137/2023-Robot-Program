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

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.data.Step;
import frc.robot.library.Constants;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;

public class XMLStepReader {
    private final File stepFile;
    private final FileLogger logger;
    private final ArrayList<Step> steps = new ArrayList<>();
    private int currentStepCounter = -1;
    //private int currentSplinePrePullCounter = -1;

    public XMLStepReader(String dir, FileLogger _logger) {
        this.stepFile = new File(dir);
        this.logger = _logger;

        if(!this.stepFile.exists()) {
            logger.writeEvent(0, FileLogger.EventType.Warning, "XML File Specified Does Not Exist, Attempting to Create a Basic One");
            try {
                this.stepFile.createNewFile();

                FileWriter tmpFileWriter = new FileWriter(this.stepFile);
                tmpFileWriter.write("<?xml version=\"1.0\"?>\n<Steps>\n\t<Step>\n\t</Step>\n</Steps>");
                tmpFileWriter.flush();
                tmpFileWriter.close();
            } catch (IOException e) {
                logger.writeEvent(0, FileLogger.EventType.Error, "Failed to create basic xml file because...");
                logger.writeEvent(0, FileLogger.EventType.Error, e.getMessage());
            }
        }

        DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();

        Document doc = null;

        try { 
            DocumentBuilder db = dbf.newDocumentBuilder();  
            doc = db.parse(this.stepFile);  
            doc.getDocumentElement().normalize();  
        } catch(Exception e) {
            e.printStackTrace();
        }

        NodeList tmpStep = doc.getElementsByTagName("Step");

        for (int i = 0; i < tmpStep.getLength(); i++) {
            this.steps.add(parseSteps(tmpStep.item(i)));
        }
    }

    public List<Step> getSteps() {
        return this.steps;
    }

    public void addStep(Step a) {
        this.steps.add(a);
    }

    public Step pullNextStep() {
        return this.steps.get(++currentStepCounter);
    }

    public List<Step> pullNextSteps() {
        List<Step> returnList = new ArrayList<>();
        int size = steps.size();

        if(size - 1 == currentStepCounter)
            return returnList;

        returnList.add(pullNextStep());

        while(size > currentStepCounter + 1) {
            if(!steps.get(currentStepCounter + 1).isParallel()) {//&& !steps.get(currentStepCounter).getCommand().equalsIgnoreCase("drive")) {
                break;
            }
            returnList.add((pullNextStep()));
        }

        return returnList;
    }

    public boolean isLastDriveStep() {
        return !hasFutureDriveStep();
    }

    public boolean isFirstDriveStep() {
        return getLastDriveStep() == null && isDriveStep(steps.get(currentStepCounter).getCommand());
//        return getLastDriveStep() == null;
    }

    public Step getLastDriveStep() {
        for(int i = getIndexLastSteps(); i >= 0; i--) {

            if(isDriveStep(steps.get(i).getCommand()))
                return steps.get(i);
        }

        return null;
    }

    public boolean lastStepsContainsDrive() {
        for(int i = getIndexLastSteps(); i >= 0; i--) {
            Step currentStep = steps.get(i);

            if(isDriveStep(currentStep.getCommand())) {
                return true;
            }

            if(!currentStep.isParallel()) {
                break;
            }
        }

        return false;
    }

    public boolean hasFutureDriveStep() {
        for(int i = currentStepCounter + 1; i < steps.size(); i++) {
            if(isDriveStep(steps.get(i).getCommand()))
                return true;
        }

        return false;
    }

    public boolean hasPastDriveStep() {
        for(int i = getIndexLastSteps(); i >= 0; i--) {
            if(isDriveStep(steps.get(i).getCommand()))
                return true;
        }

        return false;
    }

    public Step prePullNextDriveStep() {
        for(int i = currentStepCounter + 1; i < steps.size(); i++) {
            if(isDriveStep(steps.get(i).getCommand()))
                return steps.get(i);
        }

        return null;
    }

    private int getIndexLastSteps() {
        boolean flag = false;
        for(int i = currentStepCounter; i >= 0; i--) {
            if(!steps.get(i).isParallel()) {
                if(!flag)
                    flag = true;
                else
                    return i;
            }
        }

        return -1;
    }

//    public void insertStep(Step step) {
//        step.changeStepState(Constants.StepState.STATE_INIT);
//
//        steps.add(currentStepCounter, step);
//    }

    public boolean hasSteps() {
        return !(currentStepCounter == steps.size() - 1);
    }

    public static boolean isDriveStep(String name) {
        return name.equalsIgnoreCase("Drive") || name.equalsIgnoreCase("SetPosition");
    }

    public void resetCounter() {
        currentStepCounter = -1;
    }

//    public List<Step> prePullSplineSteps() {
//        List<Step> returnList = new ArrayList<>();
//        int size = steps.size();
//
//        if(size - 1 == currentSplinePrePullCounter)
//            return null;
//
//        int i;
//        for (i = Math.max(currentSplinePrePullCounter, 0); i < size; i++) {
//            if (steps.get(i).getCommand().toUpperCase().contains("RAW")) {
//                continue;
//            } else if (steps.get(i).isParallel() || steps.get(i).getCommand().equalsIgnoreCase("drive")) {
//                Step step = steps.get(i);
//                returnList.add(step);
//            } else {
//                break;
//            }
//        }
//
//        currentSplinePrePullCounter += i;
//
//        if(returnList.size() == 0)
//            return null;
//
//        return returnList;
//    }

    public Step parseSteps(Node stepNode) {
        Step returner = new Step();

        Element element = (Element) stepNode;
        NodeList childNodes = element.getChildNodes();
        for (int i = 0; i < childNodes.getLength(); i++) {
            Node child = childNodes.item(i);
            if(child.getNodeName().contains("parm")) {
                returner.setParm(child.getNodeName(), child.getTextContent());
            }
        }

        for (Step.StepValues a : Step.StepValues.values()) {
            NodeList node = element.getElementsByTagName(a.toString().toLowerCase());
            if(node.getLength() > 0)
                returner.setValue(new Pair<>(a, node.item(0).getTextContent()));
        }
        return returner;
    }

    public void writeSteps() {
        logger.writeEvent(0, FileLogger.EventType.Status, "Writing Steps from List");

        try {
            DocumentBuilderFactory docFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder docBuilder = docFactory.newDocumentBuilder();

            //root elements
            Document doc = docBuilder.newDocument();

            Element rootElement = doc.createElement("company");
            doc.appendChild(rootElement);

            for(Step a : getSteps()) {
                rootElement.appendChild(a.getXMLFileElement(doc));
            }

            //write the content into xml file
            TransformerFactory transformerFactory =  TransformerFactory.newInstance();
            Transformer transformer = transformerFactory.newTransformer();
            DOMSource source = new DOMSource(doc);

            StreamResult result =  new StreamResult(stepFile);
            transformer.transform(source, result);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}