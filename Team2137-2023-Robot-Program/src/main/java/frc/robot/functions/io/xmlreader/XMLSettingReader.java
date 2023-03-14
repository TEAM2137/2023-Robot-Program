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

import frc.robot.Robot;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.LogFactory;
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
import java.io.*;
import java.nio.file.Files;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

public class XMLSettingReader {

    private Document document;
    private final File settingFile;
    private final FileLogger log;
    private final int maxSettingsFileHistory = 15;

    public XMLSettingReader(String dir, LogFactory loggerFactory, FileLogger logger) {
        log = logger;
        this.settingFile = new File(dir);

        if(!this.settingFile.exists()) {
            logger.writeEvent(0, FileLogger.EventType.Warning, "XML File Specified Does Not Exist, Attempting to Create a Basic One");
            try {
                this.settingFile.createNewFile();

                FileWriter tmpFileWriter = new FileWriter(this.settingFile);
                tmpFileWriter.write("<?xml version=\"1.0\"?>\n<Robot>\n\t<Settings>\n\t</Settings>\n</Robot>");
                tmpFileWriter.flush();
                tmpFileWriter.close();
            } catch (IOException e) {
                logger.writeEvent(0, FileLogger.EventType.Error, "Failed to create basic xml file because...");
                logger.writeEvent(0, FileLogger.EventType.Error, e.getMessage());
            }
        }

        DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();

        log.writeEvent(8, FileLogger.EventType.Status, "Starting to Parse XML file...");
        try {
            DocumentBuilder db = dbf.newDocumentBuilder();
            document = db.parse(this.settingFile);
            document.getDocumentElement().normalize();
        } catch(Exception e) {
            e.printStackTrace();
            log.writeEvent(0, FileLogger.EventType.Error, "FAILED TO OPEN AND PARSE XML FILE");
            return;
        }

        Element rootElement = (Element) document.getChildNodes().item(0);

        Element robotEntities =  (Element) rootElement.getElementsByTagName("Hardware").item(0);
        Element settingsEntities =  (Element) rootElement.getElementsByTagName("Settings").item(0);

        log.writeEvent(6, FileLogger.EventType.Status, "Starting recursive search in XML File...");
        Robot.settingsEntityGroup = new EntityGroup(settingsEntities, null, true);
        Robot.robotEntityGroup = new EntityGroup(robotEntities, null, true);

        StringBuilder builder = new StringBuilder();
        Robot.robotEntityGroup.constructTreeItemPrintout(builder, 1);
        Robot.settingsEntityGroup.constructTreeItemPrintout(builder, 1);
        log.writeLine(builder.toString());
    }

    public void write() {
        log.writeEvent(0, FileLogger.EventType.Status, "Writing Settings from Element (Recursive)");

        Robot.robotEntityGroup.updateElement();
        try {
            String dateString = DateTimeFormatter.ofPattern("MMdd_HHmmss_SSS").format(LocalDateTime.now());

            String name = this.settingFile.getName();
            name = name.substring(0, name.length() - 4);
            String dir = this.settingFile.getParentFile().getAbsolutePath();

            Files.copy(this.settingFile.toPath(), (new File(dir + "/" + name + dateString + ".xml")).toPath());
            removeUnusedFiles(dir, name);

            TransformerFactory transformerFactory = TransformerFactory.newInstance();
            Transformer transformer = transformerFactory.newTransformer();
            transformer.setOutputProperty(OutputKeys.INDENT, "no");
            DOMSource source = new DOMSource(document);

            StreamResult result = new StreamResult(settingFile);
            transformer.transform(source, result);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void removeUnusedFiles(String dir, String fileName) {
        File directory = new File(dir);

        File[] files = directory.listFiles((a) -> {
            String name = a.getName();
            return name.contains(fileName);
        });

        if (files == null) return;

        Arrays.sort(files, Comparator.comparing(File::lastModified).reversed());

        if (files.length > maxSettingsFileHistory) {
            log.writeEvent(0, FileLogger.EventType.Status, "Cleaning old setting files...");

            for (int i = maxSettingsFileHistory; i < files.length; i++) {
                files[i].delete();
            }
        }
    }

    public EntityGroup getRobot() {
        return Robot.robotEntityGroup;
    }
}