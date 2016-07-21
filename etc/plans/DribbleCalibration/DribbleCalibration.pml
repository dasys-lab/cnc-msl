<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1469108805289" name="DribbleCalibration" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/DribbleCalibration" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1469108805290" name="Stop" comment="" entryPoint="1469108805291">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1469109061857</inTransitions>
    <inTransitions>#1469109064014</inTransitions>
    <inTransitions>#1469109066452</inTransitions>
    <inTransitions>#1469109277626</inTransitions>
    <inTransitions>#1469109279849</inTransitions>
    <outTransitions>#1469109261763</outTransitions>
  </states>
  <states id="1469108950942" name="Calibration Forward" comment="">
    <inTransitions>#1469109264181</inTransitions>
    <outTransitions>#1469109046400</outTransitions>
    <outTransitions>#1469109279849</outTransitions>
  </states>
  <states id="1469108971711" name="Calibaration Backward" comment="">
    <inTransitions>#1469109046400</inTransitions>
    <outTransitions>#1469109050223</outTransitions>
    <outTransitions>#1469109061857</outTransitions>
  </states>
  <states id="1469108982128" name="Calibration Orthogonal" comment="">
    <inTransitions>#1469109050223</inTransitions>
    <outTransitions>#1469109053820</outTransitions>
    <outTransitions>#1469109064014</outTransitions>
  </states>
  <states id="1469108996737" name="Calibration Rotation" comment="">
    <inTransitions>#1469109053820</inTransitions>
    <outTransitions>#1469109066452</outTransitions>
    <outTransitions>#1469109160465</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1469109142048" name="finished" comment="">
    <inTransitions>#1469109160465</inTransitions>
  </states>
  <states id="1469109218874" name="Calibration Take Ball" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/CalibrationTakeBall.beh#1469109486033</plans>
    <inTransitions>#1469109261763</inTransitions>
    <outTransitions>#1469109264181</outTransitions>
    <outTransitions>#1469109277626</outTransitions>
  </states>
  <transitions id="1469109046400" name="MISSING_NAME" comment="Calibration Forward finished" msg="">
    <preCondition id="1469109050023" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469108950942</inState>
    <outState>#1469108971711</outState>
  </transitions>
  <transitions id="1469109050223" name="MISSING_NAME" comment="calibration Backward finished" msg="">
    <preCondition id="1469109053532" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469108971711</inState>
    <outState>#1469108982128</outState>
  </transitions>
  <transitions id="1469109053820" name="MISSING_NAME" comment="Calibration Orthogonal finished" msg="">
    <preCondition id="1469109054874" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469108982128</inState>
    <outState>#1469108996737</outState>
  </transitions>
  <transitions id="1469109061857" name="MISSING_NAME" comment="Stop" msg="">
    <preCondition id="1469109063861" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469108971711</inState>
    <outState>#1469108805290</outState>
  </transitions>
  <transitions id="1469109064014" name="MISSING_NAME" comment="Stop" msg="">
    <preCondition id="1469109066252" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469108982128</inState>
    <outState>#1469108805290</outState>
  </transitions>
  <transitions id="1469109066452" name="MISSING_NAME" comment="Stop" msg="">
    <preCondition id="1469109068711" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469108996737</inState>
    <outState>#1469108805290</outState>
  </transitions>
  <transitions id="1469109160465" name="MISSING_NAME" comment="Calibration Rotation finished -> success" msg="">
    <preCondition id="1469109162993" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469108996737</inState>
    <outState>#1469109142048</outState>
  </transitions>
  <transitions id="1469109261763" name="MISSING_NAME" comment="Start" msg="">
    <preCondition id="1469109263885" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469108805290</inState>
    <outState>#1469109218874</outState>
  </transitions>
  <transitions id="1469109264181" name="MISSING_NAME" comment="Calibration Take Ball finished" msg="">
    <preCondition id="1469109265377" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469109218874</inState>
    <outState>#1469108950942</outState>
  </transitions>
  <transitions id="1469109277626" name="MISSING_NAME" comment="Stop" msg="">
    <preCondition id="1469109279649" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469109218874</inState>
    <outState>#1469108805290</outState>
  </transitions>
  <transitions id="1469109279849" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1469109281934" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469108950942</inState>
    <outState>#1469108805290</outState>
  </transitions>
  <entryPoints id="1469108805291" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1469108805290</state>
  </entryPoints>
</alica:Plan>
