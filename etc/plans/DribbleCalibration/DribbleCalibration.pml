<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1469108805289" name="DribbleCalibration" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/DribbleCalibration" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1469108805290" name="Stop" comment="" entryPoint="1469108805291">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1469109061857</inTransitions>
    <inTransitions>#1469109066452</inTransitions>
    <inTransitions>#1469109277626</inTransitions>
    <inTransitions>#1469109279849</inTransitions>
    <inTransitions>#1469284197329</inTransitions>
    <inTransitions>#1485356815055</inTransitions>
    <outTransitions>#1469109261763</outTransitions>
  </states>
  <states id="1469108950942" name="Calibration Forward" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/DribbleCalibration.beh#1482339937439</plans>
    <inTransitions>#1469284199867</inTransitions>
    <outTransitions>#1469109046400</outTransitions>
    <outTransitions>#1469109279849</outTransitions>
  </states>
  <states id="1469108971711" name="Calibaration Backward" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/DribbleCalibration.beh#1482339837722</plans>
    <inTransitions>#1469109046400</inTransitions>
    <outTransitions>#1469109061857</outTransitions>
    <outTransitions>#1485356843859</outTransitions>
  </states>
  <states id="1469108996737" name="Calibration Rotation Right" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../TestPlans/DribbleControlTest/DribbleControlMOS.beh#1479905216821</plans>
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/DribbleCalibration.beh#1485355250017</plans>
    <inTransitions>#1485356843859</inTransitions>
    <outTransitions>#1469109066452</outTransitions>
    <outTransitions>#1485356813423</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1469109142048" name="finished" comment=""/>
  <states id="1469109218874" name="Calibration Take Ball" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/CalibrationTakeBall.beh#1469109486033</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../Dribble/DribbleControl.beh#1450175539163</plans>
    <inTransitions>#1469109261763</inTransitions>
    <outTransitions>#1469109264181</outTransitions>
    <outTransitions>#1469109277626</outTransitions>
  </states>
  <states id="1469284166290" name="Calibration Ball Holding" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/CalibrationBallHolding.beh#1469284324012</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../TestPlans/DribbleControlTest/DribbleControlMOS.beh#1479905216821</plans>
    <inTransitions>#1469109264181</inTransitions>
    <outTransitions>#1469284197329</outTransitions>
    <outTransitions>#1469284199867</outTransitions>
  </states>
  <states id="1485356788982" name="Calibration Rotation Left" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../TestPlans/DribbleControlTest/DribbleControlMOS.beh#1479905216821</plans>
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/DribbleCalibration.beh#1485355187631</plans>
    <inTransitions>#1485356813423</inTransitions>
    <outTransitions>#1485356815055</outTransitions>
  </states>
  <transitions id="1469109046400" name="MISSING_NAME" comment="Calibration Forward finished" msg="">
    <preCondition id="1469109050023" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469108950942</inState>
    <outState>#1469108971711</outState>
  </transitions>
  <transitions id="1469109061857" name="MISSING_NAME" comment="Stop" msg="">
    <preCondition id="1469109063861" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469108971711</inState>
    <outState>#1469108805290</outState>
  </transitions>
  <transitions id="1469109066452" name="MISSING_NAME" comment="Stop" msg="">
    <preCondition id="1469109068711" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469108996737</inState>
    <outState>#1469108805290</outState>
  </transitions>
  <transitions id="1469109261763" name="MISSING_NAME" comment="Start" msg="">
    <preCondition id="1469109263885" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469108805290</inState>
    <outState>#1469109218874</outState>
  </transitions>
  <transitions id="1469109264181" name="MISSING_NAME" comment="Calibration Take Ball finished" msg="">
    <preCondition id="1469109265377" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469109218874</inState>
    <outState>#1469284166290</outState>
  </transitions>
  <transitions id="1469109277626" name="MISSING_NAME" comment="Stop" msg="">
    <preCondition id="1469109279649" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469109218874</inState>
    <outState>#1469108805290</outState>
  </transitions>
  <transitions id="1469109279849" name="MISSING_NAME" comment="Stop" msg="">
    <preCondition id="1469109281934" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469108950942</inState>
    <outState>#1469108805290</outState>
  </transitions>
  <transitions id="1469284197329" name="MISSING_NAME" comment="Stop" msg="">
    <preCondition id="1469284199355" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469284166290</inState>
    <outState>#1469108805290</outState>
  </transitions>
  <transitions id="1469284199867" name="MISSING_NAME" comment="Calibration Ball Holding finished" msg="">
    <preCondition id="1469284201761" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469284166290</inState>
    <outState>#1469108950942</outState>
  </transitions>
  <transitions id="1485356813423" name="MISSING_NAME" comment="Calibration Rotation Right finished" msg="">
    <preCondition id="1485356814622" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469108996737</inState>
    <outState>#1485356788982</outState>
  </transitions>
  <transitions id="1485356815055" name="MISSING_NAME" comment="Stop" msg="">
    <preCondition id="1485356818666" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1485356788982</inState>
    <outState>#1469108805290</outState>
  </transitions>
  <transitions id="1485356843859" name="MISSING_NAME" comment="Calibration Backward finished" msg="">
    <preCondition id="1485356846217" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469108971711</inState>
    <outState>#1469108996737</outState>
  </transitions>
  <entryPoints id="1469108805291" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1469108805290</state>
  </entryPoints>
</alica:Plan>
