<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1442919721161" name="MotionCalibration" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Calibration" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1442919721162" name="Stop" comment="" entryPoint="1442919721163">
    <outTransitions>#1442919801497</outTransitions>
  </states>
  <states id="1442919790374" name="MoveInX" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/CalcCalib.beh#1446033354004</plans>
    <plans xsi:type="alica:BehaviourConfiguration">DriveToPointCalib.beh#1474278292264</plans>
    <inTransitions>#1442919801497</inTransitions>
    <outTransitions>#1442921106318</outTransitions>
    <outTransitions>#1474288535134</outTransitions>
  </states>
  <states id="1442921032957" name="MoveToMiddle" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/CalcCalib.beh#1446033354004</plans>
    <plans xsi:type="alica:BehaviourConfiguration">DriveToPointCalib.beh#1474278926472</plans>
    <inTransitions>#1442921106318</inTransitions>
    <outTransitions>#1443003805912</outTransitions>
    <outTransitions>#1474288537777</outTransitions>
  </states>
  <states id="1443003793160" name="MoveInY" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/CalcCalib.beh#1446033354004</plans>
    <plans xsi:type="alica:BehaviourConfiguration">DriveToPointCalib.beh#1474278968035</plans>
    <inTransitions>#1443003805912</inTransitions>
    <outTransitions>#1443003845234</outTransitions>
    <outTransitions>#1464350167678</outTransitions>
  </states>
  <states id="1443003834928" name="Stop" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/Stop.beh#1413992626194</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/CalcCalib.beh#1446033354004</plans>
    <inTransitions>#1443003845234</inTransitions>
    <inTransitions>#1464350170546</inTransitions>
    <inTransitions>#1474288535134</inTransitions>
    <inTransitions>#1474288537777</inTransitions>
    <outTransitions>#1443522261454</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1443522242711" name="Success" comment="">
    <inTransitions>#1443522261454</inTransitions>
  </states>
  <states id="1464350100818" name="MoveToMiddle2" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/CalcCalib.beh#1446033354004</plans>
    <plans xsi:type="alica:BehaviourConfiguration">DriveToPointCalib.beh#1474278926472</plans>
    <inTransitions>#1464350167678</inTransitions>
    <outTransitions>#1464350170546</outTransitions>
  </states>
  <transitions id="1442919801497" name="1" comment="Situation==start" msg="">
    <preCondition id="1442919804925" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1442919721162</inState>
    <outState>#1442919790374</outState>
  </transitions>
  <transitions id="1442921106318" name="2" comment="Situation1" msg="">
    <preCondition id="1442921109582" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1442919790374</inState>
    <outState>#1442921032957</outState>
  </transitions>
  <transitions id="1443003805912" name="3" comment="Situation2" msg="">
    <preCondition id="1443003809289" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1442921032957</inState>
    <outState>#1443003793160</outState>
  </transitions>
  <transitions id="1443003845234" name="s3" comment="situation==stop" msg="">
    <preCondition id="1443003847207" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1443003793160</inState>
    <outState>#1443003834928</outState>
  </transitions>
  <transitions id="1443522261454" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1443522265673" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1443003834928</inState>
    <outState>#1443522242711</outState>
  </transitions>
  <transitions id="1464350167678" name="4" comment="Situation3" msg="">
    <preCondition id="1464350170265" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1443003793160</inState>
    <outState>#1464350100818</outState>
  </transitions>
  <transitions id="1464350170546" name="5" comment="situation==stop_finish" msg="">
    <preCondition id="1464350172193" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464350100818</inState>
    <outState>#1443003834928</outState>
  </transitions>
  <transitions id="1474288535134" name="MISSING_NAME" comment="situation==stop" msg="">
    <preCondition id="1474288537568" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1442919790374</inState>
    <outState>#1443003834928</outState>
  </transitions>
  <transitions id="1474288537777" name="MISSING_NAME" comment="situation==stop" msg="">
    <preCondition id="1474288540200" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1442921032957</inState>
    <outState>#1443003834928</outState>
  </transitions>
  <entryPoints id="1442919721163" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1442919721162</state>
  </entryPoints>
</alica:Plan>
