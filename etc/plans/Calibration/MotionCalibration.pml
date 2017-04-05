<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1442919721161" name="MotionCalibration" comment="" destinationPath="Plans/Calibration" priority="0.0" minCardinality="0" maxCardinality="2147483647" masterPlan="false" utilityFunction="" utilityThreshold="0.1">
  <states id="1442919721162" name="Stop" comment="" entryPoint="1442919721163">
    <outTransitions>Calibration/MotionCalibration.pml#1442919801497</outTransitions>
  </states>
  <states id="1442919790374" name="MoveInX" comment="">
    <plans xsi:type="alica:Behaviour">Behaviours/CalcCalib.beh#1446033324019</plans>
    <plans xsi:type="alica:Behaviour">Calibration/DriveToPointCalib.beh#1474278265440</plans>
    <inTransitions>Calibration/MotionCalibration.pml#1442919801497</inTransitions>
    <outTransitions>Calibration/MotionCalibration.pml#1442921106318</outTransitions>
    <outTransitions>Calibration/MotionCalibration.pml#1474288535134</outTransitions>
  </states>
  <states id="1442921032957" name="MoveToMiddle" comment="">
    <plans xsi:type="alica:Behaviour">Behaviours/CalcCalib.beh#1446033324019</plans>
    <plans xsi:type="alica:Behaviour">Calibration/DriveToPointCalib.beh#1474278265440</plans>
    <inTransitions>Calibration/MotionCalibration.pml#1442921106318</inTransitions>
    <outTransitions>Calibration/MotionCalibration.pml#1443003805912</outTransitions>
    <outTransitions>Calibration/MotionCalibration.pml#1474288537777</outTransitions>
  </states>
  <states id="1443003793160" name="MoveInY" comment="">
    <plans xsi:type="alica:Behaviour">Behaviours/CalcCalib.beh#1446033324019</plans>
    <plans xsi:type="alica:Behaviour">Calibration/DriveToPointCalib.beh#1474278265440</plans>
    <inTransitions>Calibration/MotionCalibration.pml#1443003805912</inTransitions>
    <outTransitions>Calibration/MotionCalibration.pml#1443003845234</outTransitions>
    <outTransitions>Calibration/MotionCalibration.pml#1464350167678</outTransitions>
  </states>
  <states id="1443003834928" name="Stop" comment="">
    <plans xsi:type="alica:Behaviour">GenericBehaviours/Stop.beh#1413992604875</plans>
    <plans xsi:type="alica:Behaviour">Behaviours/CalcCalib.beh#1446033324019</plans>
    <inTransitions>Calibration/MotionCalibration.pml#1443003845234</inTransitions>
    <inTransitions>Calibration/MotionCalibration.pml#1464350170546</inTransitions>
    <inTransitions>Calibration/MotionCalibration.pml#1474288535134</inTransitions>
    <inTransitions>Calibration/MotionCalibration.pml#1474288537777</inTransitions>
    <outTransitions>Calibration/MotionCalibration.pml#1443522261454</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1443522242711" name="Success" comment="">
    <inTransitions>Calibration/MotionCalibration.pml#1443522261454</inTransitions>
  </states>
  <states id="1464350100818" name="MoveToMiddle2" comment="">
    <plans xsi:type="alica:Behaviour">Behaviours/CalcCalib.beh#1446033324019</plans>
    <plans xsi:type="alica:Behaviour">Calibration/DriveToPointCalib.beh#1474278265440</plans>
    <inTransitions>Calibration/MotionCalibration.pml#1464350167678</inTransitions>
    <outTransitions>Calibration/MotionCalibration.pml#1464350170546</outTransitions>
  </states>
  <transitions id="1442919801497" name="1" comment="Situation==start" msg="">
    <preCondition id="1442919804925" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Calibration/MotionCalibration.pml#1442919721162</inState>
    <outState>Calibration/MotionCalibration.pml#1442919790374</outState>
  </transitions>
  <transitions id="1442921106318" name="2" comment="Situation1" msg="">
    <preCondition id="1442921109582" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Calibration/MotionCalibration.pml#1442919790374</inState>
    <outState>Calibration/MotionCalibration.pml#1442921032957</outState>
  </transitions>
  <transitions id="1443003805912" name="3" comment="Situation2" msg="">
    <preCondition id="1443003809289" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Calibration/MotionCalibration.pml#1442921032957</inState>
    <outState>Calibration/MotionCalibration.pml#1443003793160</outState>
  </transitions>
  <transitions id="1443003845234" name="s3" comment="situation==stop" msg="">
    <preCondition id="1443003847207" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Calibration/MotionCalibration.pml#1443003793160</inState>
    <outState>Calibration/MotionCalibration.pml#1443003834928</outState>
  </transitions>
  <transitions id="1443522261454" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1443522265673" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Calibration/MotionCalibration.pml#1443003834928</inState>
    <outState>Calibration/MotionCalibration.pml#1443522242711</outState>
  </transitions>
  <transitions id="1464350167678" name="4" comment="Situation3" msg="">
    <preCondition id="1464350170265" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Calibration/MotionCalibration.pml#1443003793160</inState>
    <outState>Calibration/MotionCalibration.pml#1464350100818</outState>
  </transitions>
  <transitions id="1464350170546" name="5" comment="situation==stop_finish" msg="">
    <preCondition id="1464350172193" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Calibration/MotionCalibration.pml#1464350100818</inState>
    <outState>Calibration/MotionCalibration.pml#1443003834928</outState>
  </transitions>
  <transitions id="1474288535134" name="MISSING_NAME" comment="situation==stop" msg="">
    <preCondition id="1474288537568" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Calibration/MotionCalibration.pml#1442919790374</inState>
    <outState>Calibration/MotionCalibration.pml#1443003834928</outState>
  </transitions>
  <transitions id="1474288537777" name="MISSING_NAME" comment="situation==stop" msg="">
    <preCondition id="1474288540200" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Calibration/MotionCalibration.pml#1442921032957</inState>
    <outState>Calibration/MotionCalibration.pml#1443003834928</outState>
  </transitions>
  <entryPoints id="1442919721163" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>taskrepository.tsk#1225112227903</task>
    <state>Calibration/MotionCalibration.pml#1442919721162</state>
  </entryPoints>
</alica:Plan>
