<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1507130066325" name="WheelRadiusCalibration" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Calibration" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1507130066326" name="Start" comment="" entryPoint="1507130066327">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/Stop.beh#1413992626194</plans>
    <outTransitions>#1507130218746</outTransitions>
  </states>
  <states id="1507130150009" name="Drive" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">DriveForward.beh#1507131285711</plans>
    <plans xsi:type="alica:BehaviourConfiguration">MeasureDistance.beh#1508940977696</plans>
    <plans xsi:type="alica:BehaviourConfiguration">CountDown.beh#1508941007133</plans>
    <inTransitions>#1507130218746</inTransitions>
    <inTransitions>#1507131367548</inTransitions>
    <outTransitions>#1507131118429</outTransitions>
  </states>
  <states id="1507130223303" name="MeasureAndConfigure" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">MeasureAndConfigure.beh#1507131484850</plans>
    <inTransitions>#1507131118429</inTransitions>
    <outTransitions>#1507131121008</outTransitions>
    <outTransitions>#1507131365490</outTransitions>
  </states>
  <states id="1507130989734" name="Finished" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1507131121008</inTransitions>
  </states>
  <states id="1507131359921" name="RestartMotion" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">RestartMotion.beh#1472657588489</plans>
    <inTransitions>#1507131365490</inTransitions>
    <outTransitions>#1507131367548</outTransitions>
  </states>
  <transitions id="1507130218746" name="MISSING_NAME" comment="start signal -> success" msg="">
    <preCondition id="1507130219773" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1507130066326</inState>
    <outState>#1507130150009</outState>
  </transitions>
  <transitions id="1507131118429" name="MISSING_NAME" comment="AfterDriving -> success" msg="">
    <preCondition id="1507131120023" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1507130150009</inState>
    <outState>#1507130223303</outState>
  </transitions>
  <transitions id="1507131121008" name="MISSING_NAME" comment="results are okay -> success" msg="">
    <preCondition id="1507131122842" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1507130223303</inState>
    <outState>#1507130989734</outState>
  </transitions>
  <transitions id="1507131365490" name="MISSING_NAME" comment="results are not good enough -> failure" msg="">
    <preCondition id="1507131367115" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1507130223303</inState>
    <outState>#1507131359921</outState>
  </transitions>
  <transitions id="1507131367548" name="MISSING_NAME" comment="INSTANT SUCCESS!!!" msg="">
    <preCondition id="1507131368562" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1507131359921</inState>
    <outState>#1507130150009</outState>
  </transitions>
  <entryPoints id="1507130066327" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1507130066326</state>
  </entryPoints>
</alica:Plan>
