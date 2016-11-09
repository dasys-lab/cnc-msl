<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1478709817405" name="TestCalculationPlan" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/TestPlans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1478709817406" name="NewYork" comment="" entryPoint="1478709817407">
    <plans xsi:type="alica:BehaviourConfiguration">../Calibration/RotationCalibrationCalculation.beh#1475074454339</plans>
    <outTransitions>#1478711352787</outTransitions>
  </states>
  <states id="1478711262082" name="Stop" comment="">
    <inTransitions>#1478711352787</inTransitions>
  </states>
  <transitions id="1478711352787" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1478711356604" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1478709817406</inState>
    <outState>#1478711262082</outState>
  </transitions>
  <entryPoints id="1478709817407" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1478709817406</state>
  </entryPoints>
</alica:Plan>
