<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1523356580220" name="TestThrowInPassAlignmentMaster" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1523356580221" name="Stop" comment="" entryPoint="1523356580222">
    <plans xsi:type="alica:BehaviourConfiguration">../../../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1523356627749</inTransitions>
    <outTransitions>#1523356617889</outTransitions>
  </states>
  <states id="1523356610135" name="TestRealign" comment="">
    <inTransitions>#1523356617889</inTransitions>
    <outTransitions>#1523356627749</outTransitions>
  </states>
  <transitions id="1523356617889" name="MISSING_NAME" comment="situation start" msg="">
    <preCondition id="1523356618875" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1523356580221</inState>
    <outState>#1523356610135</outState>
  </transitions>
  <transitions id="1523356627749" name="MISSING_NAME" comment="situation stop" msg="">
    <preCondition id="1523356628773" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1523356610135</inState>
    <outState>#1523356580221</outState>
  </transitions>
  <entryPoints id="1523356580222" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1523356580221</state>
  </entryPoints>
</alica:Plan>
