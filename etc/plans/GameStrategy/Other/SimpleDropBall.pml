<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1426696586622" name="SimpleDropBall" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1426696586623" name="Positioning" comment="" entryPoint="1426696586624">
    <outTransitions>#1426696640520</outTransitions>
  </states>
  <states id="1426696626634" name="Execution" comment="">
    <inTransitions>#1426696640520</inTransitions>
    <outTransitions>#1426696641744</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1426696638228" name="NewSuccessState" comment="">
    <inTransitions>#1426696641744</inTransitions>
  </states>
  <transitions id="1426696640520" name="MISSING_NAME" comment="Situation==start || anychildsuccess" msg="">
    <preCondition id="1426696641527" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426696586623</inState>
    <outState>#1426696626634</outState>
  </transitions>
  <transitions id="1426696641744" name="MISSING_NAME" comment="anychildsuccess" msg="">
    <preCondition id="1426696642635" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426696626634</inState>
    <outState>#1426696638228</outState>
  </transitions>
  <entryPoints id="1426696586624" name="MISSING_NAME" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1426696586623</state>
  </entryPoints>
</alica:Plan>
