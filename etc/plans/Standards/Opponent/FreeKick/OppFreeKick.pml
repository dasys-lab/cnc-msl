<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1445411471122" name="OppFreeKick" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Standards/Opponent/FreeKick" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <conditions xsi:type="alica:RuntimeCondition" id="1445442215438" name="NewRuntimeCondition" comment="" conditionString="" pluginName="DefaultPlugin">
    <quantifiers xsi:type="alica:ForallAgents" id="1445442232032" name="MISSING_NAME" comment="" scope="1445411471123">
      <sorts>x</sorts>
      <sorts>y</sorts>
    </quantifiers>
  </conditions>
  <states id="1445411471123" name="PosDef" comment="" entryPoint="1445411471124">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/Pos4Def.beh#1445438204426</plans>
    <outTransitions>#1447875673956</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1447875657650" name="Success" comment="">
    <inTransitions>#1447875673956</inTransitions>
  </states>
  <transitions id="1447875673956" name="MISSING_NAME" comment="any children success" msg="">
    <preCondition id="1447875675479" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1445411471123</inState>
    <outState>#1447875657650</outState>
  </transitions>
  <entryPoints id="1445411471124" name="MISSING_NAME" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1445411471123</state>
  </entryPoints>
</alica:Plan>
