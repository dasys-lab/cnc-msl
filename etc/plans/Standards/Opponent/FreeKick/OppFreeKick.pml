<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1445411471122" name="OppFreeKick" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.075" destinationPath="Plans/Standards/Opponent/FreeKick" priority="0.0" minCardinality="1" maxCardinality="4">
  <conditions xsi:type="alica:RuntimeCondition" id="1445442215438" name="NewRuntimeCondition" comment="" conditionString="" pluginName="DefaultPlugin">
    <quantifiers xsi:type="alica:ForallAgents" id="1445442232032" name="MISSING_NAME" comment="" scope="1454663055945">
      <sorts>x</sorts>
      <sorts>y</sorts>
    </quantifiers>
    <quantifiers xsi:type="alica:ForallAgents" id="1454663289240" name="MISSING_NAME" comment="" scope="1445411471123">
      <sorts>x</sorts>
      <sorts>y</sorts>
    </quantifiers>
    <quantifiers xsi:type="alica:ForallAgents" id="1454663310321" name="MISSING_NAME" comment="" scope="1454663058990">
      <sorts>x</sorts>
      <sorts>y</sorts>
    </quantifiers>
  </conditions>
  <states id="1445411471123" name="PosDefDefender" comment="" entryPoint="1445411471124">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/Pos4Def.beh#1445438204426</plans>
    <outTransitions>#1447875673956</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1447875657650" name="Success" comment="">
    <inTransitions>#1447875673956</inTransitions>
    <inTransitions>#1454663208360</inTransitions>
    <inTransitions>#1454663210633</inTransitions>
  </states>
  <states id="1454663055945" name="PosDefBlocker" comment="" entryPoint="1454663032454">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/Pos4Def.beh#1445438204426</plans>
    <outTransitions>#1454663208360</outTransitions>
  </states>
  <states id="1454663058990" name="PosDefAttacker" comment="" entryPoint="1454663045348">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/Pos4Def.beh#1445438204426</plans>
    <outTransitions>#1454663210633</outTransitions>
  </states>
  <transitions id="1447875673956" name="MISSING_NAME" comment="any children success" msg="">
    <preCondition id="1447875675479" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1445411471123</inState>
    <outState>#1447875657650</outState>
  </transitions>
  <transitions id="1454663208360" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1454663210272" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1454663055945</inState>
    <outState>#1447875657650</outState>
  </transitions>
  <transitions id="1454663210633" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1454663213143" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1454663058990</inState>
    <outState>#1447875657650</outState>
  </transitions>
  <entryPoints id="1445411471124" name="Defend" comment="" successRequired="true" minCardinality="0" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1225115406909</task>
    <state>#1445411471123</state>
  </entryPoints>
  <entryPoints id="1454663032454" name="Block" comment="" successRequired="true" minCardinality="0" maxCardinality="2">
    <task>../../../../Misc/taskrepository.tsk#1461237765109</task>
    <state>#1454663055945</state>
  </entryPoints>
  <entryPoints id="1454663045348" name="Attack" comment="" successRequired="true" minCardinality="1" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1222613952469</task>
    <state>#1454663058990</state>
  </entryPoints>
</alica:Plan>
