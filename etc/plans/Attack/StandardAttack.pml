<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1434046634656" name="StandardAttack" comment="lostBall" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Attack" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1434046634657" name="GetBall" comment="" entryPoint="1434046634658">
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/GetBall.beh#1414840399972</plans>
    <inTransitions>#1434048722503</inTransitions>
    <inTransitions>#1434048731525</inTransitions>
    <inTransitions>#1434716045767</inTransitions>
    <outTransitions>#1434048720937</outTransitions>
    <outTransitions>#1434048723878</outTransitions>
    <outTransitions>#1434716047438</outTransitions>
  </states>
  <states id="1434048406725" name="Tackle" comment="">
    <plans xsi:type="alica:Plan">Tackle.pml#1434116965565</plans>
    <inTransitions>#1434048723878</inTransitions>
    <inTransitions>#1434048729645</inTransitions>
    <outTransitions>#1434048731525</outTransitions>
    <outTransitions>#1434048734889</outTransitions>
  </states>
  <states id="1434048705508" name="HaveBall" comment="">
    <plans xsi:type="alica:Plan">PassPlay.pml#1436268896671</plans>
    <inTransitions>#1434048720937</inTransitions>
    <inTransitions>#1434048734889</inTransitions>
    <inTransitions>#1434716048579</inTransitions>
    <outTransitions>#1434048722503</outTransitions>
    <outTransitions>#1434048729645</outTransitions>
    <outTransitions>#1434716049424</outTransitions>
  </states>
  <states id="1434715893346" name="LostBall" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Wander.beh#1434716230628</plans>
    <inTransitions>#1434716047438</inTransitions>
    <inTransitions>#1434716049424</inTransitions>
    <outTransitions>#1434716045767</outTransitions>
    <outTransitions>#1434716048579</outTransitions>
  </states>
  <transitions id="1434048720937" name="MISSING_NAME" comment="haveBall" msg="">
    <preCondition id="1434048722207" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434046634657</inState>
    <outState>#1434048705508</outState>
  </transitions>
  <transitions id="1434048722503" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1434048723635" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434048705508</inState>
    <outState>#1434046634657</outState>
  </transitions>
  <transitions id="1434048723878" name="MISSING_NAME" comment="!haveBall &amp;&amp; enemy has ball" msg="">
    <preCondition id="1434048729350" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434046634657</inState>
    <outState>#1434048406725</outState>
  </transitions>
  <transitions id="1434048729645" name="MISSING_NAME" comment="haveBall &amp;&amp; enemy close" msg="">
    <preCondition id="1434048731181" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434048705508</inState>
    <outState>#1434048406725</outState>
  </transitions>
  <transitions id="1434048731525" name="MISSING_NAME" comment="haveBall &amp;&amp; enemy not close" msg="">
    <preCondition id="1434048732966" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434048406725</inState>
    <outState>#1434046634657</outState>
  </transitions>
  <transitions id="1434048734889" name="MISSING_NAME" comment="haveball &amp;&amp; enemy not close" msg="">
    <preCondition id="1434048737070" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434048406725</inState>
    <outState>#1434048705508</outState>
  </transitions>
  <transitions id="1434716045767" name="MISSING_NAME" comment="found ball" msg="">
    <preCondition id="1434716047150" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434715893346</inState>
    <outState>#1434046634657</outState>
  </transitions>
  <transitions id="1434716047438" name="MISSING_NAME" comment="doesnt see ball" msg="">
    <preCondition id="1434716048353" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434046634657</inState>
    <outState>#1434715893346</outState>
  </transitions>
  <transitions id="1434716048579" name="MISSING_NAME" comment="found ball" msg="">
    <preCondition id="1434716049299" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434715893346</inState>
    <outState>#1434048705508</outState>
  </transitions>
  <transitions id="1434716049424" name="MISSING_NAME" comment="doesnt see ball" msg="">
    <preCondition id="1434716050319" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434048705508</inState>
    <outState>#1434715893346</outState>
  </transitions>
  <entryPoints id="1434046634658" name="Attack" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1222613952469</task>
    <state>#1434046634657</state>
  </entryPoints>
</alica:Plan>
