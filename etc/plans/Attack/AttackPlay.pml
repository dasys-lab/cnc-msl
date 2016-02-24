<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1434045709191" name="AttackPlay" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Attack" priority="0.0" minCardinality="2" maxCardinality="2147483647">
  <conditions xsi:type="alica:RuntimeCondition" id="1434112519736" name="NewRuntimeCondition" comment="haveBall" conditionString="" pluginName="DefaultPlugin"/>
  <states id="1434045709193" name="Attack" comment="" entryPoint="1434045709194">
    <plans xsi:type="alica:Plan">StandardAttack.pml#1434046634656</plans>
  </states>
  <states id="1434045868018" name="MidFieldDefense" comment="" entryPoint="1434045719840">
    <inTransitions>#1436536103266</inTransitions>
    <outTransitions>#1436536098621</outTransitions>
  </states>
  <states id="1434045870617" name="Defend" comment="" entryPoint="1434045723977">
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/BackroomDefence.beh#1454507819086</plans>
  </states>
  <states id="1434112762535" name="Release" comment="" entryPoint="1434112675755">
    <inTransitions>#1436536160346</inTransitions>
    <outTransitions>#1436536148908</outTransitions>
  </states>
  <states id="1436536084172" name="ApproachPass" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">CatchPass.beh#1440754543898</plans>
    <inTransitions>#1436536098621</inTransitions>
    <inTransitions>#1436536101847</inTransitions>
    <outTransitions>#1436536100248</outTransitions>
    <outTransitions>#1436536103266</outTransitions>
  </states>
  <states id="1436536085953" name="InterceptPass" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/InterceptCarefully.beh#1427703234654</plans>
    <inTransitions>#1436536100248</inTransitions>
    <outTransitions>#1436536101847</outTransitions>
  </states>
  <states id="1436536121614" name="InterceptPass" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/InterceptCarefully.beh#1427703234654</plans>
    <inTransitions>#1436536150744</inTransitions>
    <outTransitions>#1436536151770</outTransitions>
  </states>
  <states id="1436536123918" name="ApproachPass" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">CatchPass.beh#1440754543898</plans>
    <inTransitions>#1436536148908</inTransitions>
    <inTransitions>#1436536151770</inTransitions>
    <outTransitions>#1436536150744</outTransitions>
    <outTransitions>#1436536160346</outTransitions>
  </states>
  <transitions id="1436536098621" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1436536099859" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434045868018</inState>
    <outState>#1436536084172</outState>
  </transitions>
  <transitions id="1436536100248" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1436536101707" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1436536084172</inState>
    <outState>#1436536085953</outState>
  </transitions>
  <transitions id="1436536101847" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1436536103010" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1436536085953</inState>
    <outState>#1436536084172</outState>
  </transitions>
  <transitions id="1436536103266" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1436536104021" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1436536084172</inState>
    <outState>#1434045868018</outState>
  </transitions>
  <transitions id="1436536148908" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1436536150520" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434112762535</inState>
    <outState>#1436536123918</outState>
  </transitions>
  <transitions id="1436536150744" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1436536151584" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1436536123918</inState>
    <outState>#1436536121614</outState>
  </transitions>
  <transitions id="1436536151770" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1436536152571" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1436536121614</inState>
    <outState>#1436536123918</outState>
  </transitions>
  <transitions id="1436536160346" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1436536161660" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1436536123918</inState>
    <outState>#1434112762535</outState>
  </transitions>
  <entryPoints id="1434045709194" name="Attack" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../Misc/taskrepository.tsk#1222613952469</task>
    <state>#1434045709193</state>
  </entryPoints>
  <entryPoints id="1434045719840" name="AttackSupport" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225115536468</task>
    <state>#1434045868018</state>
  </entryPoints>
  <entryPoints id="1434045723977" name="Defend" comment="" successRequired="false" minCardinality="1" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225115406909</task>
    <state>#1434045870617</state>
  </entryPoints>
  <entryPoints id="1434112675755" name="InGamePassReceiver" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1307185798142</task>
    <state>#1434112762535</state>
  </entryPoints>
</alica:Plan>
