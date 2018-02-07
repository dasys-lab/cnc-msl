<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1517049779097" name="Circle" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/TestPlans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1517050156898" name="N" comment="" entryPoint="1517050156899">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517050799588</plans>
    <outTransitions>#1517053224314</outTransitions>
  </states>
  <states id="1517050290391" name="NO" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1518021424504</plans>
    <inTransitions>#1517053224314</inTransitions>
    <outTransitions>#1517053226867</outTransitions>
  </states>
  <states id="1517050320135" name="NOO" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1518021456601</plans>
    <inTransitions>#1517053226867</inTransitions>
    <outTransitions>#1517053229393</outTransitions>
  </states>
  <states id="1517050323399" name="O" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1518021574312</plans>
    <inTransitions>#1517053229393</inTransitions>
    <outTransitions>#1517053230919</outTransitions>
  </states>
  <states id="1517050336217" name="SOO" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1518022404307</plans>
    <inTransitions>#1517053230919</inTransitions>
    <outTransitions>#1517053232437</outTransitions>
  </states>
  <states id="1517050341337" name="SO" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1518022018559</plans>
    <inTransitions>#1517053232437</inTransitions>
    <outTransitions>#1517053234621</outTransitions>
  </states>
  <states id="1517050344714" name="S" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1518021641809</plans>
    <inTransitions>#1517053234621</inTransitions>
    <outTransitions>#1517053236930</outTransitions>
  </states>
  <states id="1517050350139" name="SW" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1518021973439</plans>
    <inTransitions>#1517053236930</inTransitions>
    <outTransitions>#1518022692480</outTransitions>
  </states>
  <states id="1517053316696" name="N" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517050799588</plans>
    <inTransitions>#1518022701636</inTransitions>
    <outTransitions>#1517053333551</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1517053319601" name="NewSuccessState" comment="">
    <inTransitions>#1517053333551</inTransitions>
  </states>
  <states id="1518021513375" name="SWW" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1518022424449</plans>
    <inTransitions>#1518022692480</inTransitions>
    <outTransitions>#1518022695346</outTransitions>
  </states>
  <states id="1518021516103" name="W" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1518022550705</plans>
    <inTransitions>#1518022695346</inTransitions>
    <outTransitions>#1518022697929</outTransitions>
  </states>
  <states id="1518021520731" name="NWW" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1518022686664</plans>
    <inTransitions>#1518022697929</inTransitions>
    <outTransitions>#1518022699881</outTransitions>
  </states>
  <states id="1518021524242" name="NW" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1518021786987</plans>
    <inTransitions>#1518022699881</inTransitions>
    <outTransitions>#1518022701636</outTransitions>
  </states>
  <transitions id="1517053224314" name="MISSING_NAME" comment="succes place [0,0,0,-2000]" msg="">
    <preCondition id="1517053226586" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517050156898</inState>
    <outState>#1517050290391</outState>
  </transitions>
  <transitions id="1517053226867" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517053229128" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517050290391</inState>
    <outState>#1517050320135</outState>
  </transitions>
  <transitions id="1517053229393" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517053230743" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517050320135</inState>
    <outState>#1517050323399</outState>
  </transitions>
  <transitions id="1517053230919" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517053232269" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517050323399</inState>
    <outState>#1517050336217</outState>
  </transitions>
  <transitions id="1517053232437" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517053234317" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517050336217</inState>
    <outState>#1517050341337</outState>
  </transitions>
  <transitions id="1517053234621" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517053236593" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517050341337</inState>
    <outState>#1517050344714</outState>
  </transitions>
  <transitions id="1517053236930" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517053238931" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517050344714</inState>
    <outState>#1517050350139</outState>
  </transitions>
  <transitions id="1517053333551" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517053335120" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517053316696</inState>
    <outState>#1517053319601</outState>
  </transitions>
  <transitions id="1518022692480" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518022694769" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517050350139</inState>
    <outState>#1518021513375</outState>
  </transitions>
  <transitions id="1518022695346" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518022697537" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518021513375</inState>
    <outState>#1518021516103</outState>
  </transitions>
  <transitions id="1518022697929" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518022699512" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518021516103</inState>
    <outState>#1518021520731</outState>
  </transitions>
  <transitions id="1518022699881" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518022701387" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518021520731</inState>
    <outState>#1518021524242</outState>
  </transitions>
  <transitions id="1518022701636" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518022703381" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518021524242</inState>
    <outState>#1517053316696</outState>
  </transitions>
  <entryPoints id="1517050156899" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1517050156898</state>
  </entryPoints>
</alica:Plan>
