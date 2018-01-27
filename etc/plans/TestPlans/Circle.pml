<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1517049779097" name="Circle" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/TestPlans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1517050156898" name="NewState" comment="" entryPoint="1517050156899">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517050654918</plans>
    <outTransitions>#1517053224314</outTransitions>
  </states>
  <states id="1517050290391" name="NewState" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517050868764</plans>
    <inTransitions>#1517053224314</inTransitions>
    <outTransitions>#1517053226867</outTransitions>
  </states>
  <states id="1517050320135" name="NewState" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517051759967</plans>
    <inTransitions>#1517053226867</inTransitions>
    <outTransitions>#1517053229393</outTransitions>
  </states>
  <states id="1517050323399" name="NewState" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517051793270</plans>
    <inTransitions>#1517053229393</inTransitions>
    <outTransitions>#1517053230919</outTransitions>
  </states>
  <states id="1517050336217" name="NewState" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517051612340</plans>
    <inTransitions>#1517053230919</inTransitions>
    <outTransitions>#1517053232437</outTransitions>
  </states>
  <states id="1517050341337" name="NewState" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517051857520</plans>
    <inTransitions>#1517053232437</inTransitions>
    <outTransitions>#1517053234621</outTransitions>
  </states>
  <states id="1517050344714" name="NewState" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517052103672</plans>
    <inTransitions>#1517053234621</inTransitions>
    <outTransitions>#1517053236930</outTransitions>
  </states>
  <states id="1517050350139" name="NewState" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517051638768</plans>
    <inTransitions>#1517053236930</inTransitions>
    <outTransitions>#1517053331194</outTransitions>
  </states>
  <states id="1517053316696" name="NewState" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517050654918</plans>
    <inTransitions>#1517053331194</inTransitions>
    <outTransitions>#1517053333551</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1517053319601" name="NewSuccessState" comment="">
    <inTransitions>#1517053333551</inTransitions>
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
  <transitions id="1517053331194" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517053333358" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517050350139</inState>
    <outState>#1517053316696</outState>
  </transitions>
  <transitions id="1517053333551" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517053335120" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517053316696</inState>
    <outState>#1517053319601</outState>
  </transitions>
  <entryPoints id="1517050156899" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1517050156898</state>
  </entryPoints>
</alica:Plan>
