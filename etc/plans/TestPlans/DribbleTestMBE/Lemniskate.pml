<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1541947378314" name="Lemniskate" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/TestPlans/DribbleTestMBE" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1541947378315" name="STOP" comment="" entryPoint="1541947378316">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1542528177475</inTransitions>
    <outTransitions>#1541948201979</outTransitions>
  </states>
  <states id="1541947933168" name="start" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveToPoint.beh#1484145325268</plans>
    <inTransitions>#1541948201979</inTransitions>
    <outTransitions>#1542528161739</outTransitions>
  </states>
  <states id="1542039866490" name="P1" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveToPoint.beh#1542040282510</plans>
    <inTransitions>#1542528161739</inTransitions>
    <outTransitions>#1542528165723</outTransitions>
  </states>
  <states id="1542040456227" name="P2" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveToPoint.beh#1542527634560</plans>
    <inTransitions>#1542528165723</inTransitions>
    <outTransitions>#1542528167206</outTransitions>
  </states>
  <states id="1542040458653" name="P3" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveToPoint.beh#1542527693861</plans>
    <inTransitions>#1542528167206</inTransitions>
    <outTransitions>#1542528168638</outTransitions>
  </states>
  <states id="1542527802217" name="P4" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveToPoint.beh#1484145325268</plans>
    <inTransitions>#1542528168638</inTransitions>
    <outTransitions>#1542528170971</outTransitions>
  </states>
  <states id="1542527959475" name="P5" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveToPoint.beh#1542527992260</plans>
    <inTransitions>#1542528170971</inTransitions>
    <outTransitions>#1542528172909</outTransitions>
  </states>
  <states id="1542528046356" name="P6" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveToPoint.beh#1542528066600</plans>
    <inTransitions>#1542528172909</inTransitions>
    <outTransitions>#1542528174899</outTransitions>
  </states>
  <states id="1542528086857" name="P7" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveToPoint.beh#1542528118292</plans>
    <inTransitions>#1542528174899</inTransitions>
    <outTransitions>#1542528176204</outTransitions>
  </states>
  <states id="1542528142131" name="P8" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveToPoint.beh#1484145325268</plans>
    <inTransitions>#1542528176204</inTransitions>
    <outTransitions>#1542528177475</outTransitions>
  </states>
  <transitions id="1541948201979" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1541948204291" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1541947378315</inState>
    <outState>#1541947933168</outState>
  </transitions>
  <transitions id="1542528161739" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1542528165498" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1541947933168</inState>
    <outState>#1542039866490</outState>
  </transitions>
  <transitions id="1542528165723" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1542528166981" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1542039866490</inState>
    <outState>#1542040456227</outState>
  </transitions>
  <transitions id="1542528167206" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1542528168349" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1542040456227</inState>
    <outState>#1542040458653</outState>
  </transitions>
  <transitions id="1542528168638" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1542528170594" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1542040458653</inState>
    <outState>#1542527802217</outState>
  </transitions>
  <transitions id="1542528170971" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1542528172510" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1542527802217</inState>
    <outState>#1542527959475</outState>
  </transitions>
  <transitions id="1542528172909" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1542528174698" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1542527959475</inState>
    <outState>#1542528046356</outState>
  </transitions>
  <transitions id="1542528174899" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1542528176027" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1542528046356</inState>
    <outState>#1542528086857</outState>
  </transitions>
  <transitions id="1542528176204" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1542528177298" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1542528086857</inState>
    <outState>#1542528142131</outState>
  </transitions>
  <transitions id="1542528177475" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1542528184111" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1542528142131</inState>
    <outState>#1541947378315</outState>
  </transitions>
  <entryPoints id="1541947378316" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1541947378315</state>
  </entryPoints>
</alica:Plan>
