<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1517056420902" name="curves" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1517059003792" name="NewState" comment="" entryPoint="1517059003794">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517059846150</plans>
  </states>
  <states id="1517059221266" name="NewState" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517059982345</plans>
  </states>
  <states id="1517059229140" name="NewState" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517060240311</plans>
  </states>
  <states id="1517059234259" name="NewState" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517060347896</plans>
  </states>
  <states id="1517059236733" name="NewState" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517059846150</plans>
  </states>
  <states id="1517059238829" name="NewState" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517059916015</plans>
  </states>
  <states id="1517059240807" name="NewState" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517060154678</plans>
  </states>
  <states id="1517059243336" name="NewState" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517060299670</plans>
  </states>
  <states id="1517059246080" name="NewState" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517060430547</plans>
  </states>
  <states xsi:type="alica:SuccessState" id="1517059940771" name="NewSuccessState" comment=""/>
  <states id="1517060586509" name="NewState" comment=""/>
  <states id="1517060594421" name="NewState" comment=""/>
  <entryPoints id="1517059003794" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1517059003792</state>
  </entryPoints>
</alica:Plan>
