<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1518623186077" name="DriveZigZag_ViewZigZag" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1518623225899" name="Zig1" comment="" entryPoint="1518623225900">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517415399331</plans>
  </states>
  <states id="1518623354324" name="Zag1" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517415329054</plans>
  </states>
  <states id="1518623356568" name="Zig2" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517415170220</plans>
  </states>
  <states id="1518623359185" name="Zag2" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517415111859</plans>
  </states>
  <states id="1518623360811" name="Zig3" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517414498820</plans>
  </states>
  <states id="1518623422054" name="Zag3" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517414454147</plans>
  </states>
  <states id="1518623595252" name="Zig4" comment=""/>
  <states id="1518623598950" name="Zag4" comment=""/>
  <states id="1518623603896" name="Zig5" comment=""/>
  <states id="1518623607938" name="Zag5" comment=""/>
  <entryPoints id="1518623225900" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1518623225899</state>
  </entryPoints>
</alica:Plan>
