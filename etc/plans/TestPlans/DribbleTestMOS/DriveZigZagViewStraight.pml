<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1518623233217" name="DriveZigZagViewStraight" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1518623253780" name="Zig1" comment="" entryPoint="1518623253781">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630243941</plans>
  </states>
  <states id="1518626788701" name="Zag1" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630243942</plans>
  </states>
  <states id="1518626804312" name="Zig2" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630243943</plans>
  </states>
  <states id="1518626806747" name="Zag2" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630243951</plans>
  </states>
  <states id="1518626813522" name="Zig3" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630243961</plans>
  </states>
  <states id="1518626818294" name="Zag3" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630243971</plans>
  </states>
  <states id="1518626900749" name="Zig4" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630244941</plans>
  </states>
  <states id="1518626902606" name="Zag4" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630245941</plans>
  </states>
  <states id="1518626904168" name="Zig5" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630245942</plans>
  </states>
  <states id="1518626906208" name="Zag5" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630245943</plans>
  </states>
  <states id="1518626915328" name="Zig6" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630246041</plans>
  </states>
  <states id="1518626921067" name="Zag6" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630248941</plans>
  </states>
  <states id="1518627895634" name="Turn" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518632911294</plans>
  </states>
  <states id="1518627905490" name="Zag7" comment=""/>
  <states id="1518627908250" name="Zig7" comment=""/>
  <states id="1518627910387" name="Zag8" comment=""/>
  <states id="1518627912340" name="Zig8" comment=""/>
  <states id="1518627914142" name="Zag9" comment=""/>
  <states id="1518627927737" name="Zig9" comment=""/>
  <states id="1518627930880" name="Zag10" comment=""/>
  <states id="1518627932536" name="Zig10" comment=""/>
  <states id="1518627933858" name="Zag11" comment=""/>
  <states id="1518627935733" name="Zig11" comment=""/>
  <states id="1518627937452" name="Zag12" comment=""/>
  <states id="1518627939373" name="Zig12" comment=""/>
  <entryPoints id="1518623253781" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1518623253780</state>
  </entryPoints>
</alica:Plan>
