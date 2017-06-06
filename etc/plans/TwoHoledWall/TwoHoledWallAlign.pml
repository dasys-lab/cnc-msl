<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1496753106611" name="TwoHoledWallAlign" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/TwoHoledWall" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1496753106612" name="DriveToOrigin" comment="" entryPoint="1496753106613">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveToPoint.beh#1417620583364</plans>
    <inTransitions>#1496753473525</inTransitions>
    <outTransitions>#1496753237394</outTransitions>
  </states>
  <states id="1496753202569" name="Score" comment="">
    <plans xsi:type="alica:Plan">ShootTwoHoledWall.pml#1417620189234</plans>
    <inTransitions>#1496753237394</inTransitions>
    <outTransitions>#1496753473525</outTransitions>
  </states>
  <transitions id="1496753237394" name="MISSING_NAME" comment="anyChildSucces" msg="">
    <preCondition id="1496753238482" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1496753106612</inState>
    <outState>#1496753202569</outState>
  </transitions>
  <transitions id="1496753473525" name="MISSING_NAME" comment="anyChildSuccess" msg="">
    <preCondition id="1496753474941" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1496753202569</inState>
    <outState>#1496753106612</outState>
  </transitions>
  <entryPoints id="1496753106613" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1496753106612</state>
  </entryPoints>
</alica:Plan>
