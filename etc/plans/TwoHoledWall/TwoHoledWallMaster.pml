<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1417621468963" name="TwoHoledWallMaster" comment="" destinationPath="Plans/TwoHoledWall" priority="0.0" minCardinality="0" maxCardinality="2147483647" masterPlan="true" utilityFunction="" utilityThreshold="0.1">
  <states id="1417621468964" name="Stop" comment="" entryPoint="1417621468965">
    <plans xsi:type="alica:Behaviour">GenericBehaviours/Stop.beh#1413992604875</plans>
    <inTransitions>TwoHoledWall/TwoHoledWallMaster.pml#1417621605061</inTransitions>
    <inTransitions>TwoHoledWall/TwoHoledWallMaster.pml#1417621607473</inTransitions>
    <inTransitions>TwoHoledWall/TwoHoledWallMaster.pml#1422464874668</inTransitions>
    <outTransitions>TwoHoledWall/TwoHoledWallMaster.pml#1417621598841</outTransitions>
    <outTransitions>TwoHoledWall/TwoHoledWallMaster.pml#1422464819985</outTransitions>
  </states>
  <states id="1417621580835" name="DriveToOrigin" comment="">
    <plans xsi:type="alica:Behaviour">GenericBehaviours/DriveToPoint.beh#1417620568675</plans>
    <inTransitions>TwoHoledWall/TwoHoledWallMaster.pml#1417621598841</inTransitions>
    <outTransitions>TwoHoledWall/TwoHoledWallMaster.pml#1417621602033</outTransitions>
    <outTransitions>TwoHoledWall/TwoHoledWallMaster.pml#1417621607473</outTransitions>
    <outTransitions>TwoHoledWall/TwoHoledWallMaster.pml#1422464823873</outTransitions>
  </states>
  <states id="1417621589489" name="Score" comment="">
    <plans xsi:type="alica:Plan">TwoHoledWall/ShootTwoHoledWall.pml#1417620189234</plans>
    <inTransitions>TwoHoledWall/TwoHoledWallMaster.pml#1417621602033</inTransitions>
    <outTransitions>TwoHoledWall/TwoHoledWallMaster.pml#1417621605061</outTransitions>
    <outTransitions>TwoHoledWall/TwoHoledWallMaster.pml#1422464816725</outTransitions>
  </states>
  <states id="1422464768858" name="Joysick" comment="">
    <plans xsi:type="alica:Behaviour">Behaviours/Joystick.beh#1421854975890</plans>
    <inTransitions>TwoHoledWall/TwoHoledWallMaster.pml#1422464816725</inTransitions>
    <inTransitions>TwoHoledWall/TwoHoledWallMaster.pml#1422464819985</inTransitions>
    <inTransitions>TwoHoledWall/TwoHoledWallMaster.pml#1422464823873</inTransitions>
    <outTransitions>TwoHoledWall/TwoHoledWallMaster.pml#1422464874668</outTransitions>
  </states>
  <transitions id="1417621598841" name="MISSING_NAME" comment="situation == start" msg="">
    <preCondition id="1417621600501" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>TwoHoledWall/TwoHoledWallMaster.pml#1417621468964</inState>
    <outState>TwoHoledWall/TwoHoledWallMaster.pml#1417621580835</outState>
  </transitions>
  <transitions id="1417621602033" name="MISSING_NAME" comment="anyChildSucces" msg="">
    <preCondition id="1417621604870" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>TwoHoledWall/TwoHoledWallMaster.pml#1417621580835</inState>
    <outState>TwoHoledWall/TwoHoledWallMaster.pml#1417621589489</outState>
  </transitions>
  <transitions id="1417621605061" name="MISSING_NAME" comment="anyChildSucces || situation == stop" msg="">
    <preCondition id="1417621607305" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>TwoHoledWall/TwoHoledWallMaster.pml#1417621589489</inState>
    <outState>TwoHoledWall/TwoHoledWallMaster.pml#1417621468964</outState>
  </transitions>
  <transitions id="1417621607473" name="MISSING_NAME" comment="situation == stop" msg="">
    <preCondition id="1417621611163" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>TwoHoledWall/TwoHoledWallMaster.pml#1417621580835</inState>
    <outState>TwoHoledWall/TwoHoledWallMaster.pml#1417621468964</outState>
  </transitions>
  <transitions id="1422464816725" name="MISSING_NAME" comment="situation == joystick" msg="">
    <preCondition id="1422464818034" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>TwoHoledWall/TwoHoledWallMaster.pml#1417621589489</inState>
    <outState>TwoHoledWall/TwoHoledWallMaster.pml#1422464768858</outState>
  </transitions>
  <transitions id="1422464819985" name="MISSING_NAME" comment="situation == joystick" msg="">
    <preCondition id="1422464821945" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>TwoHoledWall/TwoHoledWallMaster.pml#1417621468964</inState>
    <outState>TwoHoledWall/TwoHoledWallMaster.pml#1422464768858</outState>
  </transitions>
  <transitions id="1422464823873" name="MISSING_NAME" comment="situation == joystick" msg="">
    <preCondition id="1422464825529" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>TwoHoledWall/TwoHoledWallMaster.pml#1417621580835</inState>
    <outState>TwoHoledWall/TwoHoledWallMaster.pml#1422464768858</outState>
  </transitions>
  <transitions id="1422464874668" name="MISSING_NAME" comment="situation != joystick" msg="">
    <preCondition id="1422464875980" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>TwoHoledWall/TwoHoledWallMaster.pml#1422464768858</inState>
    <outState>TwoHoledWall/TwoHoledWallMaster.pml#1417621468964</outState>
  </transitions>
  <entryPoints id="1417621468965" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>taskrepository.tsk#1225112227903</task>
    <state>TwoHoledWall/TwoHoledWallMaster.pml#1417621468964</state>
  </entryPoints>
</alica:Plan>
