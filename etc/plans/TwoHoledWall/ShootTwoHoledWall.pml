<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1417620189234" name="ShootTwoHoledWall" comment="notHaveBall" destinationPath="Plans/TwoHoledWall" priority="0.0" minCardinality="0" maxCardinality="2147483647" masterPlan="false" utilityFunction="" utilityThreshold="0.1">
  <states id="1417620209050" name="GrabBall" comment="" entryPoint="1417620209051">
    <plans xsi:type="alica:Behaviour">GenericBehaviours/InterceptCarefully.beh#1427703218101</plans>
    <inTransitions>TwoHoledWall/ShootTwoHoledWall.pml#1417620327753</inTransitions>
    <outTransitions>TwoHoledWall/ShootTwoHoledWall.pml#1417620267846</outTransitions>
  </states>
  <states id="1417620225739" name="AlignAndShoot" comment="">
    <plans xsi:type="alica:Behaviour">TwoHoledWall/AlignAndShootTwoHoledWall.beh#1417620683982</plans>
    <inTransitions>TwoHoledWall/ShootTwoHoledWall.pml#1417620267846</inTransitions>
    <outTransitions>TwoHoledWall/ShootTwoHoledWall.pml#1417620285804</outTransitions>
    <outTransitions>TwoHoledWall/ShootTwoHoledWall.pml#1417620327753</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1417620275486" name="NewSuccessState" comment="">
    <inTransitions>TwoHoledWall/ShootTwoHoledWall.pml#1417620285804</inTransitions>
  </states>
  <transitions id="1417620267846" name="MISSING_NAME" comment="haveBall" msg="">
    <preCondition id="1417620269159" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>TwoHoledWall/ShootTwoHoledWall.pml#1417620209050</inState>
    <outState>TwoHoledWall/ShootTwoHoledWall.pml#1417620225739</outState>
  </transitions>
  <transitions id="1417620285804" name="MISSING_NAME" comment="anyChildSuccess" msg="">
    <preCondition id="1417620286821" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>TwoHoledWall/ShootTwoHoledWall.pml#1417620225739</inState>
    <outState>TwoHoledWall/ShootTwoHoledWall.pml#1417620275486</outState>
  </transitions>
  <transitions id="1417620327753" name="MISSING_NAME" comment="notHaveBall" msg="">
    <preCondition id="1417620329181" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>TwoHoledWall/ShootTwoHoledWall.pml#1417620225739</inState>
    <outState>TwoHoledWall/ShootTwoHoledWall.pml#1417620209050</outState>
  </transitions>
  <entryPoints id="1417620209051" name="MISSING_NAME" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>taskrepository.tsk#1225112227903</task>
    <state>TwoHoledWall/ShootTwoHoledWall.pml#1417620209050</state>
  </entryPoints>
</alica:Plan>
