<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1519032290449" name="InitialForwardBackward" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/TestPlans/DribbleTestMOS" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1519032339541" name="ToRootPoint" comment="" entryPoint="1519032339542">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517050799588</plans>
    <outTransitions>#1519033634738</outTransitions>
  </states>
  <states id="1519032827198" name="WaitForBall" comment="">
    <inTransitions>#1519033634738</inTransitions>
    <outTransitions>#1519033636769</outTransitions>
  </states>
  <states id="1519032838047" name="WaitShortly" comment="">
    <inTransitions>#1519033636769</inTransitions>
    <outTransitions>#1519033637789</outTransitions>
  </states>
  <states id="1519033439526" name="Forward1" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1519033565478</plans>
    <inTransitions>#1519033637789</inTransitions>
    <outTransitions>#1519033797644</outTransitions>
  </states>
  <states id="1519033689526" name="WaitForBall2" comment="">
    <inTransitions>#1519033799295</inTransitions>
    <outTransitions>#1519033800142</outTransitions>
  </states>
  <states id="1519033691856" name="WaitShortly2" comment="">
    <inTransitions>#1519033800142</inTransitions>
    <outTransitions>#1519033800920</outTransitions>
  </states>
  <states id="1519033693855" name="Backward1" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveToPoint.beh#1519034160470</plans>
    <inTransitions>#1519033800920</inTransitions>
    <outTransitions>#1519033801925</outTransitions>
  </states>
  <states id="1519033695455" name="ToRoot2" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517050799588</plans>
    <inTransitions>#1519033797644</inTransitions>
    <outTransitions>#1519033799295</outTransitions>
  </states>
  <states id="1519033697834" name="ToRoot3" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517050799588</plans>
    <inTransitions>#1519033801925</inTransitions>
    <outTransitions>#1519033803010</outTransitions>
  </states>
  <states id="1519033700396" name="WaitForBall3" comment="">
    <inTransitions>#1519033803010</inTransitions>
    <outTransitions>#1519033803860</outTransitions>
  </states>
  <states id="1519033702534" name="WaitShortly3" comment="">
    <inTransitions>#1519033803860</inTransitions>
    <outTransitions>#1519033805249</outTransitions>
  </states>
  <states id="1519033705513" name="Forward2" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveToPoint.beh#1442921078802</plans>
    <inTransitions>#1519033805249</inTransitions>
    <outTransitions>#1519033806370</outTransitions>
  </states>
  <states id="1519033713182" name="ToRoot4" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517050799588</plans>
    <inTransitions>#1519033806370</inTransitions>
    <outTransitions>#1519033807623</outTransitions>
  </states>
  <states id="1519033715231" name="WaitForBall4" comment="">
    <inTransitions>#1519033807623</inTransitions>
    <outTransitions>#1519033808722</outTransitions>
  </states>
  <states id="1519033717576" name="WaitShortly4" comment="">
    <inTransitions>#1519033808722</inTransitions>
    <outTransitions>#1519033809793</outTransitions>
  </states>
  <states id="1519033719721" name="Backward2" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveToPoint.beh#1519034188175</plans>
    <inTransitions>#1519033809793</inTransitions>
    <outTransitions>#1519033810675</outTransitions>
  </states>
  <states id="1519033721755" name="ToRoot5" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517050799588</plans>
    <inTransitions>#1519033810675</inTransitions>
    <outTransitions>#1519033811729</outTransitions>
  </states>
  <states id="1519033724271" name="WaitForBall5" comment="">
    <inTransitions>#1519033811729</inTransitions>
    <outTransitions>#1519033812817</outTransitions>
  </states>
  <states id="1519033726450" name="WaitShortly5" comment="">
    <inTransitions>#1519033812817</inTransitions>
    <outTransitions>#1519033814009</outTransitions>
  </states>
  <states id="1519033728667" name="Forward3" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveToPoint.beh#1519034199080</plans>
    <inTransitions>#1519033814009</inTransitions>
    <outTransitions>#1519033815155</outTransitions>
  </states>
  <states id="1519033730767" name="ToRoot6" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517050799588</plans>
    <inTransitions>#1519033815155</inTransitions>
    <outTransitions>#1519033816226</outTransitions>
  </states>
  <states id="1519033733344" name="WaitForBall6" comment="">
    <inTransitions>#1519033816226</inTransitions>
    <outTransitions>#1519033817166</outTransitions>
  </states>
  <states id="1519033735835" name="WaitShortly6" comment="">
    <inTransitions>#1519033817166</inTransitions>
    <outTransitions>#1519033818148</outTransitions>
  </states>
  <states id="1519033738336" name="Backward3" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveToPoint.beh#1519034210928</plans>
    <inTransitions>#1519033818148</inTransitions>
    <outTransitions>#1519033819752</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1519033778279" name="finished" comment="">
    <inTransitions>#1519033819752</inTransitions>
  </states>
  <transitions id="1519033634738" name="MISSING_NAME" comment="success" msg="">
    <preCondition id="1519033636553" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1519032339541</inState>
    <outState>#1519032827198</outState>
  </transitions>
  <transitions id="1519033636769" name="MISSING_NAME" comment="hasBall" msg="">
    <preCondition id="1519033637686" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1519032827198</inState>
    <outState>#1519032838047</outState>
  </transitions>
  <transitions id="1519033637789" name="MISSING_NAME" comment="waited2secs" msg="">
    <preCondition id="1519033638861" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1519032838047</inState>
    <outState>#1519033439526</outState>
  </transitions>
  <transitions id="1519033797644" name="MISSING_NAME" comment="success" msg="">
    <preCondition id="1519033798908" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1519033439526</inState>
    <outState>#1519033695455</outState>
  </transitions>
  <transitions id="1519033799295" name="MISSING_NAME" comment="success" msg="">
    <preCondition id="1519033800013" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1519033695455</inState>
    <outState>#1519033689526</outState>
  </transitions>
  <transitions id="1519033800142" name="MISSING_NAME" comment="hasBall" msg="">
    <preCondition id="1519033800793" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1519033689526</inState>
    <outState>#1519033691856</outState>
  </transitions>
  <transitions id="1519033800920" name="MISSING_NAME" comment="wait2secs" msg="">
    <preCondition id="1519033801788" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1519033691856</inState>
    <outState>#1519033693855</outState>
  </transitions>
  <transitions id="1519033801925" name="MISSING_NAME" comment="success" msg="">
    <preCondition id="1519033802879" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1519033693855</inState>
    <outState>#1519033697834</outState>
  </transitions>
  <transitions id="1519033803010" name="MISSING_NAME" comment="success" msg="">
    <preCondition id="1519033803756" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1519033697834</inState>
    <outState>#1519033700396</outState>
  </transitions>
  <transitions id="1519033803860" name="MISSING_NAME" comment="hasBall" msg="">
    <preCondition id="1519033804822" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1519033700396</inState>
    <outState>#1519033702534</outState>
  </transitions>
  <transitions id="1519033805249" name="MISSING_NAME" comment="waited2secs" msg="">
    <preCondition id="1519033806168" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1519033702534</inState>
    <outState>#1519033705513</outState>
  </transitions>
  <transitions id="1519033806370" name="MISSING_NAME" comment="success" msg="">
    <preCondition id="1519033807440" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1519033705513</inState>
    <outState>#1519033713182</outState>
  </transitions>
  <transitions id="1519033807623" name="MISSING_NAME" comment="success" msg="">
    <preCondition id="1519033808463" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1519033713182</inState>
    <outState>#1519033715231</outState>
  </transitions>
  <transitions id="1519033808722" name="MISSING_NAME" comment="hasBall" msg="">
    <preCondition id="1519033809424" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1519033715231</inState>
    <outState>#1519033717576</outState>
  </transitions>
  <transitions id="1519033809793" name="MISSING_NAME" comment="waited2secs" msg="">
    <preCondition id="1519033810484" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1519033717576</inState>
    <outState>#1519033719721</outState>
  </transitions>
  <transitions id="1519033810675" name="MISSING_NAME" comment="success" msg="">
    <preCondition id="1519033811584" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1519033719721</inState>
    <outState>#1519033721755</outState>
  </transitions>
  <transitions id="1519033811729" name="MISSING_NAME" comment="success" msg="">
    <preCondition id="1519033812683" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1519033721755</inState>
    <outState>#1519033724271</outState>
  </transitions>
  <transitions id="1519033812817" name="MISSING_NAME" comment="hasBall" msg="">
    <preCondition id="1519033813705" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1519033724271</inState>
    <outState>#1519033726450</outState>
  </transitions>
  <transitions id="1519033814009" name="MISSING_NAME" comment="waited2secs" msg="">
    <preCondition id="1519033814809" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1519033726450</inState>
    <outState>#1519033728667</outState>
  </transitions>
  <transitions id="1519033815155" name="MISSING_NAME" comment="success" msg="">
    <preCondition id="1519033816018" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1519033728667</inState>
    <outState>#1519033730767</outState>
  </transitions>
  <transitions id="1519033816226" name="MISSING_NAME" comment="success" msg="">
    <preCondition id="1519033817036" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1519033730767</inState>
    <outState>#1519033733344</outState>
  </transitions>
  <transitions id="1519033817166" name="MISSING_NAME" comment="hasBall" msg="">
    <preCondition id="1519033817980" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1519033733344</inState>
    <outState>#1519033735835</outState>
  </transitions>
  <transitions id="1519033818148" name="MISSING_NAME" comment="waited2secs" msg="">
    <preCondition id="1519033819335" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1519033735835</inState>
    <outState>#1519033738336</outState>
  </transitions>
  <transitions id="1519033819752" name="MISSING_NAME" comment="success" msg="">
    <preCondition id="1519033820671" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1519033738336</inState>
    <outState>#1519033778279</outState>
  </transitions>
  <entryPoints id="1519032339542" name="MISSING_NAME" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1519032339541</state>
  </entryPoints>
</alica:Plan>
