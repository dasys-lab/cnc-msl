<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1426695119330" name="Parking" comment="" destinationPath="Plans/GameStrategy/Other" priority="0.0" minCardinality="0" maxCardinality="2147483647" masterPlan="false" utilityFunction="" utilityThreshold="0.1">
  <states id="1426695119331" name="Parking" comment="" entryPoint="1426695119332">
    <plans xsi:type="alica:Behaviour">GenericBehaviours/Parking.beh#1429111623710</plans>
    <outTransitions>GameStrategy/Other/Parking.pml#1429111676838</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1429111673933" name="NewSuccessState" comment="">
    <inTransitions>GameStrategy/Other/Parking.pml#1429111676838</inTransitions>
  </states>
  <transitions id="1429111676838" name="MISSING_NAME" comment="anyChildsuccess" msg="">
    <preCondition id="1429111678112" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>GameStrategy/Other/Parking.pml#1426695119331</inState>
    <outState>GameStrategy/Other/Parking.pml#1429111673933</outState>
  </transitions>
  <entryPoints id="1426695119332" name="MISSING_NAME" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>taskrepository.tsk#1225112227903</task>
    <state>GameStrategy/Other/Parking.pml#1426695119331</state>
  </entryPoints>
</alica:Plan>
