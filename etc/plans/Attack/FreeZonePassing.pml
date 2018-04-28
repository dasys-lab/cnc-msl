<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1508950989519" name="FreeZonePassing" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Attack" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <conditions xsi:type="alica:RuntimeCondition" id="1508951693560" name="NewRuntimeCondition" comment="" conditionString="" pluginName="DefaultPlugin">
    <vars>#1508951697442</vars>
    <vars>#1508951702562</vars>
  </conditions>
  <vars id="1508951697442" name="x" comment="" Type=""/>
  <vars id="1508951702562" name="y" comment="" Type=""/>
  <states id="1508950989520" name="Passing" comment="" entryPoint="1508950989521">
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <plans xsi:type="alica:BehaviourConfiguration">PassIntoFreeZone.beh#1508951680585</plans>
  </states>
  <entryPoints id="1508950989521" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1508950989520</state>
  </entryPoints>
</alica:Plan>
