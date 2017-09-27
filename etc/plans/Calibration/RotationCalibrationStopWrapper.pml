<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1491404851815" name="RotationCalibrationStopWrapper" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <conditions xsi:type="alica:RuntimeCondition" id="1491404930441" name="NewRuntimeCondition" comment="" conditionString="" pluginName="DefaultPlugin"/>
  <states id="1491404851816" name="Run" comment="" entryPoint="1491404851817">
    <plans xsi:type="alica:Plan">RotationCalibration.pml#1467396347588</plans>
  </states>
  <entryPoints id="1491404851817" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1491404851816</state>
  </entryPoints>
</alica:Plan>
