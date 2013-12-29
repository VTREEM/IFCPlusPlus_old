/* -*-c++-*- IfcPlusPlus - www.ifcplusplus.com - Copyright (C) 2011 Fabian Gerold
*
* This library is open source and may be redistributed and/or modified under  
* the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or 
* (at your option) any later version.  The full license is in LICENSE file
* included with this distribution, and on the openscenegraph.org website.
* 
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
* OpenSceneGraph Public License for more details.
*/

#include <vector>
#include <map>
#include <string>
#include "ifcpp/IfcPPTypes.h"
#include "ifcpp/IfcPPTypeEnums.h"
#include "ifcpp/model/IfcPPException.h"

shared_ptr<IfcPPType> createIfcPPType( const IfcPPTypeEnum type_enum, const std::string& arg, const std::map<int,shared_ptr<IfcPPEntity> >& map_entities )
{
	switch( type_enum )
	{
		case IFCABSORBEDDOSEMEASURE:return IfcAbsorbedDoseMeasure::createObjectFromStepData( arg );
		case IFCACCELERATIONMEASURE:return IfcAccelerationMeasure::createObjectFromStepData( arg );
		case IFCACTIONREQUESTTYPEENUM:return IfcActionRequestTypeEnum::createObjectFromStepData( arg );
		case IFCACTIONSOURCETYPEENUM:return IfcActionSourceTypeEnum::createObjectFromStepData( arg );
		case IFCACTIONTYPEENUM:return IfcActionTypeEnum::createObjectFromStepData( arg );
		case IFCACTUATORTYPEENUM:return IfcActuatorTypeEnum::createObjectFromStepData( arg );
		case IFCADDRESSTYPEENUM:return IfcAddressTypeEnum::createObjectFromStepData( arg );
		case IFCAIRTERMINALBOXTYPEENUM:return IfcAirTerminalBoxTypeEnum::createObjectFromStepData( arg );
		case IFCAIRTERMINALTYPEENUM:return IfcAirTerminalTypeEnum::createObjectFromStepData( arg );
		case IFCAIRTOAIRHEATRECOVERYTYPEENUM:return IfcAirToAirHeatRecoveryTypeEnum::createObjectFromStepData( arg );
		case IFCALARMTYPEENUM:return IfcAlarmTypeEnum::createObjectFromStepData( arg );
		case IFCAMOUNTOFSUBSTANCEMEASURE:return IfcAmountOfSubstanceMeasure::createObjectFromStepData( arg );
		case IFCANALYSISMODELTYPEENUM:return IfcAnalysisModelTypeEnum::createObjectFromStepData( arg );
		case IFCANALYSISTHEORYTYPEENUM:return IfcAnalysisTheoryTypeEnum::createObjectFromStepData( arg );
		case IFCANGULARVELOCITYMEASURE:return IfcAngularVelocityMeasure::createObjectFromStepData( arg );
		case IFCAREADENSITYMEASURE:return IfcAreaDensityMeasure::createObjectFromStepData( arg );
		case IFCAREAMEASURE:return IfcAreaMeasure::createObjectFromStepData( arg );
		case IFCARITHMETICOPERATORENUM:return IfcArithmeticOperatorEnum::createObjectFromStepData( arg );
		case IFCASSEMBLYPLACEENUM:return IfcAssemblyPlaceEnum::createObjectFromStepData( arg );
		case IFCAUDIOVISUALAPPLIANCETYPEENUM:return IfcAudioVisualApplianceTypeEnum::createObjectFromStepData( arg );
		case IFCBSPLINECURVEFORM:return IfcBSplineCurveForm::createObjectFromStepData( arg );
		case IFCBSPLINESURFACEFORM:return IfcBSplineSurfaceForm::createObjectFromStepData( arg );
		case IFCBEAMTYPEENUM:return IfcBeamTypeEnum::createObjectFromStepData( arg );
		case IFCBENCHMARKENUM:return IfcBenchmarkEnum::createObjectFromStepData( arg );
		case IFCBOILERTYPEENUM:return IfcBoilerTypeEnum::createObjectFromStepData( arg );
		case IFCBOOLEAN:return IfcBoolean::createObjectFromStepData( arg );
		case IFCBOOLEANOPERATOR:return IfcBooleanOperator::createObjectFromStepData( arg );
		case IFCBOXALIGNMENT:return IfcBoxAlignment::createObjectFromStepData( arg );
		case IFCBUILDINGELEMENTPARTTYPEENUM:return IfcBuildingElementPartTypeEnum::createObjectFromStepData( arg );
		case IFCBUILDINGELEMENTPROXYTYPEENUM:return IfcBuildingElementProxyTypeEnum::createObjectFromStepData( arg );
		case IFCBUILDINGSYSTEMTYPEENUM:return IfcBuildingSystemTypeEnum::createObjectFromStepData( arg );
		case IFCBURNERTYPEENUM:return IfcBurnerTypeEnum::createObjectFromStepData( arg );
		case IFCCABLECARRIERFITTINGTYPEENUM:return IfcCableCarrierFittingTypeEnum::createObjectFromStepData( arg );
		case IFCCABLECARRIERSEGMENTTYPEENUM:return IfcCableCarrierSegmentTypeEnum::createObjectFromStepData( arg );
		case IFCCABLEFITTINGTYPEENUM:return IfcCableFittingTypeEnum::createObjectFromStepData( arg );
		case IFCCABLESEGMENTTYPEENUM:return IfcCableSegmentTypeEnum::createObjectFromStepData( arg );
		case IFCCARDINALPOINTREFERENCE:return IfcCardinalPointReference::createObjectFromStepData( arg );
		case IFCCHANGEACTIONENUM:return IfcChangeActionEnum::createObjectFromStepData( arg );
		case IFCCHILLERTYPEENUM:return IfcChillerTypeEnum::createObjectFromStepData( arg );
		case IFCCHIMNEYTYPEENUM:return IfcChimneyTypeEnum::createObjectFromStepData( arg );
		case IFCCOILTYPEENUM:return IfcCoilTypeEnum::createObjectFromStepData( arg );
		case IFCCOLUMNTYPEENUM:return IfcColumnTypeEnum::createObjectFromStepData( arg );
		case IFCCOMMUNICATIONSAPPLIANCETYPEENUM:return IfcCommunicationsApplianceTypeEnum::createObjectFromStepData( arg );
		case IFCCOMPLEXNUMBER:return IfcComplexNumber::createObjectFromStepData( arg );
		case IFCCOMPLEXPROPERTYTEMPLATETYPEENUM:return IfcComplexPropertyTemplateTypeEnum::createObjectFromStepData( arg );
		case IFCCOMPOUNDPLANEANGLEMEASURE:return IfcCompoundPlaneAngleMeasure::createObjectFromStepData( arg );
		case IFCCOMPRESSORTYPEENUM:return IfcCompressorTypeEnum::createObjectFromStepData( arg );
		case IFCCONDENSERTYPEENUM:return IfcCondenserTypeEnum::createObjectFromStepData( arg );
		case IFCCONNECTIONTYPEENUM:return IfcConnectionTypeEnum::createObjectFromStepData( arg );
		case IFCCONSTRAINTENUM:return IfcConstraintEnum::createObjectFromStepData( arg );
		case IFCCONSTRUCTIONEQUIPMENTRESOURCETYPEENUM:return IfcConstructionEquipmentResourceTypeEnum::createObjectFromStepData( arg );
		case IFCCONSTRUCTIONMATERIALRESOURCETYPEENUM:return IfcConstructionMaterialResourceTypeEnum::createObjectFromStepData( arg );
		case IFCCONSTRUCTIONPRODUCTRESOURCETYPEENUM:return IfcConstructionProductResourceTypeEnum::createObjectFromStepData( arg );
		case IFCCONTEXTDEPENDENTMEASURE:return IfcContextDependentMeasure::createObjectFromStepData( arg );
		case IFCCONTROLLERTYPEENUM:return IfcControllerTypeEnum::createObjectFromStepData( arg );
		case IFCCOOLEDBEAMTYPEENUM:return IfcCooledBeamTypeEnum::createObjectFromStepData( arg );
		case IFCCOOLINGTOWERTYPEENUM:return IfcCoolingTowerTypeEnum::createObjectFromStepData( arg );
		case IFCCOSTITEMTYPEENUM:return IfcCostItemTypeEnum::createObjectFromStepData( arg );
		case IFCCOSTSCHEDULETYPEENUM:return IfcCostScheduleTypeEnum::createObjectFromStepData( arg );
		case IFCCOUNTMEASURE:return IfcCountMeasure::createObjectFromStepData( arg );
		case IFCCOVERINGTYPEENUM:return IfcCoveringTypeEnum::createObjectFromStepData( arg );
		case IFCCREWRESOURCETYPEENUM:return IfcCrewResourceTypeEnum::createObjectFromStepData( arg );
		case IFCCURTAINWALLTYPEENUM:return IfcCurtainWallTypeEnum::createObjectFromStepData( arg );
		case IFCCURVATUREMEASURE:return IfcCurvatureMeasure::createObjectFromStepData( arg );
		case IFCCURVEINTERPOLATIONENUM:return IfcCurveInterpolationEnum::createObjectFromStepData( arg );
		case IFCDAMPERTYPEENUM:return IfcDamperTypeEnum::createObjectFromStepData( arg );
		case IFCDATAORIGINENUM:return IfcDataOriginEnum::createObjectFromStepData( arg );
		case IFCDATE:return IfcDate::createObjectFromStepData( arg );
		case IFCDATETIME:return IfcDateTime::createObjectFromStepData( arg );
		case IFCDAYINMONTHNUMBER:return IfcDayInMonthNumber::createObjectFromStepData( arg );
		case IFCDAYINWEEKNUMBER:return IfcDayInWeekNumber::createObjectFromStepData( arg );
		case IFCDERIVEDUNITENUM:return IfcDerivedUnitEnum::createObjectFromStepData( arg );
		case IFCDESCRIPTIVEMEASURE:return IfcDescriptiveMeasure::createObjectFromStepData( arg );
		case IFCDIMENSIONCOUNT:return IfcDimensionCount::createObjectFromStepData( arg );
		case IFCDIRECTIONSENSEENUM:return IfcDirectionSenseEnum::createObjectFromStepData( arg );
		case IFCDISCRETEACCESSORYTYPEENUM:return IfcDiscreteAccessoryTypeEnum::createObjectFromStepData( arg );
		case IFCDISTRIBUTIONCHAMBERELEMENTTYPEENUM:return IfcDistributionChamberElementTypeEnum::createObjectFromStepData( arg );
		case IFCDISTRIBUTIONPORTTYPEENUM:return IfcDistributionPortTypeEnum::createObjectFromStepData( arg );
		case IFCDISTRIBUTIONSYSTEMENUM:return IfcDistributionSystemEnum::createObjectFromStepData( arg );
		case IFCDOCUMENTCONFIDENTIALITYENUM:return IfcDocumentConfidentialityEnum::createObjectFromStepData( arg );
		case IFCDOCUMENTSTATUSENUM:return IfcDocumentStatusEnum::createObjectFromStepData( arg );
		case IFCDOORPANELOPERATIONENUM:return IfcDoorPanelOperationEnum::createObjectFromStepData( arg );
		case IFCDOORPANELPOSITIONENUM:return IfcDoorPanelPositionEnum::createObjectFromStepData( arg );
		case IFCDOORSTYLECONSTRUCTIONENUM:return IfcDoorStyleConstructionEnum::createObjectFromStepData( arg );
		case IFCDOORSTYLEOPERATIONENUM:return IfcDoorStyleOperationEnum::createObjectFromStepData( arg );
		case IFCDOORTYPEENUM:return IfcDoorTypeEnum::createObjectFromStepData( arg );
		case IFCDOORTYPEOPERATIONENUM:return IfcDoorTypeOperationEnum::createObjectFromStepData( arg );
		case IFCDOSEEQUIVALENTMEASURE:return IfcDoseEquivalentMeasure::createObjectFromStepData( arg );
		case IFCDUCTFITTINGTYPEENUM:return IfcDuctFittingTypeEnum::createObjectFromStepData( arg );
		case IFCDUCTSEGMENTTYPEENUM:return IfcDuctSegmentTypeEnum::createObjectFromStepData( arg );
		case IFCDUCTSILENCERTYPEENUM:return IfcDuctSilencerTypeEnum::createObjectFromStepData( arg );
		case IFCDURATION:return IfcDuration::createObjectFromStepData( arg );
		case IFCDYNAMICVISCOSITYMEASURE:return IfcDynamicViscosityMeasure::createObjectFromStepData( arg );
		case IFCELECTRICAPPLIANCETYPEENUM:return IfcElectricApplianceTypeEnum::createObjectFromStepData( arg );
		case IFCELECTRICCAPACITANCEMEASURE:return IfcElectricCapacitanceMeasure::createObjectFromStepData( arg );
		case IFCELECTRICCHARGEMEASURE:return IfcElectricChargeMeasure::createObjectFromStepData( arg );
		case IFCELECTRICCONDUCTANCEMEASURE:return IfcElectricConductanceMeasure::createObjectFromStepData( arg );
		case IFCELECTRICCURRENTMEASURE:return IfcElectricCurrentMeasure::createObjectFromStepData( arg );
		case IFCELECTRICDISTRIBUTIONBOARDTYPEENUM:return IfcElectricDistributionBoardTypeEnum::createObjectFromStepData( arg );
		case IFCELECTRICFLOWSTORAGEDEVICETYPEENUM:return IfcElectricFlowStorageDeviceTypeEnum::createObjectFromStepData( arg );
		case IFCELECTRICGENERATORTYPEENUM:return IfcElectricGeneratorTypeEnum::createObjectFromStepData( arg );
		case IFCELECTRICMOTORTYPEENUM:return IfcElectricMotorTypeEnum::createObjectFromStepData( arg );
		case IFCELECTRICRESISTANCEMEASURE:return IfcElectricResistanceMeasure::createObjectFromStepData( arg );
		case IFCELECTRICTIMECONTROLTYPEENUM:return IfcElectricTimeControlTypeEnum::createObjectFromStepData( arg );
		case IFCELECTRICVOLTAGEMEASURE:return IfcElectricVoltageMeasure::createObjectFromStepData( arg );
		case IFCELEMENTASSEMBLYTYPEENUM:return IfcElementAssemblyTypeEnum::createObjectFromStepData( arg );
		case IFCELEMENTCOMPOSITIONENUM:return IfcElementCompositionEnum::createObjectFromStepData( arg );
		case IFCENERGYMEASURE:return IfcEnergyMeasure::createObjectFromStepData( arg );
		case IFCENGINETYPEENUM:return IfcEngineTypeEnum::createObjectFromStepData( arg );
		case IFCEVAPORATIVECOOLERTYPEENUM:return IfcEvaporativeCoolerTypeEnum::createObjectFromStepData( arg );
		case IFCEVAPORATORTYPEENUM:return IfcEvaporatorTypeEnum::createObjectFromStepData( arg );
		case IFCEVENTTRIGGERTYPEENUM:return IfcEventTriggerTypeEnum::createObjectFromStepData( arg );
		case IFCEVENTTYPEENUM:return IfcEventTypeEnum::createObjectFromStepData( arg );
		case IFCEXTERNALSPATIALELEMENTTYPEENUM:return IfcExternalSpatialElementTypeEnum::createObjectFromStepData( arg );
		case IFCFANTYPEENUM:return IfcFanTypeEnum::createObjectFromStepData( arg );
		case IFCFASTENERTYPEENUM:return IfcFastenerTypeEnum::createObjectFromStepData( arg );
		case IFCFILTERTYPEENUM:return IfcFilterTypeEnum::createObjectFromStepData( arg );
		case IFCFIRESUPPRESSIONTERMINALTYPEENUM:return IfcFireSuppressionTerminalTypeEnum::createObjectFromStepData( arg );
		case IFCFLOWDIRECTIONENUM:return IfcFlowDirectionEnum::createObjectFromStepData( arg );
		case IFCFLOWINSTRUMENTTYPEENUM:return IfcFlowInstrumentTypeEnum::createObjectFromStepData( arg );
		case IFCFLOWMETERTYPEENUM:return IfcFlowMeterTypeEnum::createObjectFromStepData( arg );
		case IFCFONTSTYLE:return IfcFontStyle::createObjectFromStepData( arg );
		case IFCFONTVARIANT:return IfcFontVariant::createObjectFromStepData( arg );
		case IFCFONTWEIGHT:return IfcFontWeight::createObjectFromStepData( arg );
		case IFCFOOTINGTYPEENUM:return IfcFootingTypeEnum::createObjectFromStepData( arg );
		case IFCFORCEMEASURE:return IfcForceMeasure::createObjectFromStepData( arg );
		case IFCFREQUENCYMEASURE:return IfcFrequencyMeasure::createObjectFromStepData( arg );
		case IFCFURNITURETYPEENUM:return IfcFurnitureTypeEnum::createObjectFromStepData( arg );
		case IFCGEOGRAPHICELEMENTTYPEENUM:return IfcGeographicElementTypeEnum::createObjectFromStepData( arg );
		case IFCGEOMETRICPROJECTIONENUM:return IfcGeometricProjectionEnum::createObjectFromStepData( arg );
		case IFCGLOBALORLOCALENUM:return IfcGlobalOrLocalEnum::createObjectFromStepData( arg );
		case IFCGLOBALLYUNIQUEID:return IfcGloballyUniqueId::createObjectFromStepData( arg );
		case IFCGRIDTYPEENUM:return IfcGridTypeEnum::createObjectFromStepData( arg );
		case IFCHEATEXCHANGERTYPEENUM:return IfcHeatExchangerTypeEnum::createObjectFromStepData( arg );
		case IFCHEATFLUXDENSITYMEASURE:return IfcHeatFluxDensityMeasure::createObjectFromStepData( arg );
		case IFCHEATINGVALUEMEASURE:return IfcHeatingValueMeasure::createObjectFromStepData( arg );
		case IFCHUMIDIFIERTYPEENUM:return IfcHumidifierTypeEnum::createObjectFromStepData( arg );
		case IFCIDENTIFIER:return IfcIdentifier::createObjectFromStepData( arg );
		case IFCILLUMINANCEMEASURE:return IfcIlluminanceMeasure::createObjectFromStepData( arg );
		case IFCINDUCTANCEMEASURE:return IfcInductanceMeasure::createObjectFromStepData( arg );
		case IFCINTEGER:return IfcInteger::createObjectFromStepData( arg );
		case IFCINTEGERCOUNTRATEMEASURE:return IfcIntegerCountRateMeasure::createObjectFromStepData( arg );
		case IFCINTERCEPTORTYPEENUM:return IfcInterceptorTypeEnum::createObjectFromStepData( arg );
		case IFCINTERNALOREXTERNALENUM:return IfcInternalOrExternalEnum::createObjectFromStepData( arg );
		case IFCINVENTORYTYPEENUM:return IfcInventoryTypeEnum::createObjectFromStepData( arg );
		case IFCIONCONCENTRATIONMEASURE:return IfcIonConcentrationMeasure::createObjectFromStepData( arg );
		case IFCISOTHERMALMOISTURECAPACITYMEASURE:return IfcIsothermalMoistureCapacityMeasure::createObjectFromStepData( arg );
		case IFCJUNCTIONBOXTYPEENUM:return IfcJunctionBoxTypeEnum::createObjectFromStepData( arg );
		case IFCKINEMATICVISCOSITYMEASURE:return IfcKinematicViscosityMeasure::createObjectFromStepData( arg );
		case IFCKNOTTYPE:return IfcKnotType::createObjectFromStepData( arg );
		case IFCLABEL:return IfcLabel::createObjectFromStepData( arg );
		case IFCLABORRESOURCETYPEENUM:return IfcLaborResourceTypeEnum::createObjectFromStepData( arg );
		case IFCLAMPTYPEENUM:return IfcLampTypeEnum::createObjectFromStepData( arg );
		case IFCLANGUAGEID:return IfcLanguageId::createObjectFromStepData( arg );
		case IFCLAYERSETDIRECTIONENUM:return IfcLayerSetDirectionEnum::createObjectFromStepData( arg );
		case IFCLENGTHMEASURE:return IfcLengthMeasure::createObjectFromStepData( arg );
		case IFCLIGHTDISTRIBUTIONCURVEENUM:return IfcLightDistributionCurveEnum::createObjectFromStepData( arg );
		case IFCLIGHTEMISSIONSOURCEENUM:return IfcLightEmissionSourceEnum::createObjectFromStepData( arg );
		case IFCLIGHTFIXTURETYPEENUM:return IfcLightFixtureTypeEnum::createObjectFromStepData( arg );
		case IFCLINEARFORCEMEASURE:return IfcLinearForceMeasure::createObjectFromStepData( arg );
		case IFCLINEARMOMENTMEASURE:return IfcLinearMomentMeasure::createObjectFromStepData( arg );
		case IFCLINEARSTIFFNESSMEASURE:return IfcLinearStiffnessMeasure::createObjectFromStepData( arg );
		case IFCLINEARVELOCITYMEASURE:return IfcLinearVelocityMeasure::createObjectFromStepData( arg );
		case IFCLOADGROUPTYPEENUM:return IfcLoadGroupTypeEnum::createObjectFromStepData( arg );
		case IFCLOGICAL:return IfcLogical::createObjectFromStepData( arg );
		case IFCLOGICALOPERATORENUM:return IfcLogicalOperatorEnum::createObjectFromStepData( arg );
		case IFCLUMINOUSFLUXMEASURE:return IfcLuminousFluxMeasure::createObjectFromStepData( arg );
		case IFCLUMINOUSINTENSITYDISTRIBUTIONMEASURE:return IfcLuminousIntensityDistributionMeasure::createObjectFromStepData( arg );
		case IFCLUMINOUSINTENSITYMEASURE:return IfcLuminousIntensityMeasure::createObjectFromStepData( arg );
		case IFCMAGNETICFLUXDENSITYMEASURE:return IfcMagneticFluxDensityMeasure::createObjectFromStepData( arg );
		case IFCMAGNETICFLUXMEASURE:return IfcMagneticFluxMeasure::createObjectFromStepData( arg );
		case IFCMASSDENSITYMEASURE:return IfcMassDensityMeasure::createObjectFromStepData( arg );
		case IFCMASSFLOWRATEMEASURE:return IfcMassFlowRateMeasure::createObjectFromStepData( arg );
		case IFCMASSMEASURE:return IfcMassMeasure::createObjectFromStepData( arg );
		case IFCMASSPERLENGTHMEASURE:return IfcMassPerLengthMeasure::createObjectFromStepData( arg );
		case IFCMECHANICALFASTENERTYPEENUM:return IfcMechanicalFastenerTypeEnum::createObjectFromStepData( arg );
		case IFCMEDICALDEVICETYPEENUM:return IfcMedicalDeviceTypeEnum::createObjectFromStepData( arg );
		case IFCMEMBERTYPEENUM:return IfcMemberTypeEnum::createObjectFromStepData( arg );
		case IFCMODULUSOFELASTICITYMEASURE:return IfcModulusOfElasticityMeasure::createObjectFromStepData( arg );
		case IFCMODULUSOFLINEARSUBGRADEREACTIONMEASURE:return IfcModulusOfLinearSubgradeReactionMeasure::createObjectFromStepData( arg );
		case IFCMODULUSOFROTATIONALSUBGRADEREACTIONMEASURE:return IfcModulusOfRotationalSubgradeReactionMeasure::createObjectFromStepData( arg );
		case IFCMODULUSOFSUBGRADEREACTIONMEASURE:return IfcModulusOfSubgradeReactionMeasure::createObjectFromStepData( arg );
		case IFCMOISTUREDIFFUSIVITYMEASURE:return IfcMoistureDiffusivityMeasure::createObjectFromStepData( arg );
		case IFCMOLECULARWEIGHTMEASURE:return IfcMolecularWeightMeasure::createObjectFromStepData( arg );
		case IFCMOMENTOFINERTIAMEASURE:return IfcMomentOfInertiaMeasure::createObjectFromStepData( arg );
		case IFCMONETARYMEASURE:return IfcMonetaryMeasure::createObjectFromStepData( arg );
		case IFCMONTHINYEARNUMBER:return IfcMonthInYearNumber::createObjectFromStepData( arg );
		case IFCMOTORCONNECTIONTYPEENUM:return IfcMotorConnectionTypeEnum::createObjectFromStepData( arg );
		case IFCNONNEGATIVELENGTHMEASURE:return IfcNonNegativeLengthMeasure::createObjectFromStepData( arg );
		case IFCNORMALISEDRATIOMEASURE:return IfcNormalisedRatioMeasure::createObjectFromStepData( arg );
		case IFCNULLSTYLE:return IfcNullStyle::createObjectFromStepData( arg );
		case IFCNUMERICMEASURE:return IfcNumericMeasure::createObjectFromStepData( arg );
		case IFCOBJECTTYPEENUM:return IfcObjectTypeEnum::createObjectFromStepData( arg );
		case IFCOBJECTIVEENUM:return IfcObjectiveEnum::createObjectFromStepData( arg );
		case IFCOCCUPANTTYPEENUM:return IfcOccupantTypeEnum::createObjectFromStepData( arg );
		case IFCOPENINGELEMENTTYPEENUM:return IfcOpeningElementTypeEnum::createObjectFromStepData( arg );
		case IFCOUTLETTYPEENUM:return IfcOutletTypeEnum::createObjectFromStepData( arg );
		case IFCPHMEASURE:return IfcPHMeasure::createObjectFromStepData( arg );
		case IFCPARAMETERVALUE:return IfcParameterValue::createObjectFromStepData( arg );
		case IFCPERFORMANCEHISTORYTYPEENUM:return IfcPerformanceHistoryTypeEnum::createObjectFromStepData( arg );
		case IFCPERMEABLECOVERINGOPERATIONENUM:return IfcPermeableCoveringOperationEnum::createObjectFromStepData( arg );
		case IFCPERMITTYPEENUM:return IfcPermitTypeEnum::createObjectFromStepData( arg );
		case IFCPHYSICALORVIRTUALENUM:return IfcPhysicalOrVirtualEnum::createObjectFromStepData( arg );
		case IFCPILECONSTRUCTIONENUM:return IfcPileConstructionEnum::createObjectFromStepData( arg );
		case IFCPILETYPEENUM:return IfcPileTypeEnum::createObjectFromStepData( arg );
		case IFCPIPEFITTINGTYPEENUM:return IfcPipeFittingTypeEnum::createObjectFromStepData( arg );
		case IFCPIPESEGMENTTYPEENUM:return IfcPipeSegmentTypeEnum::createObjectFromStepData( arg );
		case IFCPLANARFORCEMEASURE:return IfcPlanarForceMeasure::createObjectFromStepData( arg );
		case IFCPLANEANGLEMEASURE:return IfcPlaneAngleMeasure::createObjectFromStepData( arg );
		case IFCPLATETYPEENUM:return IfcPlateTypeEnum::createObjectFromStepData( arg );
		case IFCPOSITIVELENGTHMEASURE:return IfcPositiveLengthMeasure::createObjectFromStepData( arg );
		case IFCPOSITIVEPLANEANGLEMEASURE:return IfcPositivePlaneAngleMeasure::createObjectFromStepData( arg );
		case IFCPOSITIVERATIOMEASURE:return IfcPositiveRatioMeasure::createObjectFromStepData( arg );
		case IFCPOWERMEASURE:return IfcPowerMeasure::createObjectFromStepData( arg );
		case IFCPRESENTABLETEXT:return IfcPresentableText::createObjectFromStepData( arg );
		case IFCPRESSUREMEASURE:return IfcPressureMeasure::createObjectFromStepData( arg );
		case IFCPROCEDURETYPEENUM:return IfcProcedureTypeEnum::createObjectFromStepData( arg );
		case IFCPROFILETYPEENUM:return IfcProfileTypeEnum::createObjectFromStepData( arg );
		case IFCPROJECTORDERTYPEENUM:return IfcProjectOrderTypeEnum::createObjectFromStepData( arg );
		case IFCPROJECTEDORTRUELENGTHENUM:return IfcProjectedOrTrueLengthEnum::createObjectFromStepData( arg );
		case IFCPROJECTIONELEMENTTYPEENUM:return IfcProjectionElementTypeEnum::createObjectFromStepData( arg );
		case IFCPROPERTYSETDEFINITIONSET:return IfcPropertySetDefinitionSet::createObjectFromStepData( arg, map_entities );
		case IFCPROPERTYSETTEMPLATETYPEENUM:return IfcPropertySetTemplateTypeEnum::createObjectFromStepData( arg );
		case IFCPROTECTIVEDEVICETRIPPINGUNITTYPEENUM:return IfcProtectiveDeviceTrippingUnitTypeEnum::createObjectFromStepData( arg );
		case IFCPROTECTIVEDEVICETYPEENUM:return IfcProtectiveDeviceTypeEnum::createObjectFromStepData( arg );
		case IFCPUMPTYPEENUM:return IfcPumpTypeEnum::createObjectFromStepData( arg );
		case IFCRADIOACTIVITYMEASURE:return IfcRadioActivityMeasure::createObjectFromStepData( arg );
		case IFCRAILINGTYPEENUM:return IfcRailingTypeEnum::createObjectFromStepData( arg );
		case IFCRAMPFLIGHTTYPEENUM:return IfcRampFlightTypeEnum::createObjectFromStepData( arg );
		case IFCRAMPTYPEENUM:return IfcRampTypeEnum::createObjectFromStepData( arg );
		case IFCRATIOMEASURE:return IfcRatioMeasure::createObjectFromStepData( arg );
		case IFCREAL:return IfcReal::createObjectFromStepData( arg );
		case IFCRECURRENCETYPEENUM:return IfcRecurrenceTypeEnum::createObjectFromStepData( arg );
		case IFCREFLECTANCEMETHODENUM:return IfcReflectanceMethodEnum::createObjectFromStepData( arg );
		case IFCREINFORCINGBARROLEENUM:return IfcReinforcingBarRoleEnum::createObjectFromStepData( arg );
		case IFCREINFORCINGBARSURFACEENUM:return IfcReinforcingBarSurfaceEnum::createObjectFromStepData( arg );
		case IFCREINFORCINGBARTYPEENUM:return IfcReinforcingBarTypeEnum::createObjectFromStepData( arg );
		case IFCREINFORCINGMESHTYPEENUM:return IfcReinforcingMeshTypeEnum::createObjectFromStepData( arg );
		case IFCROLEENUM:return IfcRoleEnum::createObjectFromStepData( arg );
		case IFCROOFTYPEENUM:return IfcRoofTypeEnum::createObjectFromStepData( arg );
		case IFCROTATIONALFREQUENCYMEASURE:return IfcRotationalFrequencyMeasure::createObjectFromStepData( arg );
		case IFCROTATIONALMASSMEASURE:return IfcRotationalMassMeasure::createObjectFromStepData( arg );
		case IFCROTATIONALSTIFFNESSMEASURE:return IfcRotationalStiffnessMeasure::createObjectFromStepData( arg );
		case IFCSIPREFIX:return IfcSIPrefix::createObjectFromStepData( arg );
		case IFCSIUNITNAME:return IfcSIUnitName::createObjectFromStepData( arg );
		case IFCSANITARYTERMINALTYPEENUM:return IfcSanitaryTerminalTypeEnum::createObjectFromStepData( arg );
		case IFCSECTIONMODULUSMEASURE:return IfcSectionModulusMeasure::createObjectFromStepData( arg );
		case IFCSECTIONTYPEENUM:return IfcSectionTypeEnum::createObjectFromStepData( arg );
		case IFCSECTIONALAREAINTEGRALMEASURE:return IfcSectionalAreaIntegralMeasure::createObjectFromStepData( arg );
		case IFCSENSORTYPEENUM:return IfcSensorTypeEnum::createObjectFromStepData( arg );
		case IFCSEQUENCEENUM:return IfcSequenceEnum::createObjectFromStepData( arg );
		case IFCSHADINGDEVICETYPEENUM:return IfcShadingDeviceTypeEnum::createObjectFromStepData( arg );
		case IFCSHEARMODULUSMEASURE:return IfcShearModulusMeasure::createObjectFromStepData( arg );
		case IFCSIMPLEPROPERTYTEMPLATETYPEENUM:return IfcSimplePropertyTemplateTypeEnum::createObjectFromStepData( arg );
		case IFCSLABTYPEENUM:return IfcSlabTypeEnum::createObjectFromStepData( arg );
		case IFCSOLARDEVICETYPEENUM:return IfcSolarDeviceTypeEnum::createObjectFromStepData( arg );
		case IFCSOLIDANGLEMEASURE:return IfcSolidAngleMeasure::createObjectFromStepData( arg );
		case IFCSOUNDPOWERLEVELMEASURE:return IfcSoundPowerLevelMeasure::createObjectFromStepData( arg );
		case IFCSOUNDPOWERMEASURE:return IfcSoundPowerMeasure::createObjectFromStepData( arg );
		case IFCSOUNDPRESSURELEVELMEASURE:return IfcSoundPressureLevelMeasure::createObjectFromStepData( arg );
		case IFCSOUNDPRESSUREMEASURE:return IfcSoundPressureMeasure::createObjectFromStepData( arg );
		case IFCSPACEHEATERTYPEENUM:return IfcSpaceHeaterTypeEnum::createObjectFromStepData( arg );
		case IFCSPACETYPEENUM:return IfcSpaceTypeEnum::createObjectFromStepData( arg );
		case IFCSPATIALZONETYPEENUM:return IfcSpatialZoneTypeEnum::createObjectFromStepData( arg );
		case IFCSPECIFICHEATCAPACITYMEASURE:return IfcSpecificHeatCapacityMeasure::createObjectFromStepData( arg );
		case IFCSPECULAREXPONENT:return IfcSpecularExponent::createObjectFromStepData( arg );
		case IFCSPECULARROUGHNESS:return IfcSpecularRoughness::createObjectFromStepData( arg );
		case IFCSTACKTERMINALTYPEENUM:return IfcStackTerminalTypeEnum::createObjectFromStepData( arg );
		case IFCSTAIRFLIGHTTYPEENUM:return IfcStairFlightTypeEnum::createObjectFromStepData( arg );
		case IFCSTAIRTYPEENUM:return IfcStairTypeEnum::createObjectFromStepData( arg );
		case IFCSTATEENUM:return IfcStateEnum::createObjectFromStepData( arg );
		case IFCSTRUCTURALCURVEACTIVITYTYPEENUM:return IfcStructuralCurveActivityTypeEnum::createObjectFromStepData( arg );
		case IFCSTRUCTURALCURVEMEMBERTYPEENUM:return IfcStructuralCurveMemberTypeEnum::createObjectFromStepData( arg );
		case IFCSTRUCTURALSURFACEACTIVITYTYPEENUM:return IfcStructuralSurfaceActivityTypeEnum::createObjectFromStepData( arg );
		case IFCSTRUCTURALSURFACEMEMBERTYPEENUM:return IfcStructuralSurfaceMemberTypeEnum::createObjectFromStepData( arg );
		case IFCSUBCONTRACTRESOURCETYPEENUM:return IfcSubContractResourceTypeEnum::createObjectFromStepData( arg );
		case IFCSURFACEFEATURETYPEENUM:return IfcSurfaceFeatureTypeEnum::createObjectFromStepData( arg );
		case IFCSURFACESIDE:return IfcSurfaceSide::createObjectFromStepData( arg );
		case IFCSWITCHINGDEVICETYPEENUM:return IfcSwitchingDeviceTypeEnum::createObjectFromStepData( arg );
		case IFCSYSTEMFURNITUREELEMENTTYPEENUM:return IfcSystemFurnitureElementTypeEnum::createObjectFromStepData( arg );
		case IFCTANKTYPEENUM:return IfcTankTypeEnum::createObjectFromStepData( arg );
		case IFCTASKDURATIONENUM:return IfcTaskDurationEnum::createObjectFromStepData( arg );
		case IFCTASKTYPEENUM:return IfcTaskTypeEnum::createObjectFromStepData( arg );
		case IFCTEMPERATUREGRADIENTMEASURE:return IfcTemperatureGradientMeasure::createObjectFromStepData( arg );
		case IFCTEMPERATURERATEOFCHANGEMEASURE:return IfcTemperatureRateOfChangeMeasure::createObjectFromStepData( arg );
		case IFCTENDONANCHORTYPEENUM:return IfcTendonAnchorTypeEnum::createObjectFromStepData( arg );
		case IFCTENDONTYPEENUM:return IfcTendonTypeEnum::createObjectFromStepData( arg );
		case IFCTEXT:return IfcText::createObjectFromStepData( arg );
		case IFCTEXTALIGNMENT:return IfcTextAlignment::createObjectFromStepData( arg );
		case IFCTEXTDECORATION:return IfcTextDecoration::createObjectFromStepData( arg );
		case IFCTEXTFONTNAME:return IfcTextFontName::createObjectFromStepData( arg );
		case IFCTEXTPATH:return IfcTextPath::createObjectFromStepData( arg );
		case IFCTEXTTRANSFORMATION:return IfcTextTransformation::createObjectFromStepData( arg );
		case IFCTHERMALADMITTANCEMEASURE:return IfcThermalAdmittanceMeasure::createObjectFromStepData( arg );
		case IFCTHERMALCONDUCTIVITYMEASURE:return IfcThermalConductivityMeasure::createObjectFromStepData( arg );
		case IFCTHERMALEXPANSIONCOEFFICIENTMEASURE:return IfcThermalExpansionCoefficientMeasure::createObjectFromStepData( arg );
		case IFCTHERMALRESISTANCEMEASURE:return IfcThermalResistanceMeasure::createObjectFromStepData( arg );
		case IFCTHERMALTRANSMITTANCEMEASURE:return IfcThermalTransmittanceMeasure::createObjectFromStepData( arg );
		case IFCTHERMODYNAMICTEMPERATUREMEASURE:return IfcThermodynamicTemperatureMeasure::createObjectFromStepData( arg );
		case IFCTIME:return IfcTime::createObjectFromStepData( arg );
		case IFCTIMEMEASURE:return IfcTimeMeasure::createObjectFromStepData( arg );
		case IFCTIMESERIESDATATYPEENUM:return IfcTimeSeriesDataTypeEnum::createObjectFromStepData( arg );
		case IFCTIMESTAMP:return IfcTimeStamp::createObjectFromStepData( arg );
		case IFCTORQUEMEASURE:return IfcTorqueMeasure::createObjectFromStepData( arg );
		case IFCTRANSFORMERTYPEENUM:return IfcTransformerTypeEnum::createObjectFromStepData( arg );
		case IFCTRANSITIONCODE:return IfcTransitionCode::createObjectFromStepData( arg );
		case IFCTRANSPORTELEMENTTYPEENUM:return IfcTransportElementTypeEnum::createObjectFromStepData( arg );
		case IFCTRIMMINGPREFERENCE:return IfcTrimmingPreference::createObjectFromStepData( arg );
		case IFCTUBEBUNDLETYPEENUM:return IfcTubeBundleTypeEnum::createObjectFromStepData( arg );
		case IFCURIREFERENCE:return IfcURIReference::createObjectFromStepData( arg );
		case IFCUNITENUM:return IfcUnitEnum::createObjectFromStepData( arg );
		case IFCUNITARYCONTROLELEMENTTYPEENUM:return IfcUnitaryControlElementTypeEnum::createObjectFromStepData( arg );
		case IFCUNITARYEQUIPMENTTYPEENUM:return IfcUnitaryEquipmentTypeEnum::createObjectFromStepData( arg );
		case IFCVALVETYPEENUM:return IfcValveTypeEnum::createObjectFromStepData( arg );
		case IFCVAPORPERMEABILITYMEASURE:return IfcVaporPermeabilityMeasure::createObjectFromStepData( arg );
		case IFCVIBRATIONISOLATORTYPEENUM:return IfcVibrationIsolatorTypeEnum::createObjectFromStepData( arg );
		case IFCVOIDINGFEATURETYPEENUM:return IfcVoidingFeatureTypeEnum::createObjectFromStepData( arg );
		case IFCVOLUMEMEASURE:return IfcVolumeMeasure::createObjectFromStepData( arg );
		case IFCVOLUMETRICFLOWRATEMEASURE:return IfcVolumetricFlowRateMeasure::createObjectFromStepData( arg );
		case IFCWALLTYPEENUM:return IfcWallTypeEnum::createObjectFromStepData( arg );
		case IFCWARPINGCONSTANTMEASURE:return IfcWarpingConstantMeasure::createObjectFromStepData( arg );
		case IFCWARPINGMOMENTMEASURE:return IfcWarpingMomentMeasure::createObjectFromStepData( arg );
		case IFCWASTETERMINALTYPEENUM:return IfcWasteTerminalTypeEnum::createObjectFromStepData( arg );
		case IFCWINDOWPANELOPERATIONENUM:return IfcWindowPanelOperationEnum::createObjectFromStepData( arg );
		case IFCWINDOWPANELPOSITIONENUM:return IfcWindowPanelPositionEnum::createObjectFromStepData( arg );
		case IFCWINDOWSTYLECONSTRUCTIONENUM:return IfcWindowStyleConstructionEnum::createObjectFromStepData( arg );
		case IFCWINDOWSTYLEOPERATIONENUM:return IfcWindowStyleOperationEnum::createObjectFromStepData( arg );
		case IFCWINDOWTYPEENUM:return IfcWindowTypeEnum::createObjectFromStepData( arg );
		case IFCWINDOWTYPEPARTITIONINGENUM:return IfcWindowTypePartitioningEnum::createObjectFromStepData( arg );
		case IFCWORKCALENDARTYPEENUM:return IfcWorkCalendarTypeEnum::createObjectFromStepData( arg );
		case IFCWORKPLANTYPEENUM:return IfcWorkPlanTypeEnum::createObjectFromStepData( arg );
		case IFCWORKSCHEDULETYPEENUM:return IfcWorkScheduleTypeEnum::createObjectFromStepData( arg );

		default: throw IfcPPException("given IfcPPTypeEnum not valid");
	}
	return 0;
}
