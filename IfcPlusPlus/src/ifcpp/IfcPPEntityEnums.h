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
#pragma once
enum IfcPPEntityEnum
{
	IFCACTIONREQUEST,
	IFCACTOR,
	IFCACTORROLE,
	IFCACTUATOR,
	IFCACTUATORTYPE,
	IFCADDRESS,
	IFCADVANCEDBREP,
	IFCADVANCEDBREPWITHVOIDS,
	IFCADVANCEDFACE,
	IFCAIRTERMINAL,
	IFCAIRTERMINALBOX,
	IFCAIRTERMINALBOXTYPE,
	IFCAIRTERMINALTYPE,
	IFCAIRTOAIRHEATRECOVERY,
	IFCAIRTOAIRHEATRECOVERYTYPE,
	IFCALARM,
	IFCALARMTYPE,
	IFCANNOTATION,
	IFCANNOTATIONFILLAREA,
	IFCAPPLICATION,
	IFCAPPLIEDVALUE,
	IFCAPPROVAL,
	IFCAPPROVALRELATIONSHIP,
	IFCARBITRARYCLOSEDPROFILEDEF,
	IFCARBITRARYOPENPROFILEDEF,
	IFCARBITRARYPROFILEDEFWITHVOIDS,
	IFCASSET,
	IFCASYMMETRICISHAPEPROFILEDEF,
	IFCAUDIOVISUALAPPLIANCE,
	IFCAUDIOVISUALAPPLIANCETYPE,
	IFCAXIS1PLACEMENT,
	IFCAXIS2PLACEMENT2D,
	IFCAXIS2PLACEMENT3D,
	IFCBSPLINECURVE,
	IFCBSPLINECURVEWITHKNOTS,
	IFCBSPLINESURFACE,
	IFCBSPLINESURFACEWITHKNOTS,
	IFCBEAM,
	IFCBEAMSTANDARDCASE,
	IFCBEAMTYPE,
	IFCBLOBTEXTURE,
	IFCBLOCK,
	IFCBOILER,
	IFCBOILERTYPE,
	IFCBOOLEANCLIPPINGRESULT,
	IFCBOOLEANRESULT,
	IFCBOUNDARYCONDITION,
	IFCBOUNDARYCURVE,
	IFCBOUNDARYEDGECONDITION,
	IFCBOUNDARYFACECONDITION,
	IFCBOUNDARYNODECONDITION,
	IFCBOUNDARYNODECONDITIONWARPING,
	IFCBOUNDEDCURVE,
	IFCBOUNDEDSURFACE,
	IFCBOUNDINGBOX,
	IFCBOXEDHALFSPACE,
	IFCBUILDING,
	IFCBUILDINGELEMENT,
	IFCBUILDINGELEMENTPART,
	IFCBUILDINGELEMENTPARTTYPE,
	IFCBUILDINGELEMENTPROXY,
	IFCBUILDINGELEMENTPROXYTYPE,
	IFCBUILDINGELEMENTTYPE,
	IFCBUILDINGSTOREY,
	IFCBUILDINGSYSTEM,
	IFCBURNER,
	IFCBURNERTYPE,
	IFCCSHAPEPROFILEDEF,
	IFCCABLECARRIERFITTING,
	IFCCABLECARRIERFITTINGTYPE,
	IFCCABLECARRIERSEGMENT,
	IFCCABLECARRIERSEGMENTTYPE,
	IFCCABLEFITTING,
	IFCCABLEFITTINGTYPE,
	IFCCABLESEGMENT,
	IFCCABLESEGMENTTYPE,
	IFCCARTESIANPOINT,
	IFCCARTESIANPOINTLIST,
	IFCCARTESIANPOINTLIST3D,
	IFCCARTESIANTRANSFORMATIONOPERATOR,
	IFCCARTESIANTRANSFORMATIONOPERATOR2D,
	IFCCARTESIANTRANSFORMATIONOPERATOR2DNONUNIFORM,
	IFCCARTESIANTRANSFORMATIONOPERATOR3D,
	IFCCARTESIANTRANSFORMATIONOPERATOR3DNONUNIFORM,
	IFCCENTERLINEPROFILEDEF,
	IFCCHILLER,
	IFCCHILLERTYPE,
	IFCCHIMNEY,
	IFCCHIMNEYTYPE,
	IFCCIRCLE,
	IFCCIRCLEHOLLOWPROFILEDEF,
	IFCCIRCLEPROFILEDEF,
	IFCCIVILELEMENT,
	IFCCIVILELEMENTTYPE,
	IFCCLASSIFICATION,
	IFCCLASSIFICATIONREFERENCE,
	IFCCLOSEDSHELL,
	IFCCOIL,
	IFCCOILTYPE,
	IFCCOLOURRGB,
	IFCCOLOURRGBLIST,
	IFCCOLOURSPECIFICATION,
	IFCCOLUMN,
	IFCCOLUMNSTANDARDCASE,
	IFCCOLUMNTYPE,
	IFCCOMMUNICATIONSAPPLIANCE,
	IFCCOMMUNICATIONSAPPLIANCETYPE,
	IFCCOMPLEXPROPERTY,
	IFCCOMPLEXPROPERTYTEMPLATE,
	IFCCOMPOSITECURVE,
	IFCCOMPOSITECURVEONSURFACE,
	IFCCOMPOSITECURVESEGMENT,
	IFCCOMPOSITEPROFILEDEF,
	IFCCOMPRESSOR,
	IFCCOMPRESSORTYPE,
	IFCCONDENSER,
	IFCCONDENSERTYPE,
	IFCCONIC,
	IFCCONNECTEDFACESET,
	IFCCONNECTIONCURVEGEOMETRY,
	IFCCONNECTIONGEOMETRY,
	IFCCONNECTIONPOINTECCENTRICITY,
	IFCCONNECTIONPOINTGEOMETRY,
	IFCCONNECTIONSURFACEGEOMETRY,
	IFCCONNECTIONVOLUMEGEOMETRY,
	IFCCONSTRAINT,
	IFCCONSTRUCTIONEQUIPMENTRESOURCE,
	IFCCONSTRUCTIONEQUIPMENTRESOURCETYPE,
	IFCCONSTRUCTIONMATERIALRESOURCE,
	IFCCONSTRUCTIONMATERIALRESOURCETYPE,
	IFCCONSTRUCTIONPRODUCTRESOURCE,
	IFCCONSTRUCTIONPRODUCTRESOURCETYPE,
	IFCCONSTRUCTIONRESOURCE,
	IFCCONSTRUCTIONRESOURCETYPE,
	IFCCONTEXT,
	IFCCONTEXTDEPENDENTUNIT,
	IFCCONTROL,
	IFCCONTROLLER,
	IFCCONTROLLERTYPE,
	IFCCONVERSIONBASEDUNIT,
	IFCCONVERSIONBASEDUNITWITHOFFSET,
	IFCCOOLEDBEAM,
	IFCCOOLEDBEAMTYPE,
	IFCCOOLINGTOWER,
	IFCCOOLINGTOWERTYPE,
	IFCCOORDINATEOPERATION,
	IFCCOORDINATEREFERENCESYSTEM,
	IFCCOSTITEM,
	IFCCOSTSCHEDULE,
	IFCCOSTVALUE,
	IFCCOVERING,
	IFCCOVERINGTYPE,
	IFCCREWRESOURCE,
	IFCCREWRESOURCETYPE,
	IFCCSGPRIMITIVE3D,
	IFCCSGSOLID,
	IFCCURRENCYRELATIONSHIP,
	IFCCURTAINWALL,
	IFCCURTAINWALLTYPE,
	IFCCURVE,
	IFCCURVEBOUNDEDPLANE,
	IFCCURVEBOUNDEDSURFACE,
	IFCCURVESTYLE,
	IFCCURVESTYLEFONT,
	IFCCURVESTYLEFONTANDSCALING,
	IFCCURVESTYLEFONTPATTERN,
	IFCCYLINDRICALSURFACE,
	IFCDAMPER,
	IFCDAMPERTYPE,
	IFCDERIVEDPROFILEDEF,
	IFCDERIVEDUNIT,
	IFCDERIVEDUNITELEMENT,
	IFCDIMENSIONALEXPONENTS,
	IFCDIRECTION,
	IFCDISCRETEACCESSORY,
	IFCDISCRETEACCESSORYTYPE,
	IFCDISTRIBUTIONCHAMBERELEMENT,
	IFCDISTRIBUTIONCHAMBERELEMENTTYPE,
	IFCDISTRIBUTIONCIRCUIT,
	IFCDISTRIBUTIONCONTROLELEMENT,
	IFCDISTRIBUTIONCONTROLELEMENTTYPE,
	IFCDISTRIBUTIONELEMENT,
	IFCDISTRIBUTIONELEMENTTYPE,
	IFCDISTRIBUTIONFLOWELEMENT,
	IFCDISTRIBUTIONFLOWELEMENTTYPE,
	IFCDISTRIBUTIONPORT,
	IFCDISTRIBUTIONSYSTEM,
	IFCDOCUMENTINFORMATION,
	IFCDOCUMENTINFORMATIONRELATIONSHIP,
	IFCDOCUMENTREFERENCE,
	IFCDOOR,
	IFCDOORLININGPROPERTIES,
	IFCDOORPANELPROPERTIES,
	IFCDOORSTANDARDCASE,
	IFCDOORSTYLE,
	IFCDOORTYPE,
	IFCDRAUGHTINGPREDEFINEDCOLOUR,
	IFCDRAUGHTINGPREDEFINEDCURVEFONT,
	IFCDUCTFITTING,
	IFCDUCTFITTINGTYPE,
	IFCDUCTSEGMENT,
	IFCDUCTSEGMENTTYPE,
	IFCDUCTSILENCER,
	IFCDUCTSILENCERTYPE,
	IFCEDGE,
	IFCEDGECURVE,
	IFCEDGELOOP,
	IFCELECTRICAPPLIANCE,
	IFCELECTRICAPPLIANCETYPE,
	IFCELECTRICDISTRIBUTIONBOARD,
	IFCELECTRICDISTRIBUTIONBOARDTYPE,
	IFCELECTRICFLOWSTORAGEDEVICE,
	IFCELECTRICFLOWSTORAGEDEVICETYPE,
	IFCELECTRICGENERATOR,
	IFCELECTRICGENERATORTYPE,
	IFCELECTRICMOTOR,
	IFCELECTRICMOTORTYPE,
	IFCELECTRICTIMECONTROL,
	IFCELECTRICTIMECONTROLTYPE,
	IFCELEMENT,
	IFCELEMENTASSEMBLY,
	IFCELEMENTASSEMBLYTYPE,
	IFCELEMENTCOMPONENT,
	IFCELEMENTCOMPONENTTYPE,
	IFCELEMENTQUANTITY,
	IFCELEMENTTYPE,
	IFCELEMENTARYSURFACE,
	IFCELLIPSE,
	IFCELLIPSEPROFILEDEF,
	IFCENERGYCONVERSIONDEVICE,
	IFCENERGYCONVERSIONDEVICETYPE,
	IFCENGINE,
	IFCENGINETYPE,
	IFCEVAPORATIVECOOLER,
	IFCEVAPORATIVECOOLERTYPE,
	IFCEVAPORATOR,
	IFCEVAPORATORTYPE,
	IFCEVENT,
	IFCEVENTTIME,
	IFCEVENTTYPE,
	IFCEXTENDEDPROPERTIES,
	IFCEXTERNALINFORMATION,
	IFCEXTERNALREFERENCE,
	IFCEXTERNALREFERENCERELATIONSHIP,
	IFCEXTERNALSPATIALELEMENT,
	IFCEXTERNALSPATIALSTRUCTUREELEMENT,
	IFCEXTERNALLYDEFINEDHATCHSTYLE,
	IFCEXTERNALLYDEFINEDSURFACESTYLE,
	IFCEXTERNALLYDEFINEDTEXTFONT,
	IFCEXTRUDEDAREASOLID,
	IFCEXTRUDEDAREASOLIDTAPERED,
	IFCFACE,
	IFCFACEBASEDSURFACEMODEL,
	IFCFACEBOUND,
	IFCFACEOUTERBOUND,
	IFCFACESURFACE,
	IFCFACETEDBREP,
	IFCFACETEDBREPWITHVOIDS,
	IFCFAILURECONNECTIONCONDITION,
	IFCFAN,
	IFCFANTYPE,
	IFCFASTENER,
	IFCFASTENERTYPE,
	IFCFEATUREELEMENT,
	IFCFEATUREELEMENTADDITION,
	IFCFEATUREELEMENTSUBTRACTION,
	IFCFILLAREASTYLE,
	IFCFILLAREASTYLEHATCHING,
	IFCFILLAREASTYLETILES,
	IFCFILTER,
	IFCFILTERTYPE,
	IFCFIRESUPPRESSIONTERMINAL,
	IFCFIRESUPPRESSIONTERMINALTYPE,
	IFCFIXEDREFERENCESWEPTAREASOLID,
	IFCFLOWCONTROLLER,
	IFCFLOWCONTROLLERTYPE,
	IFCFLOWFITTING,
	IFCFLOWFITTINGTYPE,
	IFCFLOWINSTRUMENT,
	IFCFLOWINSTRUMENTTYPE,
	IFCFLOWMETER,
	IFCFLOWMETERTYPE,
	IFCFLOWMOVINGDEVICE,
	IFCFLOWMOVINGDEVICETYPE,
	IFCFLOWSEGMENT,
	IFCFLOWSEGMENTTYPE,
	IFCFLOWSTORAGEDEVICE,
	IFCFLOWSTORAGEDEVICETYPE,
	IFCFLOWTERMINAL,
	IFCFLOWTERMINALTYPE,
	IFCFLOWTREATMENTDEVICE,
	IFCFLOWTREATMENTDEVICETYPE,
	IFCFOOTING,
	IFCFOOTINGTYPE,
	IFCFURNISHINGELEMENT,
	IFCFURNISHINGELEMENTTYPE,
	IFCFURNITURE,
	IFCFURNITURETYPE,
	IFCGEOGRAPHICELEMENT,
	IFCGEOGRAPHICELEMENTTYPE,
	IFCGEOMETRICCURVESET,
	IFCGEOMETRICREPRESENTATIONCONTEXT,
	IFCGEOMETRICREPRESENTATIONITEM,
	IFCGEOMETRICREPRESENTATIONSUBCONTEXT,
	IFCGEOMETRICSET,
	IFCGRID,
	IFCGRIDAXIS,
	IFCGRIDPLACEMENT,
	IFCGROUP,
	IFCHALFSPACESOLID,
	IFCHEATEXCHANGER,
	IFCHEATEXCHANGERTYPE,
	IFCHUMIDIFIER,
	IFCHUMIDIFIERTYPE,
	IFCISHAPEPROFILEDEF,
	IFCIMAGETEXTURE,
	IFCINDEXEDCOLOURMAP,
	IFCINDEXEDTEXTUREMAP,
	IFCINDEXEDTRIANGLETEXTUREMAP,
	IFCINTERCEPTOR,
	IFCINTERCEPTORTYPE,
	IFCINVENTORY,
	IFCIRREGULARTIMESERIES,
	IFCIRREGULARTIMESERIESVALUE,
	IFCJUNCTIONBOX,
	IFCJUNCTIONBOXTYPE,
	IFCLSHAPEPROFILEDEF,
	IFCLABORRESOURCE,
	IFCLABORRESOURCETYPE,
	IFCLAGTIME,
	IFCLAMP,
	IFCLAMPTYPE,
	IFCLIBRARYINFORMATION,
	IFCLIBRARYREFERENCE,
	IFCLIGHTDISTRIBUTIONDATA,
	IFCLIGHTFIXTURE,
	IFCLIGHTFIXTURETYPE,
	IFCLIGHTINTENSITYDISTRIBUTION,
	IFCLIGHTSOURCE,
	IFCLIGHTSOURCEAMBIENT,
	IFCLIGHTSOURCEDIRECTIONAL,
	IFCLIGHTSOURCEGONIOMETRIC,
	IFCLIGHTSOURCEPOSITIONAL,
	IFCLIGHTSOURCESPOT,
	IFCLINE,
	IFCLOCALPLACEMENT,
	IFCLOOP,
	IFCMANIFOLDSOLIDBREP,
	IFCMAPCONVERSION,
	IFCMAPPEDITEM,
	IFCMATERIAL,
	IFCMATERIALCLASSIFICATIONRELATIONSHIP,
	IFCMATERIALCONSTITUENT,
	IFCMATERIALCONSTITUENTSET,
	IFCMATERIALDEFINITION,
	IFCMATERIALDEFINITIONREPRESENTATION,
	IFCMATERIALLAYER,
	IFCMATERIALLAYERSET,
	IFCMATERIALLAYERSETUSAGE,
	IFCMATERIALLAYERWITHOFFSETS,
	IFCMATERIALLIST,
	IFCMATERIALPROFILE,
	IFCMATERIALPROFILESET,
	IFCMATERIALPROFILESETUSAGE,
	IFCMATERIALPROFILESETUSAGETAPERING,
	IFCMATERIALPROFILEWITHOFFSETS,
	IFCMATERIALPROPERTIES,
	IFCMATERIALRELATIONSHIP,
	IFCMATERIALUSAGEDEFINITION,
	IFCMEASUREWITHUNIT,
	IFCMECHANICALFASTENER,
	IFCMECHANICALFASTENERTYPE,
	IFCMEDICALDEVICE,
	IFCMEDICALDEVICETYPE,
	IFCMEMBER,
	IFCMEMBERSTANDARDCASE,
	IFCMEMBERTYPE,
	IFCMETRIC,
	IFCMIRROREDPROFILEDEF,
	IFCMONETARYUNIT,
	IFCMOTORCONNECTION,
	IFCMOTORCONNECTIONTYPE,
	IFCNAMEDUNIT,
	IFCOBJECT,
	IFCOBJECTDEFINITION,
	IFCOBJECTPLACEMENT,
	IFCOBJECTIVE,
	IFCOCCUPANT,
	IFCOFFSETCURVE2D,
	IFCOFFSETCURVE3D,
	IFCOPENSHELL,
	IFCOPENINGELEMENT,
	IFCOPENINGSTANDARDCASE,
	IFCORGANIZATION,
	IFCORGANIZATIONRELATIONSHIP,
	IFCORIENTEDEDGE,
	IFCOUTERBOUNDARYCURVE,
	IFCOUTLET,
	IFCOUTLETTYPE,
	IFCOWNERHISTORY,
	IFCPARAMETERIZEDPROFILEDEF,
	IFCPATH,
	IFCPCURVE,
	IFCPERFORMANCEHISTORY,
	IFCPERMEABLECOVERINGPROPERTIES,
	IFCPERMIT,
	IFCPERSON,
	IFCPERSONANDORGANIZATION,
	IFCPHYSICALCOMPLEXQUANTITY,
	IFCPHYSICALQUANTITY,
	IFCPHYSICALSIMPLEQUANTITY,
	IFCPILE,
	IFCPILETYPE,
	IFCPIPEFITTING,
	IFCPIPEFITTINGTYPE,
	IFCPIPESEGMENT,
	IFCPIPESEGMENTTYPE,
	IFCPIXELTEXTURE,
	IFCPLACEMENT,
	IFCPLANARBOX,
	IFCPLANAREXTENT,
	IFCPLANE,
	IFCPLATE,
	IFCPLATESTANDARDCASE,
	IFCPLATETYPE,
	IFCPOINT,
	IFCPOINTONCURVE,
	IFCPOINTONSURFACE,
	IFCPOLYLOOP,
	IFCPOLYGONALBOUNDEDHALFSPACE,
	IFCPOLYLINE,
	IFCPORT,
	IFCPOSTALADDRESS,
	IFCPREDEFINEDCOLOUR,
	IFCPREDEFINEDCURVEFONT,
	IFCPREDEFINEDITEM,
	IFCPREDEFINEDPROPERTIES,
	IFCPREDEFINEDPROPERTYSET,
	IFCPREDEFINEDTEXTFONT,
	IFCPRESENTATIONITEM,
	IFCPRESENTATIONLAYERASSIGNMENT,
	IFCPRESENTATIONLAYERWITHSTYLE,
	IFCPRESENTATIONSTYLE,
	IFCPRESENTATIONSTYLEASSIGNMENT,
	IFCPROCEDURE,
	IFCPROCEDURETYPE,
	IFCPROCESS,
	IFCPRODUCT,
	IFCPRODUCTDEFINITIONSHAPE,
	IFCPRODUCTREPRESENTATION,
	IFCPROFILEDEF,
	IFCPROFILEPROPERTIES,
	IFCPROJECT,
	IFCPROJECTLIBRARY,
	IFCPROJECTORDER,
	IFCPROJECTEDCRS,
	IFCPROJECTIONELEMENT,
	IFCPROPERTY,
	IFCPROPERTYABSTRACTION,
	IFCPROPERTYBOUNDEDVALUE,
	IFCPROPERTYDEFINITION,
	IFCPROPERTYDEPENDENCYRELATIONSHIP,
	IFCPROPERTYENUMERATEDVALUE,
	IFCPROPERTYENUMERATION,
	IFCPROPERTYLISTVALUE,
	IFCPROPERTYREFERENCEVALUE,
	IFCPROPERTYSET,
	IFCPROPERTYSETDEFINITION,
	IFCPROPERTYSETTEMPLATE,
	IFCPROPERTYSINGLEVALUE,
	IFCPROPERTYTABLEVALUE,
	IFCPROPERTYTEMPLATE,
	IFCPROPERTYTEMPLATEDEFINITION,
	IFCPROTECTIVEDEVICE,
	IFCPROTECTIVEDEVICETRIPPINGUNIT,
	IFCPROTECTIVEDEVICETRIPPINGUNITTYPE,
	IFCPROTECTIVEDEVICETYPE,
	IFCPROXY,
	IFCPUMP,
	IFCPUMPTYPE,
	IFCQUANTITYAREA,
	IFCQUANTITYCOUNT,
	IFCQUANTITYLENGTH,
	IFCQUANTITYSET,
	IFCQUANTITYTIME,
	IFCQUANTITYVOLUME,
	IFCQUANTITYWEIGHT,
	IFCRAILING,
	IFCRAILINGTYPE,
	IFCRAMP,
	IFCRAMPFLIGHT,
	IFCRAMPFLIGHTTYPE,
	IFCRAMPTYPE,
	IFCRATIONALBSPLINECURVEWITHKNOTS,
	IFCRATIONALBSPLINESURFACEWITHKNOTS,
	IFCRECTANGLEHOLLOWPROFILEDEF,
	IFCRECTANGLEPROFILEDEF,
	IFCRECTANGULARPYRAMID,
	IFCRECTANGULARTRIMMEDSURFACE,
	IFCRECURRENCEPATTERN,
	IFCREFERENCE,
	IFCREGULARTIMESERIES,
	IFCREINFORCEMENTBARPROPERTIES,
	IFCREINFORCEMENTDEFINITIONPROPERTIES,
	IFCREINFORCINGBAR,
	IFCREINFORCINGBARTYPE,
	IFCREINFORCINGELEMENT,
	IFCREINFORCINGELEMENTTYPE,
	IFCREINFORCINGMESH,
	IFCREINFORCINGMESHTYPE,
	IFCRELAGGREGATES,
	IFCRELASSIGNS,
	IFCRELASSIGNSTOACTOR,
	IFCRELASSIGNSTOCONTROL,
	IFCRELASSIGNSTOGROUP,
	IFCRELASSIGNSTOGROUPBYFACTOR,
	IFCRELASSIGNSTOPROCESS,
	IFCRELASSIGNSTOPRODUCT,
	IFCRELASSIGNSTORESOURCE,
	IFCRELASSOCIATES,
	IFCRELASSOCIATESAPPROVAL,
	IFCRELASSOCIATESCLASSIFICATION,
	IFCRELASSOCIATESCONSTRAINT,
	IFCRELASSOCIATESDOCUMENT,
	IFCRELASSOCIATESLIBRARY,
	IFCRELASSOCIATESMATERIAL,
	IFCRELCONNECTS,
	IFCRELCONNECTSELEMENTS,
	IFCRELCONNECTSPATHELEMENTS,
	IFCRELCONNECTSPORTTOELEMENT,
	IFCRELCONNECTSPORTS,
	IFCRELCONNECTSSTRUCTURALACTIVITY,
	IFCRELCONNECTSSTRUCTURALMEMBER,
	IFCRELCONNECTSWITHECCENTRICITY,
	IFCRELCONNECTSWITHREALIZINGELEMENTS,
	IFCRELCONTAINEDINSPATIALSTRUCTURE,
	IFCRELCOVERSBLDGELEMENTS,
	IFCRELCOVERSSPACES,
	IFCRELDECLARES,
	IFCRELDECOMPOSES,
	IFCRELDEFINES,
	IFCRELDEFINESBYOBJECT,
	IFCRELDEFINESBYPROPERTIES,
	IFCRELDEFINESBYTEMPLATE,
	IFCRELDEFINESBYTYPE,
	IFCRELFILLSELEMENT,
	IFCRELFLOWCONTROLELEMENTS,
	IFCRELINTERFERESELEMENTS,
	IFCRELNESTS,
	IFCRELPROJECTSELEMENT,
	IFCRELREFERENCEDINSPATIALSTRUCTURE,
	IFCRELSEQUENCE,
	IFCRELSERVICESBUILDINGS,
	IFCRELSPACEBOUNDARY,
	IFCRELSPACEBOUNDARY1STLEVEL,
	IFCRELSPACEBOUNDARY2NDLEVEL,
	IFCRELVOIDSELEMENT,
	IFCRELATIONSHIP,
	IFCREPARAMETRISEDCOMPOSITECURVESEGMENT,
	IFCREPRESENTATION,
	IFCREPRESENTATIONCONTEXT,
	IFCREPRESENTATIONITEM,
	IFCREPRESENTATIONMAP,
	IFCRESOURCE,
	IFCRESOURCEAPPROVALRELATIONSHIP,
	IFCRESOURCECONSTRAINTRELATIONSHIP,
	IFCRESOURCELEVELRELATIONSHIP,
	IFCRESOURCETIME,
	IFCREVOLVEDAREASOLID,
	IFCREVOLVEDAREASOLIDTAPERED,
	IFCRIGHTCIRCULARCONE,
	IFCRIGHTCIRCULARCYLINDER,
	IFCROOF,
	IFCROOFTYPE,
	IFCROOT,
	IFCROUNDEDRECTANGLEPROFILEDEF,
	IFCSIUNIT,
	IFCSANITARYTERMINAL,
	IFCSANITARYTERMINALTYPE,
	IFCSCHEDULINGTIME,
	IFCSECTIONPROPERTIES,
	IFCSECTIONREINFORCEMENTPROPERTIES,
	IFCSECTIONEDSPINE,
	IFCSENSOR,
	IFCSENSORTYPE,
	IFCSHADINGDEVICE,
	IFCSHADINGDEVICETYPE,
	IFCSHAPEASPECT,
	IFCSHAPEMODEL,
	IFCSHAPEREPRESENTATION,
	IFCSHELLBASEDSURFACEMODEL,
	IFCSIMPLEPROPERTY,
	IFCSIMPLEPROPERTYTEMPLATE,
	IFCSITE,
	IFCSLAB,
	IFCSLABELEMENTEDCASE,
	IFCSLABSTANDARDCASE,
	IFCSLABTYPE,
	IFCSLIPPAGECONNECTIONCONDITION,
	IFCSOLARDEVICE,
	IFCSOLARDEVICETYPE,
	IFCSOLIDMODEL,
	IFCSPACE,
	IFCSPACEHEATER,
	IFCSPACEHEATERTYPE,
	IFCSPACETYPE,
	IFCSPATIALELEMENT,
	IFCSPATIALELEMENTTYPE,
	IFCSPATIALSTRUCTUREELEMENT,
	IFCSPATIALSTRUCTUREELEMENTTYPE,
	IFCSPATIALZONE,
	IFCSPATIALZONETYPE,
	IFCSPHERE,
	IFCSTACKTERMINAL,
	IFCSTACKTERMINALTYPE,
	IFCSTAIR,
	IFCSTAIRFLIGHT,
	IFCSTAIRFLIGHTTYPE,
	IFCSTAIRTYPE,
	IFCSTRUCTURALACTION,
	IFCSTRUCTURALACTIVITY,
	IFCSTRUCTURALANALYSISMODEL,
	IFCSTRUCTURALCONNECTION,
	IFCSTRUCTURALCONNECTIONCONDITION,
	IFCSTRUCTURALCURVEACTION,
	IFCSTRUCTURALCURVECONNECTION,
	IFCSTRUCTURALCURVEMEMBER,
	IFCSTRUCTURALCURVEMEMBERVARYING,
	IFCSTRUCTURALCURVEREACTION,
	IFCSTRUCTURALITEM,
	IFCSTRUCTURALLINEARACTION,
	IFCSTRUCTURALLOAD,
	IFCSTRUCTURALLOADCASE,
	IFCSTRUCTURALLOADCONFIGURATION,
	IFCSTRUCTURALLOADGROUP,
	IFCSTRUCTURALLOADLINEARFORCE,
	IFCSTRUCTURALLOADORRESULT,
	IFCSTRUCTURALLOADPLANARFORCE,
	IFCSTRUCTURALLOADSINGLEDISPLACEMENT,
	IFCSTRUCTURALLOADSINGLEDISPLACEMENTDISTORTION,
	IFCSTRUCTURALLOADSINGLEFORCE,
	IFCSTRUCTURALLOADSINGLEFORCEWARPING,
	IFCSTRUCTURALLOADSTATIC,
	IFCSTRUCTURALLOADTEMPERATURE,
	IFCSTRUCTURALMEMBER,
	IFCSTRUCTURALPLANARACTION,
	IFCSTRUCTURALPOINTACTION,
	IFCSTRUCTURALPOINTCONNECTION,
	IFCSTRUCTURALPOINTREACTION,
	IFCSTRUCTURALREACTION,
	IFCSTRUCTURALRESULTGROUP,
	IFCSTRUCTURALSURFACEACTION,
	IFCSTRUCTURALSURFACECONNECTION,
	IFCSTRUCTURALSURFACEMEMBER,
	IFCSTRUCTURALSURFACEMEMBERVARYING,
	IFCSTRUCTURALSURFACEREACTION,
	IFCSTYLEMODEL,
	IFCSTYLEDITEM,
	IFCSTYLEDREPRESENTATION,
	IFCSUBCONTRACTRESOURCE,
	IFCSUBCONTRACTRESOURCETYPE,
	IFCSUBEDGE,
	IFCSURFACE,
	IFCSURFACECURVESWEPTAREASOLID,
	IFCSURFACEFEATURE,
	IFCSURFACEOFLINEAREXTRUSION,
	IFCSURFACEOFREVOLUTION,
	IFCSURFACEREINFORCEMENTAREA,
	IFCSURFACESTYLE,
	IFCSURFACESTYLELIGHTING,
	IFCSURFACESTYLEREFRACTION,
	IFCSURFACESTYLERENDERING,
	IFCSURFACESTYLESHADING,
	IFCSURFACESTYLEWITHTEXTURES,
	IFCSURFACETEXTURE,
	IFCSWEPTAREASOLID,
	IFCSWEPTDISKSOLID,
	IFCSWEPTDISKSOLIDPOLYGONAL,
	IFCSWEPTSURFACE,
	IFCSWITCHINGDEVICE,
	IFCSWITCHINGDEVICETYPE,
	IFCSYSTEM,
	IFCSYSTEMFURNITUREELEMENT,
	IFCSYSTEMFURNITUREELEMENTTYPE,
	IFCTSHAPEPROFILEDEF,
	IFCTABLE,
	IFCTABLECOLUMN,
	IFCTABLEROW,
	IFCTANK,
	IFCTANKTYPE,
	IFCTASK,
	IFCTASKTIME,
	IFCTASKTIMERECURRING,
	IFCTASKTYPE,
	IFCTELECOMADDRESS,
	IFCTENDON,
	IFCTENDONANCHOR,
	IFCTENDONANCHORTYPE,
	IFCTENDONTYPE,
	IFCTESSELLATEDFACESET,
	IFCTESSELLATEDITEM,
	IFCTEXTLITERAL,
	IFCTEXTLITERALWITHEXTENT,
	IFCTEXTSTYLE,
	IFCTEXTSTYLEFONTMODEL,
	IFCTEXTSTYLEFORDEFINEDFONT,
	IFCTEXTSTYLETEXTMODEL,
	IFCTEXTURECOORDINATE,
	IFCTEXTURECOORDINATEGENERATOR,
	IFCTEXTUREMAP,
	IFCTEXTUREVERTEX,
	IFCTEXTUREVERTEXLIST,
	IFCTIMEPERIOD,
	IFCTIMESERIES,
	IFCTIMESERIESVALUE,
	IFCTOPOLOGICALREPRESENTATIONITEM,
	IFCTOPOLOGYREPRESENTATION,
	IFCTRANSFORMER,
	IFCTRANSFORMERTYPE,
	IFCTRANSPORTELEMENT,
	IFCTRANSPORTELEMENTTYPE,
	IFCTRAPEZIUMPROFILEDEF,
	IFCTRIANGULATEDFACESET,
	IFCTRIMMEDCURVE,
	IFCTUBEBUNDLE,
	IFCTUBEBUNDLETYPE,
	IFCTYPEOBJECT,
	IFCTYPEPROCESS,
	IFCTYPEPRODUCT,
	IFCTYPERESOURCE,
	IFCUSHAPEPROFILEDEF,
	IFCUNITASSIGNMENT,
	IFCUNITARYCONTROLELEMENT,
	IFCUNITARYCONTROLELEMENTTYPE,
	IFCUNITARYEQUIPMENT,
	IFCUNITARYEQUIPMENTTYPE,
	IFCVALVE,
	IFCVALVETYPE,
	IFCVECTOR,
	IFCVERTEX,
	IFCVERTEXLOOP,
	IFCVERTEXPOINT,
	IFCVIBRATIONISOLATOR,
	IFCVIBRATIONISOLATORTYPE,
	IFCVIRTUALELEMENT,
	IFCVIRTUALGRIDINTERSECTION,
	IFCVOIDINGFEATURE,
	IFCWALL,
	IFCWALLELEMENTEDCASE,
	IFCWALLSTANDARDCASE,
	IFCWALLTYPE,
	IFCWASTETERMINAL,
	IFCWASTETERMINALTYPE,
	IFCWINDOW,
	IFCWINDOWLININGPROPERTIES,
	IFCWINDOWPANELPROPERTIES,
	IFCWINDOWSTANDARDCASE,
	IFCWINDOWSTYLE,
	IFCWINDOWTYPE,
	IFCWORKCALENDAR,
	IFCWORKCONTROL,
	IFCWORKPLAN,
	IFCWORKSCHEDULE,
	IFCWORKTIME,
	IFCZSHAPEPROFILEDEF,
	IFCZONE
};

