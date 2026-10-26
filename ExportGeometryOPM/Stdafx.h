#pragma once

#define winNT 1
#include <MicroStationAPI.h>
#include <mselmdsc.fdf>
#include <msmdlmesh.fdf>
#include <mskisolid.fdf>
#include <ElementGraphics.h>

// add
#include <xmlfragment.fdf>
#include <imodel\xmlinstanceschemamanager.h>
#include <IModel\xmlinstanceapi.h>
#include <msdialog.fdf>
#include <mdlxmltools.fdf>
#include <msbsplin.fdf>

#using <System.dll>
#using <System.Core.dll>
//#using <System.Runtime.InteropServices.dll>

#using <ustation.dll>
#using <Bentley.MicroStation.dll>
#using <Bentley.MicroStation.Interfaces.1.0.dll>
#using <Bentley.General.1.0.dll>


// add
#using <bentley.interop.microstationdgn.dll>
#using <bentley.propertymanager.dll>
#using <Bentley.ECObjects.2.0.dll>
#using <Bentley.ECSystem.2.0.dll>
#using <Bentley.Plant.Utilities.dll>
#using <Bentley.MicroStation.XmlInstanceapi.dll>
#using <Bentley.MicroStation.XmlInstanceCore.dll>
#using <Bentley.Building.Mechanical.Api.dll>
#using <Bentley.APModeling.Controls.dll>
#using <Bentley.DGNECPlugin.dll>
#using <Bentley.DGNECPlugin.Common.dll>
#using <Bentley.EC.Persistence.2.0.dll>
#using <Bentley.Platform.dll>
#using <Bentley.UI.dll>
#using <System.Windows.Forms.dll>
//#using <nwcreate_17.dll>

using namespace std;
using namespace System::IO;
using namespace System::Linq;
using namespace System::Collections::Generic;
using namespace System::Runtime::InteropServices;
