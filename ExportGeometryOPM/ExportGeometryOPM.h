#pragma once

using namespace System;

namespace ExportGeometryOPM
{

	typedef Bentley::MicroStation::AddIn Base;
	[Bentley::MicroStation::AddInAttribute(KeyinTree = "commands.xml", MdlTaskID = "ExportGeometryOPM")]
	public ref class MyAddin sealed : public Base
	{
		private: MyAddin(System::IntPtr mdlDesc) : Base(mdlDesc)
		{
		}
		public: virtual int Run(array<System::String^>^ commandLine) override
		{
			return 0;
		}
	};
}
