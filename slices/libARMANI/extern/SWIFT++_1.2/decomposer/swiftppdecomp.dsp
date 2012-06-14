# Microsoft Developer Studio Project File - Name="swiftppdecomp" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** NICHT BEARBEITEN **

# TARGTYPE "Win32 (x86) Static Library" 0x0104

CFG=swiftppdecomp - Win32 Debug
!MESSAGE Dies ist kein gültiges Makefile. Zum Erstellen dieses Projekts mit NMAKE
!MESSAGE verwenden Sie den Befehl "Makefile exportieren" und führen Sie den Befehl
!MESSAGE 
!MESSAGE NMAKE /f "swiftppdecomp.mak".
!MESSAGE 
!MESSAGE Sie können beim Ausführen von NMAKE eine Konfiguration angeben
!MESSAGE durch Definieren des Makros CFG in der Befehlszeile. Zum Beispiel:
!MESSAGE 
!MESSAGE NMAKE /f "swiftppdecomp.mak" CFG="swiftppdecomp - Win32 Debug"
!MESSAGE 
!MESSAGE Für die Konfiguration stehen zur Auswahl:
!MESSAGE 
!MESSAGE "swiftppdecomp - Win32 Release" (basierend auf  "Win32 (x86) Static Library")
!MESSAGE "swiftppdecomp - Win32 Debug" (basierend auf  "Win32 (x86) Static Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
RSC=rc.exe

!IF  "$(CFG)" == "swiftppdecomp - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Target_Dir ""
LINK32=link.exe -lib
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_MBCS" /D "_LIB" /YX /FD /c
# ADD CPP /nologo /W3 /GX /O2 /I "../include" /I "../../qhull-2003.1/src" /D "NDEBUG" /D "WIN32" /D "_MBCS" /D "_LIB" /D "SWIFT_ALLOW_BOUNDARY" /D "SWIFT_DECOMP" /YX /FD /c
# ADD BASE RSC /l 0x409 /d "NDEBUG"
# ADD RSC /l 0x409 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo

!ELSEIF  "$(CFG)" == "swiftppdecomp - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Target_Dir ""
LINK32=link.exe -lib
# ADD BASE CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_MBCS" /D "_LIB" /YX /FD /GZ /c
# ADD CPP /nologo /W3 /GX /Zi /Od /I "../include" /I "../../qhull-2003.1/src" /D "_DEBUG" /D "SWIFT_ALLOW_BOUNDARY" /D "SWIFT_DECOMP" /D "WIN32" /D "_MBCS" /D "_LIB" /FD /GZ /c /Tp
# SUBTRACT CPP /Fr /YX
# ADD BASE RSC /l 0x409 /d "_DEBUG"
# ADD RSC /l 0x409 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo

!ENDIF 

# Begin Target

# Name "swiftppdecomp - Win32 Release"
# Name "swiftppdecomp - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat"
# Begin Source File

SOURCE=..\src\fileio.cpp
# End Source File
# Begin Source File

SOURCE=..\src\lut.cpp
# End Source File
# Begin Source File

SOURCE=..\src\mesh.cpp
# End Source File
# Begin Source File

SOURCE=..\src\mesh_utils.cpp
# End Source File
# Begin Source File

SOURCE=..\src\object.cpp
# End Source File
# Begin Source File

SOURCE=..\src\pair.cpp
# End Source File
# Begin Source File

SOURCE=..\src\pqueue.cpp
# End Source File
# Begin Source File

SOURCE=..\src\scene.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl"
# Begin Source File

SOURCE=..\include\SWIFT.h
# End Source File
# Begin Source File

SOURCE=..\include\SWIFT_array.h
# End Source File
# Begin Source File

SOURCE=..\include\SWIFT_boxnode.h
# End Source File
# Begin Source File

SOURCE=..\include\SWIFT_common.h
# End Source File
# Begin Source File

SOURCE=..\include\SWIFT_config.h
# End Source File
# Begin Source File

SOURCE=..\include\SWIFT_fileio.h
# End Source File
# Begin Source File

SOURCE=..\include\SWIFT_front.h
# End Source File
# Begin Source File

SOURCE=..\include\SWIFT_linalg.h
# End Source File
# Begin Source File

SOURCE=..\include\SWIFT_lut.h
# End Source File
# Begin Source File

SOURCE=..\include\SWIFT_mesh.h
# End Source File
# Begin Source File

SOURCE=..\include\SWIFT_mesh_utils.h
# End Source File
# Begin Source File

SOURCE=..\include\SWIFT_object.h
# End Source File
# Begin Source File

SOURCE=..\include\SWIFT_pair.h
# End Source File
# Begin Source File

SOURCE=..\include\SWIFT_pqueue.h
# End Source File
# End Group
# End Target
# End Project
