; Inno Setup script for Mesheal-windows-x64-setup.exe. Built by .github/workflows/release.yml:
;
;   ISCC.exe /DAppVersion=3 /DSourceDir=C:\path\to\Mesheal /O. packaging\windows\mesheal.iss
;
; SourceDir is the folder that also goes into Mesheal-windows-x64.zip (mesheal.exe, its DLLs,
; LICENSE.md, licenses\). The installer:
;   - installs for the current user only by default (no administrator prompt), into
;     %LOCALAPPDATA%\Programs\Mesheal; "Install for all users" asks for administrator rights
;   - adds a Start menu entry and, if ticked, a desktop icon
;   - lists Mesheal under "Open with" for .stl files. Windows 10 and 11 do not let an installer
;     make itself the default app; the user picks Mesheal once with "Always"
;   - adds an uninstaller to Settings > Apps, which also removes the .stl registration
; Installing a newer version over an older one updates it in place (same AppId).

#ifndef AppVersion
  #error Pass /DAppVersion=<version>
#endif
#ifndef SourceDir
  #error Pass /DSourceDir=<folder with mesheal.exe>
#endif

[Setup]
; Never change AppId: Windows uses it to recognise an existing install for updates and uninstall
AppId={{7994AF21-AEC4-4DED-A3BC-435EA4E633DF}
AppName=Mesheal
AppVersion={#AppVersion}
AppVerName=Mesheal {#AppVersion}
AppPublisher=Mesheal
AppPublisherURL=https://github.com/jsweezy/mesheal
AppSupportURL=https://github.com/jsweezy/mesheal/issues
AppUpdatesURL=https://github.com/jsweezy/mesheal/releases
DefaultDirName={autopf}\Mesheal
DefaultGroupName=Mesheal
DisableProgramGroupPage=yes
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog commandline
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
MinVersion=10.0
ChangesAssociations=yes
SetupIconFile=..\..\assets\icons\mesheal.ico
UninstallDisplayIcon={app}\mesheal.exe
UninstallDisplayName=Mesheal
OutputBaseFilename=Mesheal-windows-x64-setup
Compression=lzma2/max
SolidCompression=yes
WizardStyle=modern
CloseApplications=yes

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked

[Files]
Source: "{#SourceDir}\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{autoprograms}\Mesheal"; Filename: "{app}\mesheal.exe"; Comment: "Check and repair STL meshes"
Name: "{autodesktop}\Mesheal"; Filename: "{app}\mesheal.exe"; Tasks: desktopicon

[Registry]
; HKA is HKEY_CURRENT_USER for a per-user install and HKEY_LOCAL_MACHINE for an all-users install.
; "Open with" entry for .stl files, without taking over the user's current default app
Root: HKA; Subkey: "Software\Classes\.stl\OpenWithProgids"; ValueType: string; ValueName: "Mesheal.stl"; ValueData: ""; Flags: uninsdeletevalue
Root: HKA; Subkey: "Software\Classes\Mesheal.stl"; ValueType: string; ValueName: ""; ValueData: "STL mesh"; Flags: uninsdeletekey
Root: HKA; Subkey: "Software\Classes\Mesheal.stl\DefaultIcon"; ValueType: string; ValueName: ""; ValueData: "{app}\mesheal.exe,0"
Root: HKA; Subkey: "Software\Classes\Mesheal.stl\shell\open\command"; ValueType: string; ValueName: ""; ValueData: """{app}\mesheal.exe"" ""%1"""
Root: HKA; Subkey: "Software\Classes\Applications\mesheal.exe"; ValueType: string; ValueName: "FriendlyAppName"; ValueData: "Mesheal"; Flags: uninsdeletekey
Root: HKA; Subkey: "Software\Classes\Applications\mesheal.exe\SupportedTypes"; ValueType: string; ValueName: ".stl"; ValueData: ""
Root: HKA; Subkey: "Software\Classes\Applications\mesheal.exe\DefaultIcon"; ValueType: string; ValueName: ""; ValueData: "{app}\mesheal.exe,0"
Root: HKA; Subkey: "Software\Classes\Applications\mesheal.exe\shell\open\command"; ValueType: string; ValueName: ""; ValueData: """{app}\mesheal.exe"" ""%1"""

[Run]
Filename: "{app}\mesheal.exe"; Description: "{cm:LaunchProgram,Mesheal}"; Flags: nowait postinstall skipifsilent
