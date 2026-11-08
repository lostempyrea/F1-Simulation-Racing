; Script generated for Inno Setup 6
#define MyAppName "Formula 1 Grand Prix Simulation"
#define MyAppVersion "1.0.0"
#define MyAppPublisher "lostempyrea"
#define MyAppURL "https://github.com/lostempyrea/F1-Simulation-Racing"
#define MyAppExeName "F1_GrandPrix.exe"

[Setup]
AppId={{E6F74D4E-9A5A-4B6A-8A4F-5C4D78901234}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppPublisher={#MyAppPublisher}
AppPublisherURL={#MyAppURL}
AppSupportURL={#MyAppURL}
AppUpdatesURL={#MyAppURL}
DefaultDirName={autopf}\F1 Grand Prix Simulation
DefaultGroupName={#MyAppName}
DisableProgramGroupPage=yes
OutputDir=c:\Users\nguye\PROJECT\F1\installer
OutputBaseFilename=F1_Simulation_Racing_Setup_v1.0.0
Compression=lzma2/ultra64
SolidCompression=yes
WizardStyle=modern
ArchitecturesInstallIn64BitMode=x64compatible

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked

[Files]
Source: "c:\Users\nguye\PROJECT\F1\Build\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs; Excludes: "*BurstDebugInformation*,*.pdb"

[Icons]
Name: "{group}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"
Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; Tasks: desktopicon

[Run]
Filename: "{app}\{#MyAppExeName}"; Description: "{cm:LaunchProgram,{#StringChange(MyAppName, '&', '&&')}}"; Flags: postinstall nowait skipifsilent
