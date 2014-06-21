program RadioAnalyzer;

uses
  Forms,
  URadioAnalyzer in 'URadioAnalyzer.pas' {FormRadioAnalyzer},
  FrameInfoRobot in 'FrameInfoRobot.pas' {FrameRobotInformation: TFrame},
  UTable in 'UTable.pas',
  ShockwaveFlashObjects_TLB in '..\..\..\Program Files\Borland\Delphi6\Imports\ShockwaveFlashObjects_TLB.pas';

{$R *.res}

begin
  Application.Initialize;
  Application.CreateForm(TFormRadioAnalyzer, FormRadioAnalyzer);
  Application.Run;
end.
