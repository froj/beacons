unit URadioAnalyzer;

interface

uses
  Windows, Messages, SysUtils, Variants, Classes, Graphics, Controls, Forms,
  Dialogs, CPDrv, StdCtrls, ExtCtrls, DateUtils, FrameInfoRobot, UTable,
  jpeg;

type
  TFormRadioAnalyzer = class(TForm)
    CommPortDriver: TCommPortDriver;
    PanelBottom: TPanel;
    MemoReceiveValues: TMemo;
    LabelSoundSpeed: TLabel;
    PanelLeft: TPanel;
    FrameRobotInformation0: TFrameRobotInformation;
    FrameRobotInformation1: TFrameRobotInformation;
    RoboticTable: TRoboticTable;
    Button1: TButton;
    LabelTimerMatch: TLabel;
    TimerMatch: TTimer;
    ButtonStartMatch: TButton;
    Image1: TImage;
    LabelTimerMatchTitle: TLabel;
    RadioGroupMode: TRadioGroup;
    ButtonNext: TButton;
    ButtonPrevious: TButton;
    OpenDialog: TOpenDialog;
    procedure CommPortDriverReceiveData(Sender: TObject; DataPtr: Pointer;
      DataSize: Cardinal);
    procedure FormCreate(Sender: TObject);
    procedure FormClose(Sender: TObject; var Action: TCloseAction);
    procedure Button1Click(Sender: TObject);
    procedure TimerMatchTimer(Sender: TObject);
    procedure ButtonStartMatchClick(Sender: TObject);
    procedure RadioGroupModeClick(Sender: TObject);
    procedure ButtonNextClick(Sender: TObject);
    procedure ButtonPreviousClick(Sender: TObject);
  private
    { Private declarations }
    MatchTimeCount : Integer;
    ValuesReceive : array[0..MAXBYTE] of Integer;
    FramesRobotInformation : array[0..4] of TFrameRobotInformation;
    IndexValuesReceive : Byte;
    MatchHistory : TStringList;
    MatchHistoryIndex : Integer;
    LgrMessage : Integer;
    procedure ReceiveValue(Value : Byte);
    function BytesToFloat(Byte1, Byte2, Byte3, Byte4 : Byte) : Single;
    function SoundSpeed(MmByTick : Single) : Single;
    procedure DoCommandReceive;
    procedure WriteMemo;
    function TimeWithMs : String;
    procedure ShowRobotInformation(RobotID : Byte);
    function GetCaseNumber(X, Y : Integer) : Byte;
    procedure LoadHistoryLine(LineText : String);
  public
    { Public declarations }
  end;

const
  CMD_SEND_US = $68;
  CMD_COORDONEES = $48;
  CMD_COORDONEES_WITH_THETA = $4C;
  CMD_SOUND_SPEED = $F4;

  LOG_MATCH_FOLDER = 'C:\Logs matchs\';

resourcestring
  rsSoundSpeed = 'Vitesse du son';
  rsMeterPerSecond = 'm/s';

var
  FormRadioAnalyzer: TFormRadioAnalyzer;

implementation

{$R *.dfm}

procedure TFormRadioAnalyzer.CommPortDriverReceiveData(Sender: TObject;
  DataPtr: Pointer; DataSize: Cardinal);
var p: ^byte;
    b:byte;
begin
  p := DataPtr;
  while DataSize > 0 do
  begin
    b:=p^;
    ReceiveValue(b);
    dec(DataSize);
    inc(p);
  end;
end;

procedure TFormRadioAnalyzer.FormCreate(Sender: TObject);
begin
  MatchHistory := TStringList.Create;

  BorderStyle := bsNone;

  Top := 0;
  Left := 0;

  Width := Screen.Width;
  Height := Screen.Height;


  //Remplir tableau FramesRobotInformation
  FramesRobotInformation[0] := FrameRobotInformation0;
  FramesRobotInformation[1] := FrameRobotInformation1;

  //Initialiser le nom des robots
  FramesRobotInformation[0].RobotName := 'Katrina';
  FramesRobotInformation[0].RobotColor := clRed;

  FramesRobotInformation[1].RobotName := 'Adversaire';
  FramesRobotInformation[1].RobotColor := clGray;


  //InfoRobot
  with RoboticTable.Robots[0] do
  begin
    Color := FramesRobotInformation[0].RobotColor;
    Size := 300;
  end;

  with RoboticTable.Robots[1] do
  begin
    Color := FramesRobotInformation[1].RobotColor;
    Size := 300;
  end;

  //Ouvrir le port COM
  CommPortDriver.Connect;
end;

procedure TFormRadioAnalyzer.FormClose(Sender: TObject; var Action: TCloseAction);
begin
  CommPortDriver.Disconnect;
  MatchHistory.Free;
end;

procedure TFormRadioAnalyzer.ReceiveValue(Value: Byte);
begin
  if IndexValuesReceive = 0 then
  begin
    LgrMessage := Value;
    IndexValuesReceive := IndexValuesReceive + 1;
  end
  else
  begin
    ValuesReceive[IndexValuesReceive - 1] := Value;
    if IndexValuesReceive = LgrMessage  then
    begin
      IndexValuesReceive := 0;
      DoCommandReceive;
    end
    else
    begin
      IndexValuesReceive := IndexValuesReceive + 1;
    end;
  end;
end;

function TFormRadioAnalyzer.BytesToFloat(Byte1, Byte2, Byte3, Byte4: Byte): Single;
var
  PResult : PByte;
begin
  PResult := @Result;
  PResult^ := Byte1;
  inc(PResult, sizeof(Byte));
  PResult^ := Byte2;
  inc(PResult, sizeof(Byte));
  PResult^ := Byte3;
  inc(PResult, sizeof(Byte));
  PResult^ := Byte4;
end;

function TFormRadioAnalyzer.SoundSpeed(MmByTick: Single): Single;
begin
  Result := MmByTick * 921.6;  //(7372800 / 8) / 1000
end;

procedure TFormRadioAnalyzer.DoCommandReceive;
var
  Command : Byte;
  Parameter : Byte;
begin
  WriteMemo;
  Command := ValuesReceive[0] and $FC;
  Parameter := ValuesReceive[0] and $03;
  case Command of
    CMD_COORDONEES:
                  begin
                    LockWindowUpdate(self.Handle); //Locker la fenêtre
                    ShowRobotInformation(Parameter);
                    LockWindowUpdate(0); //Délocker la fenêtre
                  end;
  end;
end;

procedure TFormRadioAnalyzer.WriteMemo;
var
  StringCommand : string;
  i : Integer;
begin
  StringCommand := TimeWithMs + ': ';
  for i := 0 to LgrMessage - 1 do
  begin
    StringCommand := StringCommand + IntToHex(ValuesReceive[i], 2) + ' ';
  end;
  MemoReceiveValues.Lines.Add(StringCommand);
end;

function TFormRadioAnalyzer.TimeWithMs: String;
begin
  Result := FormatDateTime('hh:nn:ss.zzz',Now);
end;

procedure TFormRadioAnalyzer.ShowRobotInformation(RobotID : Byte);
var
   BrutTheta, DegreeTheta : Integer;
begin
  //Mettre à jour frame RobotInformation
  FramesRobotInformation[RobotID].X := ((ValuesReceive[1] shl 8) + ValuesReceive[2]);
  FramesRobotInformation[RobotID].Y := ((ValuesReceive[3] shl 8) + ValuesReceive[4]);


  BrutTheta := ((ValuesReceive[5] shl 8) + ValuesReceive[6]);
  DegreeTheta := Trunc((BrutTheta / 65535) * 360);

  FramesRobotInformation[RobotID].Theta := DegreeTheta;

  FramesRobotInformation[RobotID].CaseNumber := GetCaseNumber(((ValuesReceive[1] shl 8) + ValuesReceive[2]),
                                                              ((ValuesReceive[3] shl 8) + ValuesReceive[4]));

  //Mettre à jour dessin
  RoboticTable.Robots[RobotID].X := ((ValuesReceive[1] shl 8) + ValuesReceive[2]);
  RoboticTable.Robots[RobotID].Y := ((ValuesReceive[3] shl 8) + ValuesReceive[4]);
  RoboticTable.Robots[RobotID].Theta := DegreeTheta;
end;

function TFormRadioAnalyzer.GetCaseNumber(X, Y: Integer): Byte;
begin
   Result := 0;
end;

procedure TFormRadioAnalyzer.Button1Click(Sender: TObject);
begin
  IndexValuesReceive := 0;
end;

procedure TFormRadioAnalyzer.TimerMatchTimer(Sender: TObject);
begin
  MatchTimeCount := MatchTimeCount + 1;
  LabelTimerMatch.Caption := Format('%.2d secondes', [MatchTimeCount]);
  if (MatchTimeCount = 90) then
  begin
    TimerMatch.Enabled := False;
    MemoReceiveValues.Lines.Add(TimeWithMs + ': Fin du match');
    MemoReceiveValues.Lines.SaveToFile(IncludeTrailingPathDelimiter(LOG_MATCH_FOLDER) + 'Match ' + FormatDateTime('hh nn ss',Now) + '.txt');
  end;
end;

procedure TFormRadioAnalyzer.ButtonStartMatchClick(Sender: TObject);
begin
  MatchTimeCount := 0;
  LabelTimerMatch.Caption := Format('%.2d secondes', [MatchTimeCount]);
  TimerMatch.Enabled := True;
  MemoReceiveValues.Clear;
  MemoReceiveValues.Lines.Add(TimeWithMs + ': Début du match');
end;

procedure TFormRadioAnalyzer.RadioGroupModeClick(Sender: TObject);
begin
  if RadioGroupMode.ItemIndex = 1 then
  begin
    CommPortDriver.Disconnect;
    if OpenDialog.Execute then
    begin
      MatchHistory.LoadFromFile(OpenDialog.FileName);
      MatchHistoryIndex := 0;
    end;
  end
  else
  begin
    CommPortDriver.Connect;
  end;
end;

procedure TFormRadioAnalyzer.ButtonNextClick(Sender: TObject);
begin
  if MatchHistoryIndex < MatchHistory.Count then
  begin
    MatchHistoryIndex := MatchHistoryIndex + 1;
    LoadHistoryLine(MatchHistory[MatchHistoryIndex]);
    DoCommandReceive;
  end;
end;

procedure TFormRadioAnalyzer.LoadHistoryLine(LineText: String);
begin
  if (Length(LineText)) = 32 then
  begin
    ValuesReceive[5] := StrToInt('$' + Copy(LineText, Length(LineText) - 2, 2));
    ValuesReceive[4] := StrToInt('$' + Copy(LineText, Length(LineText) - 5, 2));
    ValuesReceive[3] := StrToInt('$' + Copy(LineText, Length(LineText) - 8, 2));
    ValuesReceive[2] := StrToInt('$' + Copy(LineText, Length(LineText) - 11, 2));
    ValuesReceive[1] := StrToInt('$' + Copy(LineText, Length(LineText) - 14, 2));
    ValuesReceive[0] := StrToInt('$' + Copy(LineText, Length(LineText) - 17, 2));
  end;
end;

procedure TFormRadioAnalyzer.ButtonPreviousClick(Sender: TObject);
begin
  if MatchHistoryIndex > 0 then
  begin
    MatchHistoryIndex := MatchHistoryIndex - 1;
    LoadHistoryLine(MatchHistory[MatchHistoryIndex]);
    DoCommandReceive;
  end;
end;

end.
