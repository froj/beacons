unit FrameInfoRobot;

interface

uses 
  Windows, Messages, SysUtils, Classes, Graphics, Controls, Forms, Dialogs,
  StdCtrls;

type
  TFrameRobotInformation = class(TFrame)
    LabelRobotName: TLabel;
    LabelXTitle: TLabel;
    LabelYTitle: TLabel;
    LabelCaseNumberTitle: TLabel;
    LabelThetaTitle: TLabel;
    LabelX: TLabel;
    LabelY: TLabel;
    LabelCaseNumber: TLabel;
    LabelTheta: TLabel;
  private
    FX: Integer;
    FCaseNumber: Byte;
    FY: Integer;
    FRobotName: String;
    FTheta: Integer;
    FRobotColor: TColor;
    procedure SetX(const Value: Integer);
    procedure SetCaseNumber(const Value: Byte);
    procedure SetY(const Value: Integer);
    procedure SetRobotName(const Value: String);
    procedure SetTheta(const Value: Integer);
    procedure SetRobotColor(const Value: TColor);
    { Private declarations }
  public
    { Public declarations }
  published
    property RobotName : String read FRobotName write SetRobotName;
    property X : Integer read FX write SetX;
    property Y : Integer read FY write SetY;
    property CaseNumber : Byte read FCaseNumber write SetCaseNumber;
    property Theta : Integer read FTheta write SetTheta;
    property RobotColor : TColor read FRobotColor write SetRobotColor;
  end;

implementation

{$R *.dfm}

{ TFrameRobotInformation }

procedure TFrameRobotInformation.SetRobotName(const Value: String);
begin
  FRobotName := Value;
  LabelRobotName.Caption := Value;
end;

procedure TFrameRobotInformation.SetX(const Value: Integer);
begin
  FX := Value;
  LabelX.Caption := IntToStr(Value);
end;

procedure TFrameRobotInformation.SetY(const Value: Integer);
begin
  FY := Value;
  LabelY.Caption := IntToStr(Value);
end;

procedure TFrameRobotInformation.SetCaseNumber(const Value: Byte);
begin
  FCaseNumber := Value;
  LabelCaseNumber.Caption := IntToStr(Value);
end;

procedure TFrameRobotInformation.SetTheta(const Value: Integer);
begin
  FTheta := Value;
  LabelTheta.Caption := IntToStr(Value);
end;

procedure TFrameRobotInformation.SetRobotColor(const Value: TColor);
begin
  FRobotColor := Value;
  LabelRobotName.Font.Color := Value;
end;

end.
