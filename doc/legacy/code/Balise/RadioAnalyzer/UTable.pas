unit UTable;

interface

uses
  Windows, Messages, SysUtils, Classes, Controls, ExtCtrls, Graphics, Dialogs, Math;

type
  TRoboticTable = class;

  TRobot = class
  private
    FTheta: Integer;
    FX: Integer;
    FY: Integer;
    FRoboticTable : TRoboticTable;
    FID: Byte;
    FColor: TColor;
    FSize: Integer;
    procedure SetColor(const Value: TColor);
    procedure SetSize(const Value: Integer);
    procedure SetTheta(const Value: Integer);
    procedure SetX(const Value: Integer);
    procedure SetY(const Value: Integer);
  public
    procedure Draw;
    constructor Create(RoboticTable: TRoboticTable; ID : Byte); reintroduce;
  published
    property X : Integer read FX write SetX;
    property Y : Integer read FY write SetY;
    property Theta : Integer read FTheta write SetTheta;
    property ID : Byte read FID;
    property Color : TColor read FColor write SetColor;
    property Size : Integer read FSize write SetSize;
  end;

  TRoboticTable = class(TGraphicControl)
  private
    FColorTable1: TColor;
    FColorTable2: TColor;
    FNbCasesHeight: Integer;
    FNbCasesWidth: Integer;
    FRealTableWidth: Double;
    FRealTableHeight: Double;
    FNbRobots: Integer;
    FColorTable3: TColor;
    FRealFosseSize: Integer;
    procedure SetColorTable1(const Value: TColor);
    procedure SetColorTable2(const Value: TColor);
    procedure DrawTable;
    procedure SetNbCasesHeight(const Value: Integer);
    procedure SetNbCasesWidth(const Value: Integer);
    procedure SetRealTableHeight(const Value: Double);
    procedure SetRealTableWidth(const Value: Double);
    procedure SetNbRobots(const Value: Integer);
    procedure SetColorTable3(const Value: TColor);
    procedure SetRealFosseSize(const Value: Integer);
    { Private declarations }
  protected
    { Protected declarations }
  public
    { Public declarations }
    Robots : array[0..3] of TRobot;
    constructor Create(AOwner: TComponent); override;
    destructor Destroy; override;
    function RealSizeToPixelWidth(RealSize : Double) : Integer;
    function RealSizeToPixelHeight(RealSize : Double) : Integer;
    function PixelWidthToRealSize(PixelWidth : Integer) : Double;
    function PixelHeightToRealSize(PixelHeight : Integer) : Double;
  published
    { Published declarations }
    procedure Paint; override;
    property ColorTable1 : TColor read FColorTable1 write SetColorTable1;
    property ColorTable2 : TColor read FColorTable2 write SetColorTable2;
    property ColorTable3 : TColor read FColorTable3 write SetColorTable3;
    property NbCasesHeight : Integer read FNbCasesHeight write SetNbCasesHeight;
    property NbCasesWidth : Integer read FNbCasesWidth write SetNbCasesWidth;
    property RealTableWidth : Double read FRealTableWidth write SetRealTableWidth;
    property RealTableHeight : Double read FRealTableHeight write SetRealTableHeight;
    property NbRobots : Integer read FNbRobots write SetNbRobots;
    property RealFosseSize : Integer read FRealFosseSize write SetRealFosseSize;
  end;

procedure Register;

implementation

procedure Register;
begin
  RegisterComponents('Robotique', [TRoboticTable]);
end;

{ TRoboticTable }

constructor TRoboticTable.Create(AOwner: TComponent);
var
  i : Integer;
begin
  inherited;
  Self.Caption := EmptyStr;
  Self.Height := 150;
  Self.Width := 150;
  Self.FColorTable1 := clGreen;
  Self.FColorTable2 := clYellow;
  Self.FNbCasesHeight := 10;
  Self.FNbCasesWidth := 5;
  for i := 0 to 3 do
    Robots[i] := TRobot.Create(self, i);

end;

procedure TRoboticTable.DrawTable;
var
  CarreWidth, CarreHeight : Integer;
  i, j : Integer;
  Ravin : TRect;
begin
  //Calculer la taille des carrés.
  CarreWidth := (Self.Width - RealSizeToPixelWidth(FRealFosseSize)) div NbCasesWidth;
  CarreHeight := Self.Height div NbCasesHeight;

  //Dessiner le fond
  Self.Canvas.Brush.Color := FColorTable3;
  Ravin.Top := 0;
  Ravin.Left := (CarreWidth * (NbCasesWidth div 2));
  Ravin.Right := Ravin.Left + RealSizeToPixelWidth(FRealFosseSize);
  Ravin.Bottom := Height;

  Self.Canvas.Rectangle(Ravin);

  //Dessiner les carrés
  Self.Canvas.Pen.Color := clBlack; //Bord noir
  for i := 0 to NbCasesWidth - 1 do
  begin
    for j := 0 to NbCasesHeight - 1 do
    begin
      if (i mod 2 = 0) then
        if (j mod 2 = 0) then
          Self.Canvas.Brush.Color := ColorTable1
        else
          Self.Canvas.Brush.Color := ColorTable2
      else
        if (j mod 2 = 0) then
          Self.Canvas.Brush.Color := ColorTable2
        else
          Self.Canvas.Brush.Color := ColorTable1;
      if (i < (NbCasesWidth div 2)) then
        Self.Canvas.Rectangle(i* CarreWidth, j* CarreHeight, (i+1) * CarreWidth, (j+1) * CarreHeight)
      else
        Self.Canvas.Rectangle(i* CarreWidth + RealSizeToPixelWidth(FRealFosseSize), j* CarreHeight, (i+1) * CarreWidth + RealSizeToPixelWidth(FRealFosseSize), (j+1) * CarreHeight);
      //Dessiner le ravin si besoin


    end;
  end;
  Self.Width := (NbCasesWidth * CarreWidth) + RealSizeToPixelWidth(FRealFosseSize);
  Self.Height := NbCasesHeight * CarreHeight;
end;

procedure TRoboticTable.Paint;
var
  i : Integer;
begin
  inherited;
  DrawTable;
  for i := 1 to NbRobots do
    Robots[i - 1].Draw;
end;

function TRoboticTable.PixelHeightToRealSize(PixelHeight: Integer): Double;
begin     
  Result := (PixelHeight / Self.Height) * RealTableHeight;
end;

function TRoboticTable.PixelWidthToRealSize(PixelWidth: Integer): Double;
begin
  Result := (PixelWidth / Self.Width) * RealTableWidth;
end;

function TRoboticTable.RealSizeToPixelHeight(RealSize: Double): Integer;
begin
  Result := trunc((RealSize / RealTableHeight) * Self.Height);
end;

function TRoboticTable.RealSizeToPixelWidth(RealSize: Double): Integer;
begin
  if RealTableWidth <> 0 then
    Result := trunc((RealSize / RealTableWidth) * Self.Width)
  else
    Result := 0;
end;

procedure TRoboticTable.SetColorTable1(const Value: TColor);
begin
  FColorTable1 := Value;
  Invalidate;
end;

procedure TRoboticTable.SetColorTable2(const Value: TColor);
begin
  FColorTable2 := Value;
  Invalidate;
end;

procedure TRoboticTable.SetNbCasesHeight(const Value: Integer);
begin
  FNbCasesHeight := Value;
  Invalidate;
end;

procedure TRoboticTable.SetNbCasesWidth(const Value: Integer);
begin
  FNbCasesWidth := Value;
  Invalidate;
end;

procedure TRoboticTable.SetRealTableHeight(const Value: Double);
begin
  FRealTableHeight := Value;
  Invalidate;
end;

procedure TRoboticTable.SetRealTableWidth(const Value: Double);
begin
  FRealTableWidth := Value;
  Invalidate;
end;

procedure TRoboticTable.SetNbRobots(const Value: Integer);
begin
  FNbRobots := Value;
  Invalidate;
end;

//TODO peut-etre a effacer
{function TRoboticTable.GetInverseRobotY: Double;
begin
  Result := FRealTableHeight - RobotY;
end;

procedure TRoboticTable.SetInverseRobotY(const Value: Double);
begin
  RobotY := FRealTableHeight - Value;
end; }

procedure TRoboticTable.SetColorTable3(const Value: TColor);
begin
  FColorTable3 := Value;
  Invalidate;
end;

procedure TRoboticTable.SetRealFosseSize(const Value: Integer);
begin
  FRealFosseSize := Value;
  Invalidate;
end;

destructor TRoboticTable.Destroy;
var
  i : Integer;
begin
  for i := 0 to 3 do
    Robots[i].Free;
  inherited;
end;

{ TRobot }


constructor TRobot.Create(RoboticTable: TRoboticTable; ID: Byte);
begin
  FRoboticTable := RoboticTable;
  FID := ID;
end;

procedure TRobot.Draw;
var
  DiameterPixelWidth, DiameterPixelHeight  : Integer;
begin
  DiameterPixelHeight := FRoboticTable.RealSizeToPixelHeight(Size);
  DiameterPixelWidth := FRoboticTable.RealSizeToPixelWidth(Size);

  FRoboticTable.Canvas.Brush.Color := Color;

  FRoboticTable.Canvas.Ellipse(Trunc(FRoboticTable.RealSizeToPixelWidth(Y) - (DiameterPixelHeight / 2)),
                               Trunc(FRoboticTable.RealSizeToPixelHeight(X) - (DiameterPixelWidth / 2)),
                               Trunc(FRoboticTable.RealSizeToPixelWidth(Y) + (DiameterPixelHeight / 2)),
                               Trunc(FRoboticTable.RealSizeToPixelHeight(X) + (DiameterPixelWidth / 2)));
  //FRoboticTable.Canvas.MoveTo(FRoboticTable.RealSizeToPixelWidth(Y), FRoboticTable.RealSizeToPixelHeight(x));

end;

procedure TRobot.SetColor(const Value: TColor);
begin
  if (FColor <> Value) then
  begin
    FColor := Value;
    Draw; //Pas besoin de repeindre la table car se repeind au même endroit
  end;
end;

procedure TRobot.SetSize(const Value: Integer);
begin
  if (FSize <> Value) then
  begin
    FSize := Value;
    FRoboticTable.Invalidate;
  end;
end;

procedure TRobot.SetTheta(const Value: Integer);
begin
  if (FTheta <> Value) then
  begin
    FTheta := Value;
    FRoboticTable.Invalidate;
  end;
end;

procedure TRobot.SetX(const Value: Integer);
begin
  if (FX <> Value) then
  begin
    FX := Value;
    FRoboticTable.Invalidate;
  end;
end;

procedure TRobot.SetY(const Value: Integer);
begin
  if (FY <> Value) then
  begin
    FY := Value;
    FRoboticTable.Invalidate;
  end;
end;

end.
