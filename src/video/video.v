`default_nettype none
module video (
  input         clk,
  input         reset,
  output [7:0]  vga_r,
  output [7:0]  vga_b,
  output [7:0]  vga_g,
  output        vga_hs,
  output        vga_vs,
  output        vga_de,
  input  [7:0]  vga_data,
  output [13:0] vga_addr,
  input [1:0]   mode,
  input [4:0]   border_color,
  input [79:0]  colors,
  output        n_int
);

  parameter HA = 640;
  parameter HS  = 96;
  parameter HFP = 16;
  parameter HBP = 48;
  parameter HT  = HA + HS + HFP + HBP;
  parameter HB = 0;
  parameter HB2 = HB/2-8; // NOTE pixel coarse H-adjust
  parameter HDELAY = 3; // NOTE pixel fine H-adjust
  parameter HB_ADJ = 4; // NOTE border H-adjust

  parameter VA = 480;
  parameter VS  = 2;
  parameter VFP = 11;
  parameter VBP = 31;
  parameter VT  = VA + VS + VFP + VBP;
  parameter VB = 40;
  parameter VB2 = VB/2;

  reg [9:0] hc = 0;
  reg [9:0] vc = 0;
  reg INT = 0;
  reg[5:0] intCnt = 1;

  reg [23:0] palette [0:26];

  initial begin
    palette[0]  = 24'h000000;
    palette[1]  = 24'h000080;
    palette[2]  = 24'h0000ff;
    palette[3]  = 24'h800000;
    palette[4]  = 24'h800080;
    palette[5]  = 24'h8000ff;
    palette[6]  = 24'hff0000;
    palette[7]  = 24'hff0080;
    palette[8]  = 24'hff00ff;
    palette[9]  = 24'h008000;
    palette[10] = 24'h008080;
    palette[11] = 24'h0080ff;
    palette[12] = 24'h808000;
    palette[13] = 24'h808080;
    palette[14] = 24'h8080ff;
    palette[15] = 24'hff8000;
    palette[16] = 24'hff8080;
    palette[17] = 24'hff80ff;
    palette[18] = 24'h000000;
    palette[19] = 24'h00ff00;
    palette[20] = 24'h00ffff;
    palette[21] = 24'h000000;
    palette[22] = 24'h80ff00;
    palette[23] = 24'h80ffff;
    palette[24] = 24'hffff00;
    palette[25] = 24'hffff80;
    palette[26] = 24'hffffff;
  end

  assign n_int = !INT;

  // Set horizontal and vertical counters
  always @(posedge clk) begin
    if (hc == HT - 1) begin
      hc <= 0;
      if (vc == VT - 1) vc <= 0;
      else vc <= vc + 1;
    end else hc <= hc + 1;
    if (hc == HA + HFP && vc == VA + VFP) begin
      INT <= 1;
    end
    if (INT) intCnt <= intCnt + 1;
    if (!intCnt) INT <= 0;
  end

  assign vga_hs = !(hc >= HA + HFP && hc < HA + HFP + HS);
  assign vga_vs = !(vc >= VA + VFP && vc < VA + VFP + VS);
  assign vga_de = !(hc >= HA || vc >= VA);

  wire [9:0] x = hc - HB;
  wire [7:0] y = vc[9:1] - VB2;

  wire [9:0] x8 = x + 8;

  wire h_border = (hc < (HB + HB_ADJ) || hc >= (HA - HB + HB_ADJ));
  wire v_border = (vc < VB || vc >= VA - VB);
  wire border = h_border || v_border;

  reg [7:0] pixels;

  // Read video memory
  always @(posedge clk) begin
    vga_addr <= {y[2:0], 11'b0} + (y[7:3] * 80) + x8[9:3];
    if (x[2:0] == 7) pixels <= vga_data;
    else begin
      if (mode == 0 && x[2:0] == 3) pixels <= {pixels[6:0], 1'b0};
      if (mode == 1 && x[0] == 1) pixels <= {pixels[6:0], 1'b0};
      if (mode == 2) pixels <= {pixels[6:0], 1'b0};
    end
  end

  reg [3:0] pix;

  always @* begin
    case (mode) 
      0: pix = {pixels[7], pixels[3], pixels[5], pixels[1]};
      1: pix = {pixels[7], pixels[3]};
      2: pix = pixels[7];
      3: pix = {pixels[7], pixels[3]};
    endcase
  end

  wire [4:0] color = colors[(pix * 5) + 4 -: 4];
  wire [4:0] col = border ? border_color : color;

  wire [7:0] red = palette[col][23:16];
  wire [7:0] green = palette[col][15:8];
  wire [7:0] blue = palette[col][7:0];

  assign vga_r = !vga_de ? 8'b0 : red;
  assign vga_g = !vga_de ? 8'b0 : green;
  assign vga_b = !vga_de ? 8'b0 : blue;

endmodule

