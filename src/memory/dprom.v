// This is rom with an extra write port that can be used to initialize it
module dprom #(
  parameter MEM_INIT_FILE = "",
  parameter DATA_WIDTH = 8,
  parameter DEPTH = 16384,
  parameter ADDRESS_WIDTH = $clog2(DEPTH)
) ( 
  input                       clk,
  input [ADDRESS_WIDTH-1:0]   addr,
  output reg [DATA_WIDTH-1:0] dout,
  input                       we_b,
  input [ADDRESS_WIDTH-1:0]   addr_b,
  input [DATA_WIDTH-1:0]      din_b
);

  reg [DATA_WIDTH:0] rom [0:DEPTH-1];

  initial
    if (MEM_INIT_FILE != "")
      $readmemh(MEM_INIT_FILE, rom);
   
  always @(posedge clk) begin
    dout <= rom[addr];
  end

  always @(posedge clk) begin
    if (we_b)
      rom[addr_b] <= din_b;
  end

endmodule
