module ram #(
  parameter MEM_INIT_FILE = "",
  parameter DATA_WIDTH = 8,
  parameter DEPTH = 16384,
  parameter ADDRESS_WIDTH = $clog2(DEPTH)
) (
  input                       clk,
  input                       we,
  input [ADDRESS_WIDTH-1:0]   addr,
  input [DATA_WIDTH-1:0]      din,
  output reg [DATA_WIDTH-1:0] dout
);

  reg [DATA_WIDTH-1:0] ram[0:DEPTH-1];

  initial
    if (MEM_INIT_FILE != "")
      $readmemh(MEM_INIT_FILE, ram);

  always @(posedge clk) begin
    if (we)
      ram[addr] <= din;
    dout <= ram[addr];
  end

endmodule
