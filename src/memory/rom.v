module rom #(
  parameter MEM_INIT_FILE = "",
  parameter DATA_WIDTH = 8,
  parameter DEPTH = 16384,
  parameter ADDRESS_WIDTH = $clog2(DEPTH)
) ( 
  input                       clk,
  input [ADDRESS_WIDTH-1:0]   addr,
  output reg [DATA_WIDTH-1:0] dout
);

  reg [DATA_WIDTH:0] rom [0:DEPTH-1];

  initial
    if (MEM_INIT_FILE != "")
      $readmemh(MEM_INIT_FILE, rom);
   
  always @(posedge clk) begin
    dout <= rom[addr];
  end

endmodule
