`default_nettype none
module top #(
  parameter c_vga_out      = 0,  // 0; Just HDMI, 1: VGA and HDMI
  parameter c_acia_serial  = 0,  // 0: disabled, 1: ACIA serial
  parameter c_esp32_serial = 1,  // 0: disabled, 1: ESP32 serial (micropython console)
  parameter c_sdram        = 0,  // SDRAM or BRAM 
  parameter c_keyboard     = 1,  // Include keyboard support
  parameter c_diag         = 0,  // 0: No led diagnostcs, 1: led diagnostics 
  parameter c_speed        = 2,  // CPU speed = 25 / 2 ** (c_speed + 1) MHz
  parameter c_reset        = 15, // Bits (minus 1) in power-up reset counter
  parameter c_lcd_hex      = 1   // SPI LCD HEX decoder
) (
  input         clk_25mhz,
  // Buttons
  input [6:0]   btn,
  // HDMI
  output [3:0]  gpdi_dp,
  output [3:0]  gpdi_dn,
  // Keyboard
  input         usb_fpga_bd_dp,
  input         usb_fpga_bd_dn,
  output        usb_fpga_pu_dp,
  output        usb_fpga_pu_dn,
  // Audio
  output [3:0]  audio_l,
  output [3:0]  audio_r,
  // ESP32 passthru
  input         ftdi_txd,
  output        ftdi_rxd,
  input         wifi_txd,
  output        wifi_rxd,  // SPI from ESP32
  input         wifi_gpio16,
  input         wifi_gpio5,
  output        wifi_gpio0,

  inout  sd_clk, sd_cmd,
  inout   [3:0] sd_d,

  output sdram_csn,       // chip select
  output sdram_clk,       // clock to SDRAM
  output sdram_cke,       // clock enable to SDRAM
  output sdram_rasn,      // SDRAM RAS
  output sdram_casn,      // SDRAM CAS
  output sdram_wen,       // SDRAM write-enable
  output [12:0] sdram_a,  // SDRAM address bus
  output  [1:0] sdram_ba, // SDRAM bank-address
  output  [1:0] sdram_dqm,// byte select
  inout  [15:0] sdram_d,  // data bus to/from SDRAM

  inout  [27:0] gp,gn,
  // SPI display
  output        oled_csn,
  output        oled_clk,
  output        oled_mosi,
  output        oled_dc,
  output        oled_resn,
  // Leds
  output [7:0]  led
);

  // ===============================================================
  // CPU signals
  // ===============================================================
  wire          n_wr;
  wire          n_rd;
  wire          n_int;
  wire          n_mreq;
  wire          n_iorq;
  wire          n_m1;
  
  // Buses
  wire [15:0]   cpu_address;
  wire [7:0]    cpu_data_out;
  reg [7:0]     cpu_data_in;
  wire [15:0]   pc;

  // Derived signals
  wire n_iowr  = n_wr | n_iorq;
  wire n_memwr = n_wr | n_mreq;
  wire n_iord  = n_rd | n_iorq;
  wire n_memrd = n_rd | n_mreq;
  
  // Chip selects
  wire          n_ram_cs;
  
  wire          tdata_cs;
  wire          tctrl_cs;

  wire          ga_cs;
  wire          rom_bank_cs;
  wire          ppi_a_cs;
  wire          ppi_b_cs;
  wire          ppi_c_cs;
  wire          ppi_control_cs;

  reg [7:0]     ppi_a;
  reg [7:0]     ppi_c;
  reg [7:0]     ppi_control;
  wire          ppi_a_input = ppi_control[4];

  // Miscellaneous signals
  wire [7:0]    acia_dout;
  reg [6:0]     r_btn_joy;
  reg [7:0]     r_cpu_control;
  wire          spi_load = r_cpu_control[1];

  reg [1:0]     mode;
  reg [3:0]     pen;
  reg [9:0]     scr_start;
  reg [4:0]     r_crtc;
  reg [4:0]     border_color;
  reg           border_selected;
  reg [4:0]     colors [0:15];
  reg           hi_rom_disable;
  reg           lo_rom_disable;
  reg [2:0]     ram_bank;
  reg [2:0]     ram_config;
  reg [3:0]     rom_bank;
  wire [7:0]    ram_out;
  wire [7:0]    dpram_out;
  wire [7:0]    rom_out;
  wire [7:0]    kbd_out;
  wire          key_nmi;

  wire   [7:0]  red;
  wire   [7:0]  green;
  wire   [7:0]  blue;
  wire          hsync;
  wire          vsync;
  wire          vga_de;
  
  // ===============================================================
  // System Clock generation
  // ===============================================================
  wire clk_sdram_locked;
  wire [3:0] clocks;
  ecp5pll
  #(
      .in_hz( 25*1000000),
    .out0_hz(125*1000000),
    .out1_hz( 25*1000000),
    .out2_hz(100*1000000),                // SDRAM core
    .out3_hz(100*1000000), .out3_deg(180) // SDRAM chip 45-330:ok 0-30:not
  )
  ecp5pll_inst
  (
    .clk_i(clk_25mhz),
    .clk_o(clocks),
    .locked(clk_sdram_locked)
  );
  wire clk_hdmi  = clocks[0];
  wire clk_vga   = clocks[1];
  wire clk_cpu  = clocks[1];
  wire clk_sdram = clocks[2];
  wire sdram_clk = clocks[3]; // phase shifted for chip

  // ===============================================================
  // CPU clock generation
  // ===============================================================
  reg [c_speed:0] cpu_clk_count;
  
  always @(posedge clk_cpu) begin
    cpu_clk_count <= cpu_clk_count + 1;
  end

  wire cpu_clk_enable = cpu_clk_count[c_speed]; 

  // ===============================================================
  // Reset generation
  // ===============================================================
  reg [c_reset:0] pwr_up_reset_counter = 0;
  wire            pwr_up_reset_n = &pwr_up_reset_counter;
  wire            n_reset = pwr_up_reset_n & btn[0] & ~r_cpu_control[0];
  wire            reset = ~n_reset;

  always @(posedge clk_cpu) begin
     if (!pwr_up_reset_n)
       pwr_up_reset_counter <= pwr_up_reset_counter + 1;
  end

  // ===============================================================
  // Chip selects
  // ===============================================================
  assign n_ram_cs = 1'b0;
  assign tctrl_cs = 1'b0;
  assign tdata_cs = 1'b0;

  assign ga_cs = cpu_address[15:14] == 2'b01;
  assign rom_bank_cs = cpu_address[13] == 1'b0;

  assign ppi_a_cs = cpu_address[15:8] == 8'hf4;
  assign ppi_b_cs = cpu_address[15:8] == 8'hf5;
  assign ppi_c_cs = cpu_address[15:8] == 8'hf6;
  assign ppi_control_cs = cpu_address[15:8] == 8'hf7;

  // ===============================================================
  // Memory decoding
  // ===============================================================
  always @* begin
    cpu_data_in = 0;
    if (n_memrd == 1'b0) begin
      cpu_data_in = ram_out;
      case (cpu_address[15:14])
        0: if (!lo_rom_disable) cpu_data_in = rom_out;
        3: if (!hi_rom_disable) cpu_data_in = rom_out; else cpu_data_in = dpram_out;
      endcase
    end else if (n_iord == 1'b0) begin
      if (ppi_b_cs) cpu_data_in = 8'h1e | ~vsync; // Amstrad branding
      if (ppi_a_cs) cpu_data_in = kbd_out;
    end
  end

  // ===============================================================
  // CPU
  // ===============================================================
  tv80n cpu1 (
    .reset_n(n_reset),
    .clk(cpu_clk_enable),
    .wait_n(~spi_load & ~r_btn_joy[1]),
    .int_n(n_int),
    .nmi_n(1'b1),
    .busrq_n(1'b1),
    .mreq_n(n_mreq),
    .m1_n(n_m1),
    .iorq_n(n_iorq),
    .wr_n(n_wr),
    .rd_n(n_rd),
    .A(cpu_address),
    .di(cpu_data_in),
    .do(cpu_data_out),
    .pc(pc)
  );

  // ===============================================================
  // CPC OUTs
  // ===============================================================
  always @(posedge clk_cpu) begin
    if (n_iowr == 1'b0) begin
      if (ga_cs) begin
        case (cpu_data_out[7:6])
          0: begin
               border_selected <= cpu_address[4];
               if (!cpu_data_out[4]) pen <= cpu_data_out[3:0];
             end
          1: begin
               if (border_selected) border_color = cpu_data_out[4:0];
               else colors[pen] = cpu_data_out[4:0];
             end
          2: begin
               hi_rom_disable <= cpu_data_out[3];
               lo_rom_disable <= cpu_data_out[2];
               mode <= cpu_data_out[1:0];
             end
          3: begin
               ram_bank <= cpu_data_out[5:3];
               ram_config = cpu_data_out[2:0];
             end
        endcase
      end else if (rom_bank_cs) begin
        rom_bank <= cpu_data_out[3:0];
      end else if (ppi_a_cs) begin
        ppi_a <= cpu_data_out;
      end else if (ppi_c_cs) begin
        ppi_c <= cpu_data_out;
      end else if (ppi_control_cs) begin
        ppi_control <= cpu_data_out;
      end else if (cpu_address[15:8] == 8'hbc) begin
        r_crtc <= cpu_data_out[4:0];
      end else if (cpu_address[15:8] == 8'hbd) begin
        if (r_crtc == 12) scr_start[9:8] <= cpu_data_out[1:0];
        if (r_crtc == 13) scr_start[7:0] <= cpu_data_out;
      end
    end
  end

  // ===============================================================
  // Joystick for OSD control and games
  // ===============================================================
  always @(posedge clk_cpu) r_btn_joy <= btn;

  // ===============================================================
  // Keyboard
  // ===============================================================
  assign usb_fpga_pu_dp = 1; // pull-ups for us2 connector
  assign usb_fpga_pu_dn = 1;

  wire [10:0] ps2_key;
  wire [24:0] ps2_mouse = 0;
  wire        keypad_mod = 0;
  wire        right_shift_mod = 0;
  wire [9:0]  fn;

  generate
    if (c_keyboard) begin
      // Get PS/2 keyboard events
      ps2 ps2_kbd (
       .clk(clk_cpu),
       .ps2_clk(usb_fpga_bd_dp),
       .ps2_data(usb_fpga_bd_dn),
       .ps2_key(ps2_key)
      );
    end

    hid keyboard(
      .clk(clk_cpu),
      .reset(reset),
      .key_strobe(ps2_key[10]),
      .key_pressed(ps2_key[9]),
      .key_extended(ps2_key[8]),
      .key_code(ps2_key[7:0]),
      .right_shift_mod(right_shift_mod),
      .keypad_mod(keypad_mod),
      .ps2_mouse(ps2_mouse),
      .X(kbd_out),
      .Y(ppi_c[3:0]),
      .Fn(fn),
      .key_nmi(key_nmi)
    );
       
  endgenerate

  // ===============================================================
  // SPI Slave from ESP32
  // ===============================================================
  wire        spi_ram_wr, spi_ram_rd;
  wire [31:0] spi_ram_addr;
  wire [7:0]  spi_ram_di;
  wire [7:0]  spi_ram_do = ram_out;
  wire        irq;

  assign sd_d[3] = 1'bz; // FPGA pin pullup sets SD card inactive at SPI bus
  assign wifi_gpio0 = ~irq;

  spi_ram_btn #(
    .c_sclk_capable_pin(1'b0),
    .c_addr_bits(32)
  ) spi_ram_btn_inst (
    .clk(clk_cpu),
    .csn(~wifi_gpio5),
    .sclk(wifi_gpio16),
    .mosi(sd_d[1]), // wifi_gpio4
    .miso(sd_d[2]), // wifi_gpio12
    .btn(r_btn_joy),
    .irq(irq),
    .wr(spi_ram_wr),
    .rd(spi_ram_rd),
    .addr(spi_ram_addr),
    .data_in(spi_ram_do),
    .data_out(spi_ram_di)
  );

  always @(posedge clk_cpu) begin
    if (spi_ram_wr && spi_ram_addr[31:24] == 8'hFF) begin
      r_cpu_control <= spi_ram_di;
    end
  end

  // ===============================================================
  // RAM
  // ===============================================================
  wire [7:0]  vid_out;
  wire [13:0] vga_addr;
  wire [10:0] adj = vga_addr[10:0] + {scr_start, 1'b0};
  wire [13:0] vga_addr_adj = {vga_addr[13:11], adj}; 

  generate
    if (c_sdram == 0) begin
      ram #( 
        .DATA_WIDTH(8),
        .DEPTH(48 * 1024)
      ) ram48 (
        .clk(clk_cpu),
        .we(n_memwr == 1'b0 && cpu_address[15:14] < 3),
        .addr(cpu_address),
        .din(cpu_data_out),
        .dout(ram_out)
      );

      dpram #( 
        .DATA_WIDTH(8),
        .DEPTH(16 * 1024)
      ) vram (
        .clk_a(clk_cpu),
        .we_a(n_memwr == 1'b0 && cpu_address[15:14] == 3),
        .addr_a(cpu_address[13:0]),
        .din_a(cpu_data_out),
        .dout_a(dpram_out),
        .clk_b(clk_vga),
        .addr_b(vga_addr_adj),
        .dout_b(vid_out)
      );

      rom #( 
        .DATA_WIDTH(8),
        .DEPTH(32 * 1024),
        .MEM_INIT_FILE("../../roms/boot.mem")
      ) rom32 (
        .clk(clk_cpu),
        .addr(cpu_address[14:0]),
        .dout(rom_out)
      );

    end else begin
      wire sdram_d_wr;
      wire [15:0] sdram_d_in, sdram_d_out;
      wire [23:0] sdram_address = {8'b0, cpu_address};

      assign sdram_d = sdram_d_wr ? sdram_d_out : 16'hzzzz;
      assign sdram_d_in = sdram_d;

      sdram sdram_i (
       .sd_data_in(sdram_d_in),
       .sd_data_out(sdram_d_out),
       .sd_addr(sdram_a),
       .sd_dqm(sdram_dqm),
       .sd_cs(sdram_csn),
       .sd_ba(sdram_ba),
       .sd_we(sdram_wen),
       .sd_ras(sdram_rasn),
       .sd_cas(sdram_casn),
       // system interface
       .clk(clk_sdram),
       .clkref(cpu_clk_enable),
       .init(!clk_sdram_locked),
       .we_out(sdram_d_wr),
       // cpu/chipset interface
       .weA(spi_load ? 1'b0 : n_ram_cs == 1'b0 && n_memwr == 1'b0),
       .addrA(sdram_address),
       .oeA(cpu_clk_enable),
       .dinA(cpu_data_out),
       .doutA(ram_out),
       // SPI interface
       .weB(spi_load ? spi_ram_wr && spi_ram_addr[31:24] == 8'h00 : 1'b0),
       .addrB(spi_ram_addr[23:0]),
       .dinB(spi_ram_di),
       .oeB(0),
       .doutB()
      );
    end
  endgenerate

  // ===============================================================
  // Video
  // ===============================================================
  
  generate
    genvar i;
    if (c_vga_out) begin // Optional assignment of pins for VGA Pmod
      for(i = 0; i < 4; i = i+1) begin
        assign gp[10-i] = blue[4+i];
        assign gn[3-i] = green[4+i];
        assign gn[10-i] = red[4+i];
      end
      assign gp[2] = vsync;
      assign gp[3] = hsync;
    end
  endgenerate

  wire [3:0] col_ind;

  video vga (
    .clk(clk_vga),
    .vga_r(red),
    .vga_g(green),
    .vga_b(blue),
    .vga_de(vga_de),
    .vga_hs(hsync),
    .vga_vs(vsync),
    .vga_addr(vga_addr),
    .vga_data(vid_out),
    .mode(mode),
    .border_color(border_color),
    .pen(col_ind),
    .color(colors[col_ind]),
    .int_ack(n_iorq == 1'b0 && n_m1 == 1'b0),
    .int_clear(ga_cs && n_iowr == 1'b0 && cpu_data_out[7:6] == 2 && cpu_data_out[4]),
    .n_int(n_int)
  );

  // ===============================================================
  // OSD
  // ===============================================================
  wire [7:0] osd_vga_r, osd_vga_g, osd_vga_b;
  wire osd_vga_hsync, osd_vga_vsync, osd_vga_blank;

  spi_osd #(
    .c_start_x(62), .c_start_y(80),
    .c_chars_x(64), .c_chars_y(20),
    .c_init_on(0),
    .c_char_file("osd.mem"),
    .c_font_file("font_bizcat8x16.mem")
  ) spi_osd_inst (
    .clk_pixel(clk_vga), .clk_pixel_ena(1),
    .i_r(red),
    .i_g(green),
    .i_b(blue),
    .i_hsync(~hsync), .i_vsync(~vsync), .i_blank(~vga_de),
    .i_csn(~wifi_gpio5), .i_sclk(wifi_gpio16), .i_mosi(sd_d[1]), // .o_miso(),
    .o_r(osd_vga_r), .o_g(osd_vga_g), .o_b(osd_vga_b),
    .o_hsync(osd_vga_hsync), .o_vsync(osd_vga_vsync), .o_blank(osd_vga_blank)
  );

  // ===============================================================
  // Convert VGA to HDMI
  // ===============================================================
  HDMI_out vga2dvid (
    .pixclk(clk_vga),
    .pixclk_x5(clk_hdmi),
    .red(osd_vga_r),
    .green(osd_vga_g),
    .blue(osd_vga_b),
    .vde(~osd_vga_blank),
    .hSync(osd_vga_hsync),
    .vSync(osd_vga_vsync),
    .gpdi_dp(gpdi_dp),
    .gpdi_dn(gpdi_dn)
  );

  // ===============================================================
  // Audio
  // ===============================================================
  wire [9:0] sound_10;
  assign audio_l = sound_10[7:4];
  assign audio_r = audio_l;
  wire [7:0] ch_a, ch_b, ch_c;
  wire [7:0] audio_out;

  jt49_bus audio (
    .rst_n(~reset),
    .clk(clk_cpu),
    .clk_en(cpu_clk_enable),
    .bdir(ppi_c[7]),
    .bc1(ppi_c[6]),
    .din(ppi_a),
    .dout(audio_out),
    .A(ch_a),
    .B(ch_b),
    .C(ch_c),
    .sel(1'b0),
    .sound(sound_10),
    .IOA_in(),
    .IOA_out(),
    .IOB_in(),
    .IOB_out()
  );

  // ===============================================================
  // ACIA for serial terminal
  // ===============================================================
  wire acia_txd, acia_rxd;

  generate
    if(c_acia_serial) begin       // FTDI pins to host can be used by ACIA ...
      assign acia_rxd = ftdi_txd;
      assign ftdi_rxd = acia_txd;
    end else begin
      assign acia_rxd = 0;
      if(c_esp32_serial) begin    // ... or for passthru to ESP32
        assign wifi_rxd = ftdi_txd;
        assign ftdi_rxd = wifi_txd;
      end
    end
  endgenerate

  // 6850 ACIA (uart)
  reg baudclk; // 16 * 9600 = 153600 = 25Mhz/162
  reg [7:0] baudctr = 0;
  reg [3:0] e_counter = 0;
  reg e_clk;

  generate
    if(c_acia_serial) begin
      always @(posedge clk_cpu) begin
        baudctr <= baudctr + 1;
        baudclk <= (baudctr > 20);
        if(baudctr > 40) baudctr <= 0;
        e_counter <= e_counter + 1;
        if (e_counter == 4) e_counter <= 0;
        e_clk <= (e_counter < 2);
      end

      // 9600 8N1
      ACIA acia(
        .clk(clk_cpu),
        .reset(reset),
        .cs(tctrl_cs | tdata_cs),
        .e_clk(cpu_clk_enable),
        //.e_clk(e_clk),
        .rw_n(n_iowr),
        .rs(tdata_cs),
        //.data_in(tctrl_cs ? 8'h01 : cpu_data_out), // Set output when anything is written to tctrl
        .data_in(cpu_data_out),
        .data_out(acia_dout),
        .txclk(baudclk),
        .rxclk(baudclk),
        .txdata(acia_txd),
        .rxdata(acia_rxd),
        .cts_n(1'b0),
        .dcd_n(1'b0)
      );
    end
  endgenerate

  // ===============================================================
  // LCD diagnostics
  // ===============================================================
  generate
  if(c_lcd_hex) begin
  // SPI DISPLAY
  reg [127:0] r_display;
  // HEX decoder does printf("%16X\n%16X\n", r_display[63:0], r_display[127:64]);
  always @(posedge clk_cpu)
    //r_display <= {cpu_data_in, cpu_data_out, cpu_address, pc};
    r_display <= scr_start;

  parameter c_color_bits = 16;
  wire [7:0] x;
  wire [7:0] y;
  wire [c_color_bits-1:0] color;
  hex_decoder_v
  #(
    .c_data_len(128),
    .c_row_bits(4),
    .c_grid_6x8(1), // NOTE: TRELLIS needs -abc9 option to compile
    .c_font_file("hex_font.mem"),
    .c_color_bits(c_color_bits)
  )
  hex_decoder_v_inst
  (
    .clk(clk_hdmi),
    .data(r_display),
    .x(x[7:1]),
    .y(y[7:1]),
    .color(color)
  );

  wire next_pixel;
  reg [c_color_bits-1:0] r_color;
  wire w_oled_csn;

  always @(posedge clk_hdmi)
    if(next_pixel) r_color <= color;

  lcd_video #(
    .c_clk_mhz(125),
    .c_init_file("st7789_linit_xflip.mem"),
    .c_clk_phase(0),
    .c_clk_polarity(1),
    .c_init_size(38)
  ) lcd_video_inst (
    .clk(clk_hdmi),
    .reset(r_btn_joy[5]),
    .x(x),
    .y(y),
    .next_pixel(next_pixel),
    .color(r_color),
    .spi_clk(oled_clk),
    .spi_mosi(oled_mosi),
    .spi_dc(oled_dc),
    .spi_resn(oled_resn),
    .spi_csn(w_oled_csn)
  );

  //assign oled_csn = w_oled_csn; // 8-pin ST7789: oled_csn is connected to CSn
  assign oled_csn = 1; // 7-pin ST7789: oled_csn is connected to BLK (backlight enable pin)
  end
  endgenerate

  // ===============================================================
  // Led diagnostics
  // ===============================================================
  reg [15:0] diag16;

  generate
    genvar i;
    if (c_diag) begin
      for(i = 0; i < 4; i = i+1) begin
        assign gn[17-i] = diag16[8+i];
        assign gp[17-i] = diag16[12+i];
        assign gn[24-i] = diag16[i];
        assign gp[24-i] = diag16[4+i];
      end
    end
  endgenerate

  // ===============================================================
  // Leds
  // ===============================================================
  assign led = 0;
  
endmodule
