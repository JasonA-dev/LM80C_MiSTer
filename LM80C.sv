module emu
(
    //Master input clock
    input         CLK_50M,

    //Async reset from top-level module.
    input         RESET,

    //Must be passed to hps_io module
    inout  [48:0] HPS_BUS,

    //Base video clock. Usually equals to clk_sys.
    output        CLK_VIDEO,
    output        CE_PIXEL,
    output [12:0] VIDEO_ARX,
    output [12:0] VIDEO_ARY,

    output  [7:0] VGA_R,
    output  [7:0] VGA_G,
    output  [7:0] VGA_B,
    output        VGA_HS,
    output        VGA_VS,
    output        VGA_DE,
    output        VGA_F1,
    output [1:0]  VGA_SL,
    output        VGA_SCALER,
    output        VGA_DISABLE,

    input  [11:0] HDMI_WIDTH,
    input  [11:0] HDMI_HEIGHT,
    output        HDMI_FREEZE,
    output        HDMI_BLACKOUT,

`ifdef MISTER_FB
    // Framebuffer signals...
    output        FB_EN,
    output  [4:0] FB_FORMAT,
    output [11:0] FB_WIDTH,
    output [11:0] FB_HEIGHT,
    output [31:0] FB_BASE,
    output [13:0] FB_STRIDE,
    input         FB_VBL,
    input         FB_LL,
    output        FB_FORCE_BLANK,
`ifdef MISTER_FB_PALETTE
    // Palette control for 8bit modes...
    output        FB_PAL_CLK,
    output  [7:0] FB_PAL_ADDR,
    output [23:0] FB_PAL_DOUT,
    input  [23:0] FB_PAL_DIN,
    output        FB_PAL_WR,
`endif
`endif

    output        LED_USER,
    output  [1:0] LED_POWER,
    output  [1:0] LED_DISK,
    output  [1:0] BUTTONS,

    input         CLK_AUDIO,
    output [15:0] AUDIO_L,
    output [15:0] AUDIO_R,
    output        AUDIO_S,
    output  [1:0] AUDIO_MIX,

    inout   [3:0] ADC_BUS,

    //SD-SPI
    output        SD_SCK,
    output        SD_MOSI,
    input         SD_MISO,
    output        SD_CS,
    input         SD_CD,

    //High latency DDR3 RAM interface...
    output        DDRAM_CLK,
    input         DDRAM_BUSY,
    output  [7:0] DDRAM_BURSTCNT,
    output [28:0] DDRAM_ADDR,
    input  [63:0] DDRAM_DOUT,
    input         DDRAM_DOUT_READY,
    output        DDRAM_RD,
    output [63:0] DDRAM_DIN,
    output  [7:0] DDRAM_BE,
    output        DDRAM_WE,

    //SDRAM interface
    output        SDRAM_CLK,
    output        SDRAM_CKE,
    output [12:0] SDRAM_A,
    output  [1:0] SDRAM_BA,
    inout  [15:0] SDRAM_DQ,
    output        SDRAM_DQML,
    output        SDRAM_DQMH,
    output        SDRAM_nCS,
    output        SDRAM_nCAS,
    output        SDRAM_nRAS,
    output        SDRAM_nWE,

`ifdef MISTER_DUAL_SDRAM
    //Secondary SDRAM
    input         SDRAM2_EN,
    output        SDRAM2_CLK,
    output [12:0] SDRAM2_A,
    output  [1:0] SDRAM2_BA,
    inout  [15:0] SDRAM2_DQ,
    output        SDRAM2_nCS,
    output        SDRAM2_nCAS,
    output        SDRAM2_nRAS,
    output        SDRAM2_nWE,
`endif

    input         UART_CTS,
    output        UART_RTS,
    input         UART_RXD,
    output        UART_TXD,
    output        UART_DTR,
    input         UART_DSR,

    // Open-drain User port.
    input   [6:0] USER_IN,
    output  [6:0] USER_OUT,

    input         OSD_STATUS
);

////////////////////////////////////////////////////////////////////////

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CLK, SDRAM_CKE, SDRAM_DQML,
        SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;
assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE,
        DDRAM_RD, DDRAM_WE} = '0;

assign VGA_F1       = 0;
assign VGA_SCALER   = 0;
assign VGA_DISABLE  = 0;
assign HDMI_FREEZE  = 0;
assign HDMI_BLACKOUT= 0;

assign AUDIO_S   = 0;
assign AUDIO_L   = {1'd0,CHANNEL_L,7'd0};
assign AUDIO_R   = {1'd0,CHANNEL_R,7'd0};
assign AUDIO_MIX = 0;

assign LED_DISK  = 0;
assign LED_POWER = 0;
assign BUTTONS   = 0;

////////////////////////////////////////////////////////////////////////
`include "build_id.v"
localparam CONF_STR = {
	"LM80C;;",
   "F1,BAS, Load BAS File;",
	"F2,PRG, Load PRG File;", 
	"F3,BIN, Load BIOS ROM;",     
	"-;",
	"O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"O[2],TV Mode,NTSC,PAL;",
	"O[4:3],Noise,White,Red,Green,Blue;",
	"-;",
	"T[0],Reset;",
	"R[0],Reset and close OSD;",
	"v,0;", // [optional] config version 0-99. 
	        // If CONF_STR options are changed in incompatible way, then change version number too,
			  // so all options will get default values on first start.
	"V,v",`BUILD_DATE 
};

////////////////////////////////////////////////////////////////////////
// HPS I/O and OSD
////////////////////////////////////////////////////////////////////////

wire         forced_scandoubler;
wire  [21:0] gamma_bus;

wire   [1:0] buttons_internal;
wire [127:0] status;
wire  [10:0] ps2_key;

wire         ioctl_download;
wire   [7:0] ioctl_index;
wire         ioctl_wr;
wire  [24:0] ioctl_addr;
wire   [7:0] ioctl_dout;

hps_io #(.CONF_STR(CONF_STR)) hps_io_inst
(
    .clk_sys(clk_sys),
    .HPS_BUS(HPS_BUS),
    .EXT_BUS(),

    .forced_scandoubler(forced_scandoubler),
    .gamma_bus(gamma_bus),

    .buttons(buttons_internal),
    .status(status),
    .status_menumask({status[5]}),

    .ps2_key(ps2_key),

    .ioctl_download(ioctl_download),
    .ioctl_addr(ioctl_addr),
    .ioctl_dout(ioctl_dout),
    .ioctl_wr(ioctl_wr),
    .ioctl_index(ioctl_index)
);

////////////////////////////////////////////////////////////////////////
// PLL
////////////////////////////////////////////////////////////////////////

wire clk_sys, clk_cpu, clk_vdp;
wire pll_locked;
reg [3:0] div_sys;

pll pll_inst
(
    .refclk (CLK_50M),
    .rst    (1'b0),
    .outclk_0 (clk_sys), // 42.954545 MHz     // 29.4912
    //.outclk_1 (clk_cpu), //  3.579545 MHz
    .outclk_1 (clk_vdp), // 10.738636 MHz     // 42.954545
    .locked   (pll_locked)
);

assign CLK_VIDEO = clk_sys;
assign CE_PIXEL  = ce_pix;  

// ce_pix

// previous code for 5.3
/*
reg ce_5m3 = 0;
always @(posedge clk_sys) begin
    reg [2:0] div;
    div <= div + 1'd1;
    ce_5m3  <= !div[2:0];  // Generates 5.3 MHz enable signal (CE_PIXEL)
end
*/
reg       ce_div = 1'b0;  // toggles each clock
reg       ce_pix = 1'b0;  // one cycle wide pulse every other clock
always @(posedge clk_vdp) begin
    ce_div    <= ~ce_div;
    // If you want a 1-cycle-wide strobe at half-rate:
    ce_pix <= (ce_div == 1'b0);
end


// vdp

reg [1:0] cnt_vdp;
always @(posedge clk_vdp)
	cnt_vdp <= cnt_vdp + 1;
	
wire vdp_ena = cnt_vdp == 0;

// cpu

wire cpu_clock = clk_div[2];
reg [3:0] clk_div;
always @(posedge clk_sys)
	clk_div <= clk_div + 3'd1;

wire z80_ena = clk_div == 0 || clk_div == 8;
wire psg_ena = clk_div == 0;   

////////////////////////////////////////////////////////////////////////
// Reset Logic
////////////////////////////////////////////////////////////////////////

wire  ROM_loaded;
wire  eraser_busy;
wire  reset_key;
wire  is_downloading;
wire [1:0] buttons = buttons_internal;
wire [127:0] status_local = status;

wire reset = ~ROM_loaded | RESET | reset_key | eraser_busy | buttons[1] | status[0];

////////////////////////////////////////////////////////////////////////
// CPU WAIT and LED
////////////////////////////////////////////////////////////////////////

wire is_wait = ~ROM_loaded | is_downloading;

assign LED_USER = ~is_wait;  

////////////////////////////////////////////////////////////////////////

wire [5:0] rgb_r;
wire [5:0] rgb_g;
wire [5:0] rgb_b;
wire       HSync;
wire       VSync;
wire       VBlank;
wire       HBlank;

wire       is_pal;
assign is_pal = status_local[2];

wire [1:0] ar = status_local[122:121];
assign VIDEO_ARX = (!ar) ? 12'd4 : (ar - 1'd1);
assign VIDEO_ARY = (!ar) ? 12'd3 : 12'd0;

assign VGA_R = {rgb_r, 2'b00};
assign VGA_G = {rgb_g, 2'b00};
assign VGA_B = {rgb_b, 2'b00};
assign VGA_HS = HSync;
assign VGA_VS = VSync;
assign VGA_DE = ~(VBlank | HBlank);

wire [7:0] CHANNEL_L;
wire [7:0] CHANNEL_R;

lm80c lm80c_inst
(
    .RESET      (reset),
    .WAIT       (is_wait),

    .sys_clock  (clk_sys),
    .cpu_clock  (clk_cpu),
	.vdp_clock  (clk_vdp),

    .z80_ena    (z80_ena),
    .vdp_ena    (vdp_ena),
    .psg_ena    (psg_ena),

    // Video
    .R          (rgb_r),
    .G          (rgb_g),
    .B          (rgb_b),
    .HS         (HSync),
    .VS         (VSync),
    .VBlank     (VBlank),
    .HBlank     (HBlank),

    .is_pal     (is_pal),

    // Audio
    .CHANNEL_L  (CHANNEL_L),
    .CHANNEL_R  (CHANNEL_R),

    // Keyboard
    .KM         (KM),

    // RAM interface
    .ram_addr   (cpu_addr),
    .ram_din    (cpu_dout),
    .ram_dout   (sdram_dout),
    .ram_rd     (cpu_rd),
    .ram_wr     (cpu_wr),

    // Parallel port
    .PIO_data_A (PIO_data_A),
    .PIO_data_B (PIO_data_B)
);

////////////////////////////////////////////////////////////////////////
// PS2 Keyboard adapter
////////////////////////////////////////////////////////////////////////

wire [7:0] KM[7:0];

///////////////////////////////////////////////////////////////////////////////
// 1. Generate a valid strobe whenever ps2_key[10] toggles
///////////////////////////////////////////////////////////////////////////////
reg old_state;
reg key_strobe;

always @(posedge clk_sys or posedge reset) begin
    if (reset) begin
        old_state   <= 1'b0;
        key_strobe  <= 1'b0;
    end else begin
        // Watch bit [10] for toggles
        old_state <= ps2_key[10];
        if (old_state != ps2_key[10]) begin
            // Toggle key_strobe every time ps2_key[10] changes
            key_strobe <= ~key_strobe;
        end
    end
end

///////////////////////////////////////////////////////////////////////////////
// 2. Build the 16-bit "key" and 1-bit "key_status" for the adapter
///////////////////////////////////////////////////////////////////////////////
wire [15:0] adapter_key;
wire        adapter_key_status;

// If extended bit [8] = 1, put 0xE0 in the top byte, else 0x00.
// Lower byte is the scancode [7:0].
assign adapter_key        = ps2_key[8] 
                            ? {8'hE0, ps2_key[7:0]} 
                            : {8'h00, ps2_key[7:0]};

// key_status is simply the pressed bit [9].
assign adapter_key_status = ps2_key[9];

lm80c_ps2keyboard_adapter kbd_adapter (
    .clk(clk_sys),
    .reset(reset),
    .valid(key_strobe),
    .key(adapter_key),
    .key_status(adapter_key_status),
    .KM(KM),
    .resetkey(reset_key)
);

////////////////////////////////////////////////////////////////////////
// Downloader
////////////////////////////////////////////////////////////////////////

wire [24:0] download_addr;
wire [7:0]  download_data;
wire        download_wr;

downloader #(
    .BOOT_INDEX (0),
    .PRG_INDEX  (2),
    .ROM_INDEX  (3),
    .ROM_START_ADDR (25'h00000),
    .PRG_START_ADDR (25'h1560f),
    .PTR_PROGND     (25'h155eb)
)
downloader_inst
(
    .ioctl_download(ioctl_download),
    .ioctl_index   (ioctl_index),
    .ioctl_addr    (ioctl_addr),
    .ioctl_dout    (ioctl_dout),
    .ioctl_wr      (ioctl_wr),

    .downloading   (is_downloading),
    .ROM_done      (ROM_loaded),

    .clk           (clk_sys),
    .clk_ena       (1'b1),
    .wr            (download_wr),
    .addr          (download_addr),
    .data          (download_data)
);

////////////////////////////////////////////////////////////////////////
// Eraser
////////////////////////////////////////////////////////////////////////

wire eraser_wr;
wire [24:0] eraser_addr;
wire [7:0]  eraser_data;
wire st_menu_reset; 

eraser eraser_inst
(
    .clk      (clk_sys),
    .ena      (z80_ena),
    .trigger  (st_menu_reset),
    .erasing  (eraser_busy),
    .wr       (eraser_wr),
    .addr     (eraser_addr),
    .data     (eraser_data)
);

////////////////////////////////////////////////////////////////////////
// Split ROM & RAM in On-Chip DPRAM
////////////////////////////////////////////////////////////////////////

wire [7:0] sdram_dout;   // CPU read data
wire [7:0] rom_dout;
wire [7:0] ram_dout;

wire [15:0] cpu_addr;
wire [7:0]  cpu_dout;
wire        cpu_rd;
wire        cpu_wr;

wire [7:0]  PIO_data_A;
wire [7:0]  PIO_data_B;
wire        ROM_ENABLED = PIO_data_B[0];

// The CPU sees either ROM or RAM depending on ROM_ENABLED
wire rom_cpu_sel = ROM_ENABLED && (cpu_addr < 16'h8000);
wire ram_cpu_sel = ~rom_cpu_sel;

assign sdram_dout = rom_cpu_sel ? rom_dout : ram_dout;

// Write logic for ROM/RAM
reg        rom_wr;
reg [14:0] rom_wr_addr;
reg  [7:0] rom_wr_data;

reg        ram_wr;
reg [15:0] ram_wr_addr;
reg  [7:0] ram_wr_data;

always @(*) begin
    rom_wr       = 1'b0;
    ram_wr       = 1'b0;
    rom_wr_addr  = 15'h0000;
    ram_wr_addr  = 16'h0000;
    rom_wr_data  = 8'h00;
    ram_wr_data  = 8'h00;

    // Downloader writes
    if(is_downloading && download_wr) begin
        if(download_addr < 25'h08000) begin
            rom_wr      = 1'b1;
            rom_wr_addr = download_addr[14:0];
            rom_wr_data = download_data;
        end
        else if((download_addr >= 25'h10000) && (download_addr < 25'h20000)) begin
            ram_wr      = 1'b1;
            ram_wr_addr = download_addr[15:0];
            ram_wr_data = download_data;
        end
    end
    // Eraser writes
    else if(eraser_busy && eraser_wr) begin
        if(eraser_addr < 25'h08000) begin
            rom_wr      = 1'b1;
            rom_wr_addr = eraser_addr[14:0];
            rom_wr_data = eraser_data;
        end
        else if((eraser_addr >= 25'h10000) && (eraser_addr < 25'h20000)) begin
            ram_wr      = 1'b1;
            ram_wr_addr = eraser_addr[15:0];
            ram_wr_data = eraser_data;
        end
    end
    else begin
        // CPU writes
        // If ROM_ENABLED=0, CPU can write entire 64k
        if(cpu_wr && (ram_cpu_sel || ~ROM_ENABLED)) begin
            ram_wr       = 1'b1;
            ram_wr_addr  = cpu_addr;
            ram_wr_data  = cpu_dout;
        end
    end
end

// CPU read addresses
wire [14:0] rom_cpu_addr = cpu_addr[14:0];
wire [15:0] ram_cpu_addr = cpu_addr;

// 32 KB ROM
dpram #(8,15) rom_mem
(
    .clock_a   (clk_sys),
    .address_a (rom_wr_addr),
    .wren_a    (rom_wr),
    .data_a    (rom_wr_data),
    .q_a       (), 

    .clock_b   (clk_sys),
    .address_b (rom_cpu_addr),
    .wren_b    (1'b0),
    .data_b    (8'h00),
    .q_b       (rom_dout)
);

// 64 KB RAM
dpram #(8,16) ram_mem
(
    .clock_a   (clk_sys),
    .address_a (ram_wr_addr),
    .wren_a    (ram_wr),
    .data_a    (ram_wr_data),
    .q_a       (),

    .clock_b   (clk_sys),
    .address_b (ram_cpu_addr),
    .wren_b    (1'b0),
    .data_b    (8'h00),
    .q_b       (ram_dout)
);

endmodule
