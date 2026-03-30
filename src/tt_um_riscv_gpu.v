`timescale 1ns/1ps
`default_nettype none

// TinyTapeout top wrapper for 4x4 uint8 single-MAC accelerator with SPI.
//
// Pin map:
//   ui_in[0]  = SPI SCLK
//   ui_in[1]  = SPI CS_N
//   ui_in[2]  = SPI MOSI
//   ui_in[7:3]  unused
//
//   uo_out[0] = SPI MISO
//   uo_out[1] = busy
//   uo_out[2] = done (sticky, cleared on next start)
//   uo_out[7:3] = 0
//
//   uio       = unused (active input)
//
// SPI command byte: {R/W[7], SEL[6:5], ROW[4:3], COL[2:1], 0}
//   SEL 00 = write matrix A element   (1 data byte)
//   SEL 01 = write matrix B element   (1 data byte)
//   SEL 10 = control / status
//            write: bit 0 = start      (1 data byte)
//            read:  {6'b0, done, busy}  (1 byte out)
//   SEL 11 = read matrix C element    (3 bytes out, MSB first)

module tt_um_riscv_gpu (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    // --- Unused bidirectional IOs ---
    assign uio_out = 8'b0;
    assign uio_oe  = 8'b0;   // all inputs

    // --- Internal reset (active-high) ---
    wire rst = ~rst_n;

    // --- SPI pin mapping ---
    wire spi_sclk = ui_in[0];
    wire spi_cs_n = ui_in[1];
    wire spi_mosi = ui_in[2];
    wire spi_miso;

    // --- SPI slave <-> register bus ---
    wire        cmd_valid;
    wire [7:0]  cmd_byte;
    wire        wr_valid;
    wire [7:0]  wr_byte;
    reg  [23:0] rd_data;

    // --- Command decode ---
    wire       cmd_is_read = cmd_byte[7];
    wire [1:0] cmd_sel     = cmd_byte[6:5];
    wire [1:0] cmd_row     = cmd_byte[4:3];
    wire [1:0] cmd_col     = cmd_byte[2:1];

    // --- TPU core signals ---
    reg         core_load_en;
    reg         core_load_sel;
    reg  [1:0]  core_load_row;
    reg  [1:0]  core_load_col;
    reg  [7:0]  core_load_data;
    reg         core_start;
    wire        core_busy;
    wire        core_done;
    wire [19:0] core_c_data;

    // --- Sticky done flag ---
    reg done_sticky;

    always @(posedge clk or posedge rst) begin
        if (rst)
            done_sticky <= 1'b0;
        else if (core_start)
            done_sticky <= 1'b0;
        else if (core_done)
            done_sticky <= 1'b1;
    end

    // --- Read data mux (combinational, feeds SPI shift-out) ---
    always @(*) begin
        case (cmd_sel)
            2'b10:   rd_data = {16'b0, 6'b0, done_sticky, core_busy};
            2'b11:   rd_data = {4'b0, core_c_data};
            default: rd_data = 24'b0;
        endcase
    end

    // --- Write handling ---
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            core_load_en   <= 1'b0;
            core_load_sel  <= 1'b0;
            core_load_row  <= 2'b0;
            core_load_col  <= 2'b0;
            core_load_data <= 8'b0;
            core_start     <= 1'b0;
        end else begin
            core_load_en <= 1'b0;
            core_start   <= 1'b0;

            if (wr_valid) begin
                case (cmd_sel)
                    2'b00: begin   // Matrix A
                        core_load_en   <= 1'b1;
                        core_load_sel  <= 1'b0;
                        core_load_row  <= cmd_row;
                        core_load_col  <= cmd_col;
                        core_load_data <= wr_byte;
                    end
                    2'b01: begin   // Matrix B
                        core_load_en   <= 1'b1;
                        core_load_sel  <= 1'b1;
                        core_load_row  <= cmd_row;
                        core_load_col  <= cmd_col;
                        core_load_data <= wr_byte;
                    end
                    2'b10: begin   // Control: start
                        if (wr_byte[0])
                            core_start <= 1'b1;
                    end
                    default: ;
                endcase
            end
        end
    end

    // --- SPI slave ---
    spi_slave u_spi (
        .clk       (clk),
        .rst       (rst),
        .spi_sclk  (spi_sclk),
        .spi_cs_n  (spi_cs_n),
        .spi_mosi  (spi_mosi),
        .spi_miso  (spi_miso),
        .cmd_valid (cmd_valid),
        .cmd_byte  (cmd_byte),
        .wr_valid  (wr_valid),
        .wr_byte   (wr_byte),
        .rd_data   (rd_data)
    );

    // --- MAC core (single 4x4 unsigned, 64-cycle matmul) ---
    mac_core #(
        .N  (4),
        .DW (8),
        .CW (20)
    ) u_core (
        .clk       (clk),
        .rst       (rst),
        .start     (core_start),
        .busy      (core_busy),
        .done      (core_done),
        .load_en   (core_load_en),
        .load_sel  (core_load_sel),
        .load_row  (core_load_row),
        .load_col  (core_load_col),
        .load_data (core_load_data),
        .c_rd_en   (1'b1),
        .c_rd_row  (cmd_row),
        .c_rd_col  (cmd_col),
        .c_rd_data (core_c_data)
    );

    // --- Output pin mapping ---
    assign uo_out[0]   = spi_miso;
    assign uo_out[1]   = core_busy;
    assign uo_out[2]   = done_sticky;
    assign uo_out[7:3] = 5'b0;

endmodule
