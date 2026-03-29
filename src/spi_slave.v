`timescale 1ns/1ps

// SPI slave (mode 0: CPOL=0, CPHA=0) with oversampling.
// Protocol:
//   - CS_N low starts a transaction.
//   - First 8 SCLK cycles: command byte shifted in (MSB first).
//   - Subsequent cycles: write data shifted in / read data shifted out.
//   - CS_N high ends the transaction.
//
// Command byte format: {R/W[7], SEL[6:5], ROW[4:3], COL[2:1], 0}
//   R/W : 1 = read, 0 = write
//   SEL : 00 = matrix A, 01 = matrix B, 10 = control/status, 11 = result C
//   ROW : 2-bit row address
//   COL : 2-bit column address

module spi_slave (
    input  wire        clk,        // system clock (must be >> spi_sclk)
    input  wire        rst,

    // SPI pins
    input  wire        spi_sclk,
    input  wire        spi_cs_n,
    input  wire        spi_mosi,
    output reg         spi_miso,

    // Register interface
    output reg         cmd_valid,   // pulse: command byte received
    output reg  [7:0]  cmd_byte,    // latched command byte
    output reg         wr_valid,    // pulse: write data byte received
    output reg  [7:0]  wr_byte,     // latched write data
    input  wire [23:0] rd_data      // read response (active when cmd_byte is a read)
);

    // --- Synchronizers (2-FF for metastability) ---
    reg [2:0] sclk_sync;
    reg [2:0] cs_n_sync;
    reg [1:0] mosi_sync;

    wire sclk_rise = (sclk_sync[2:1] == 2'b01);
    wire sclk_fall = (sclk_sync[2:1] == 2'b10);
    wire cs_active = ~cs_n_sync[2];

    always @(posedge clk) begin
        if (rst) begin
            sclk_sync <= 3'b000;
            cs_n_sync <= 3'b111;
            mosi_sync <= 2'b00;
        end else begin
            sclk_sync <= {sclk_sync[1:0], spi_sclk};
            cs_n_sync <= {cs_n_sync[1:0], spi_cs_n};
            mosi_sync <= {mosi_sync[0], spi_mosi};
        end
    end

    // --- Shift registers and control ---
    reg [5:0]  bit_cnt;
    reg [7:0]  shift_in;
    reg [23:0] shift_out;
    reg        cmd_phase;       // 1 while receiving command byte
    reg        rd_loaded;       // 1 after read data has been loaded

    always @(posedge clk) begin
        if (rst) begin
            bit_cnt   <= 6'd0;
            shift_in  <= 8'd0;
            shift_out <= 24'd0;
            cmd_byte  <= 8'd0;
            cmd_valid <= 1'b0;
            wr_byte   <= 8'd0;
            wr_valid  <= 1'b0;
            cmd_phase <= 1'b1;
            rd_loaded <= 1'b0;
            spi_miso  <= 1'b0;
        end else begin
            cmd_valid <= 1'b0;
            wr_valid  <= 1'b0;

            if (!cs_active) begin
                // Transaction ended / not active
                bit_cnt   <= 6'd0;
                cmd_phase <= 1'b1;
                rd_loaded <= 1'b0;
                spi_miso  <= 1'b0;
            end else begin
                // --- SCLK rising edge: sample MOSI ---
                if (sclk_rise) begin
                    shift_in <= {shift_in[6:0], mosi_sync[1]};
                    bit_cnt  <= bit_cnt + 6'd1;

                    if (cmd_phase && bit_cnt == 6'd7) begin
                        // Command byte complete
                        cmd_byte  <= {shift_in[6:0], mosi_sync[1]};
                        cmd_valid <= 1'b1;
                        cmd_phase <= 1'b0;
                    end else if (!cmd_phase && bit_cnt[2:0] == 3'd7) begin
                        // Write data byte complete
                        wr_byte  <= {shift_in[6:0], mosi_sync[1]};
                        wr_valid <= 1'b1;
                    end
                end

                // --- Load read data one cycle after cmd_valid ---
                if (cmd_valid && cmd_byte[7]) begin
                    shift_out <= rd_data;
                    rd_loaded <= 1'b1;
                end

                // --- SCLK falling edge: drive MISO ---
                if (sclk_fall && rd_loaded) begin
                    spi_miso  <= shift_out[23];
                    shift_out <= {shift_out[22:0], 1'b0};
                end
            end
        end
    end

endmodule
