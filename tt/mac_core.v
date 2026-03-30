`timescale 1ns/1ps

// Single-MAC matrix multiply core for 4x4 uint8.
// Computes C[i][j] = sum_k(A[i][k] * B[k][j]) by iterating
// through all (i, j, k) in 64 clock cycles.

module mac_core #(
    parameter N  = 4,
    parameter DW = 8,
    parameter CW = 20
) (
    input  wire                   clk,
    input  wire                   rst,
    input  wire                   start,
    output reg                    busy,
    output reg                    done,

    // Scratchpad load interface
    input  wire                   load_en,
    input  wire                   load_sel,  // 0: A, 1: B
    input  wire [$clog2(N)-1:0]   load_row,
    input  wire [$clog2(N)-1:0]   load_col,
    input  wire [DW-1:0]          load_data,

    // Result read interface
    input  wire                   c_rd_en,
    input  wire [$clog2(N)-1:0]   c_rd_row,
    input  wire [$clog2(N)-1:0]   c_rd_col,
    output reg  [CW-1:0]          c_rd_data
);

    localparam ROW_W = $clog2(N);

    localparam [1:0] ST_IDLE    = 2'd0;
    localparam [1:0] ST_COMPUTE = 2'd1;
    localparam [1:0] ST_DONE    = 2'd2;

    reg [1:0] state;

    // Scratchpads
    reg [DW-1:0] a_spm [0:N-1][0:N-1];
    reg [DW-1:0] b_spm [0:N-1][0:N-1];
    reg [CW-1:0] c_spm [0:N-1][0:N-1];

    // Loop counters
    reg [ROW_W-1:0] ci, cj, ck;

    // MAC datapath
    wire [2*DW-1:0] prod = a_spm[ci][ck] * b_spm[ck][cj];
    reg  [CW-1:0]   acc;

    integer i, j;

    // Combinational read
    always @(*) begin
        c_rd_data = {CW{1'b0}};
        if (c_rd_en)
            c_rd_data = c_spm[c_rd_row][c_rd_col];
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= ST_IDLE;
            busy  <= 1'b0;
            done  <= 1'b0;
            ci    <= {ROW_W{1'b0}};
            cj    <= {ROW_W{1'b0}};
            ck    <= {ROW_W{1'b0}};
            acc   <= {CW{1'b0}};

            for (i = 0; i < N; i = i + 1)
                for (j = 0; j < N; j = j + 1) begin
                    a_spm[i][j] <= {DW{1'b0}};
                    b_spm[i][j] <= {DW{1'b0}};
                    c_spm[i][j] <= {CW{1'b0}};
                end
        end else begin
            done <= 1'b0;

            // Load scratchpads when idle
            if (load_en && !busy) begin
                if (load_sel)
                    b_spm[load_row][load_col] <= load_data;
                else
                    a_spm[load_row][load_col] <= load_data;
            end

            case (state)
                ST_IDLE: begin
                    if (start) begin
                        busy <= 1'b1;
                        ci   <= {ROW_W{1'b0}};
                        cj   <= {ROW_W{1'b0}};
                        ck   <= {ROW_W{1'b0}};
                        acc  <= {CW{1'b0}};
                        state <= ST_COMPUTE;
                    end
                end

                ST_COMPUTE: begin
                    acc <= acc + {{(CW-2*DW){1'b0}}, prod};

                    if (ck == N[ROW_W-1:0] - 1'b1) begin
                        // Inner loop done — store result
                        c_spm[ci][cj] <= acc + {{(CW-2*DW){1'b0}}, prod};
                        acc <= {CW{1'b0}};
                        ck  <= {ROW_W{1'b0}};

                        if (cj == N[ROW_W-1:0] - 1'b1) begin
                            cj <= {ROW_W{1'b0}};

                            if (ci == N[ROW_W-1:0] - 1'b1) begin
                                // All done
                                busy  <= 1'b0;
                                done  <= 1'b1;
                                state <= ST_DONE;
                            end else begin
                                ci <= ci + 1'b1;
                            end
                        end else begin
                            cj <= cj + 1'b1;
                        end
                    end else begin
                        ck <= ck + 1'b1;
                    end
                end

                ST_DONE: begin
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
