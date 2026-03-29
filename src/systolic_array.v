`timescale 1ns/1ps

module systolic_array #(
    parameter integer N  = 8,   // array size
    parameter integer DW = 8,   // data width
    parameter integer CW = 32   // accumulator width
)(
    input  wire                    clk,
    input  wire                    rst,
    input  wire                    clear,

    // Flattened ports to avoid unpacked-array port synthesis errors.
    input  wire [N*DW-1:0]   a_in,   // row inputs
    input  wire [N*DW-1:0]   b_in,   // column inputs

    output wire [N*N*CW-1:0] c_out
);

    // ================= INTERNAL BUSES =================
    wire [DW-1:0] a_vec [0:N-1];
    wire [DW-1:0] b_vec [0:N-1];
    wire [CW-1:0] c_mat [0:N-1][0:N-1];

    wire [DW-1:0] a_bus [0:N-1][0:N];     // flows right
    wire [DW-1:0] b_bus [0:N][0:N-1];     // flows down

    genvar i, j;

    // ================= PORT UNPACK =================
    generate
        for (i = 0; i < N; i = i + 1) begin : GEN_PORT_UNPACK
            assign a_vec[i] = a_in[(i*DW) +: DW];
            assign b_vec[i] = b_in[(i*DW) +: DW];
        end
    endgenerate

    // ================= INPUT BOUNDARY =================
    generate
        for (i = 0; i < N; i = i + 1) begin : GEN_INPUT_BOUNDARY
            assign a_bus[i][0] = a_vec[i];
            assign b_bus[0][i] = b_vec[i];
        end
    endgenerate

    // ================= PE GRID =================
    generate
        for (i = 0; i < N; i = i + 1) begin : ROW
            for (j = 0; j < N; j = j + 1) begin : COL
                pe #(
                    .DW(DW),
                    .CW(CW)
                ) PE (
                    .clk   (clk),
                    .rst   (rst),
                    .clear (clear),

                    .a_in  (a_bus[i][j]),
                    .a_out (a_bus[i][j+1]),

                    .b_in  (b_bus[i][j]),
                    .b_out (b_bus[i+1][j]),

                    .c     (c_mat[i][j])
                );
            end
        end
    endgenerate

    // ================= PORT PACK =================
    generate
        for (i = 0; i < N; i = i + 1) begin : GEN_PORT_PACK_ROW
            for (j = 0; j < N; j = j + 1) begin : GEN_PORT_PACK_COL
                assign c_out[(((i*N)+j)*CW) +: CW] = c_mat[i][j];
            end
        end
    endgenerate

endmodule
