`default_nettype none

module cordic #(
    parameter WIDTH = 16,
    parameter STAGES = 16
)(
    input  wire             clk,
    input  wire             rst_n,
    input  wire             start,
    input  wire signed [WIDTH-1:0] x_in,
    input  wire signed [WIDTH-1:0] y_in,
    output reg  signed [WIDTH-1:0] angle_out, // -Pi to Pi mapped to min/max
    output reg  signed [WIDTH-1:0] mag_out,   // Scaled magnitude
    output reg              done,
    output reg              busy
);

    reg signed [WIDTH-1:0] x, y, z;
    reg [4:0] iter;
    reg state;

    localparam IDLE = 0;
    localparam RUN  = 1;

    // Atan Lookup
    function signed [WIDTH-1:0] get_atan(input [3:0] i);
        case(i)
            0: get_atan = 16'd8192; // 45.000 deg
            1: get_atan = 16'd4836; // 26.565 deg
            2: get_atan = 16'd2555; // 14.036 deg
            3: get_atan = 16'd1297; // 7.125 deg
            4: get_atan = 16'd651;  // 3.576 deg
            5: get_atan = 16'd326;  // 1.790 deg
            6: get_atan = 16'd163;  // 0.895 deg
            7: get_atan = 16'd81;   // 0.448 deg
            8: get_atan = 16'd41;   // 0.224 deg
            9: get_atan = 16'd20;   // 0.112 deg
            10: get_atan = 16'd10;   // 0.056 deg
            11: get_atan = 16'd5;    // 0.028 deg
            12: get_atan = 16'd3;    // 0.014 deg
            13: get_atan = 16'd1;    // 0.007 deg
            14: get_atan = 16'd1;    // 0.003 deg
            15: get_atan = 16'd0;    // 0.001 deg
            default: get_atan = 16'd0;
        endcase
    endfunction

    wire signed [WIDTH-1:0] x_shift = x >>> iter;
    wire signed [WIDTH-1:0] y_shift = y >>> iter;
    wire signed [WIDTH-1:0] current_atan = get_atan(iter[3:0]);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            done      <= 0;
            busy      <= 0;
            angle_out <= 0;
            mag_out   <= 0;
            x         <= 0;
            y         <= 0;
            z         <= 0;
            iter      <= 0;
        end else begin
            done <= 0;

            case (state)
                IDLE: begin
                    if (start) begin
                        state <= RUN;
                        busy  <= 1;
                        iter  <= 0;

                        // Pre-rotation for full quadrant support
                        if (x_in < 0) begin
                            x <= -x_in;
                            y <= -y_in;
                            if (y_in >= 0)
                                z <= 16'sd32767; // Approx Pi
                            else
                                z <= -16'sd32767; // Approx -Pi
                                // Actually -32768 is fine too,
                                // but 32767 avoids overflow issues with symmetric checks
                        end else begin
                            x <= x_in;
                            y <= y_in;
                            z <= 0;
                        end
                    end else begin
                        busy <= 0;
                    end
                end

                RUN: begin
                    if (iter == STAGES) begin
                        state     <= IDLE;
                        done      <= 1;
                        busy      <= 0;
                        angle_out <= z;
                        mag_out   <= x;
                    end else begin
                        iter <= iter + 1;

                        if (y >= 0) begin
                            // Rotate CW (-angle)
                            x <= x + y_shift;
                            y <= y - x_shift;
                            z <= z + current_atan;
                        end else begin
                            // Rotate CCW (+angle)
                            x <= x - y_shift;
                            y <= y + x_shift;
                            z <= z - current_atan;
                        end
                    end
                end
            endcase
        end
    end

endmodule
