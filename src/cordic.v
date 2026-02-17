`default_nettype none

module cordic #(
    parameter WIDTH = 16,
    parameter STAGES = 16
)(
    input  wire             clk,
    input  wire             rst_n,
    input  wire             start,
    // Note: We use 16-bit inputs but if WIDTH < 16, we truncate internally
    input  wire signed [15:0] x_in,
    input  wire signed [15:0] y_in,
    output reg  signed [15:0] angle_out, // -Pi to Pi mapped to min/max
    output reg  signed [15:0] mag_out,   // Scaled magnitude
    output reg              done,
    output reg              busy
);

    reg signed [WIDTH-1:0] x, y, z;
    reg [4:0] iter;
    reg state;

    localparam IDLE = 0;
    localparam RUN  = 1;

    // Atan Lookup scaled for WIDTH
    // 180 degrees = 2^(WIDTH-1)
    function signed [WIDTH-1:0] get_atan(input [3:0] i);
        // Base values for 16-bit
        case(i)
            0: get_atan = 8192 >>> (16-WIDTH); // 45.000 deg
            1: get_atan = 4836 >>> (16-WIDTH); // 26.565 deg
            2: get_atan = 2555 >>> (16-WIDTH); // 14.036 deg
            3: get_atan = 1297 >>> (16-WIDTH); // 7.125 deg
            4: get_atan = 651 >>> (16-WIDTH);  // 3.576 deg
            5: get_atan = 326 >>> (16-WIDTH);  // 1.790 deg
            6: get_atan = 163 >>> (16-WIDTH);  // 0.895 deg
            7: get_atan = 81 >>> (16-WIDTH);   // 0.448 deg
            8: get_atan = 41 >>> (16-WIDTH);   // 0.224 deg
            9: get_atan = 20 >>> (16-WIDTH);   // 0.112 deg
            10: get_atan = 10 >>> (16-WIDTH);  // 0.056 deg
            11: get_atan = 5 >>> (16-WIDTH);   // 0.028 deg
            12: get_atan = 3 >>> (16-WIDTH);   // 0.014 deg
            13: get_atan = 1 >>> (16-WIDTH);   // 0.007 deg
            14: get_atan = 1 >>> (16-WIDTH);   // 0.003 deg
            15: get_atan = 0;                 // 0.001 deg
            default: get_atan = 0;
        endcase
    endfunction

    wire signed [WIDTH-1:0] x_shift = x >>> iter;
    wire signed [WIDTH-1:0] y_shift = y >>> iter;
    wire signed [WIDTH-1:0] current_atan = get_atan(iter[3:0]);

    // Pi constant for pre-rotation
    // WIDTH=12 -> 2047
    localparam signed [WIDTH-1:0] PI_CONST = {1'b0, {(WIDTH-1){1'b1}}};

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
                        // Truncate inputs to WIDTH
                        if (x_in[15] == 1) begin // x < 0
                            // Handle potential overflow on negation of max negative
                            // Just take the negation and shift
                            // But Verilog arithmetic width?
                            // Let's manually construct x_trunc
                            // x_in >>> (16-WIDTH)

                            // Safe approach:
                            // neg_x_in = -x_in;
                            // x = neg_x_in >>> (16-WIDTH);

                            x <= (-x_in) >>> (16-WIDTH);
                            y <= (-y_in) >>> (16-WIDTH);

                            if (y_in[15] == 0) // y >= 0
                                z <= PI_CONST;
                            else
                                z <= -PI_CONST;
                        end else begin
                            x <= (x_in >>> (16-WIDTH));
                            y <= (y_in >>> (16-WIDTH));
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
                        // Scale back up to 16-bit for output consistency
                        angle_out <= {z, {(16-WIDTH){1'b0}}};
                        mag_out   <= {x, {(16-WIDTH){1'b0}}};
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
