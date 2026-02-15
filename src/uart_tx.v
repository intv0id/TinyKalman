`default_nettype none

module uart_tx #(
    parameter BAUD_DIV = 104 // 10MHz / 9600 = 1041.6 -> 1042
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start,
    input  wire [7:0] data_in,
    output reg        tx,
    output reg        busy,
    output reg        done
);

    localparam IDLE  = 0;
    localparam START = 1;
    localparam DATA  = 2;
    localparam STOP  = 3;

    reg [1:0] state;
    reg [15:0] clk_cnt; // 16-bit to be safe for low baud rates
    reg [2:0] bit_cnt;
    reg [7:0] shift_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            tx        <= 1'b1; // Idle High
            busy      <= 1'b0;
            done      <= 1'b0;
            clk_cnt   <= 0;
            bit_cnt   <= 0;
            shift_reg <= 0;
        end else begin
            done <= 1'b0;

            case (state)
                IDLE: begin
                    tx <= 1'b1;
                    if (start) begin
                        state     <= START;
                        busy      <= 1'b1;
                        shift_reg <= data_in;
                        clk_cnt   <= 0;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                START: begin
                    tx <= 1'b0; // Start Bit
                    if (clk_cnt == BAUD_DIV - 1) begin
                        state   <= DATA;
                        clk_cnt <= 0;
                        bit_cnt <= 0;
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                DATA: begin
                    tx <= shift_reg[0]; // LSB First
                    if (clk_cnt == BAUD_DIV - 1) begin
                        clk_cnt <= 0;
                        shift_reg <= {1'b0, shift_reg[7:1]};
                        if (bit_cnt == 7) begin
                            state <= STOP;
                        end else begin
                            bit_cnt <= bit_cnt + 1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                STOP: begin
                    tx <= 1'b1; // Stop Bit
                    if (clk_cnt == BAUD_DIV - 1) begin
                        state   <= IDLE;
                        clk_cnt <= 0;
                        busy    <= 1'b0;
                        done    <= 1'b1;
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end
            endcase
        end
    end

endmodule
