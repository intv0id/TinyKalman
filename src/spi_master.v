`default_nettype none

module spi_master #(
    parameter CLK_DIV = 5  // System Clock / (2 * SPI Clock)
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start,
    input  wire [7:0] data_in,
    output reg  [7:0] data_out,
    output reg        busy,
    output reg        done,

    // SPI Interface
    output reg        sclk,
    output reg        mosi,
    input  wire       miso
);

    // SPI Mode 3: CPOL=1, CPHA=1
    // Idle SCLK: High
    // Sample: Rising Edge (Trailing)
    // Shift: Falling Edge (Leading)

    localparam IDLE      = 0;
    localparam WAIT_FALL = 1; // Wait for falling edge to shift
    localparam WAIT_RISE = 2; // Wait for rising edge to sample

    reg [1:0] state;
    reg [2:0] bit_cnt;
    reg [7:0] clk_cnt;
    reg [7:0] shift_reg;

    // 2-stage synchronizer for MISO
    reg miso_sync_0;
    reg miso_sync_1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            miso_sync_0 <= 1'b0;
            miso_sync_1 <= 1'b0;
        end else begin
            miso_sync_0 <= miso;
            miso_sync_1 <= miso_sync_0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            sclk      <= 1'b1; // CPOL=1
            mosi      <= 1'b0;
            busy      <= 1'b0;
            done      <= 1'b0;
            bit_cnt   <= 0;
            clk_cnt   <= 0;
            data_out  <= 0;
            shift_reg <= 0;
        end else begin
            done <= 1'b0;

            case (state)
                IDLE: begin
                    sclk <= 1'b1;
                    if (start) begin
                        busy      <= 1'b1;
                        shift_reg <= data_in;
                        bit_cnt   <= 0;
                        clk_cnt   <= 0;
                        state     <= WAIT_FALL;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                WAIT_FALL: begin // Waiting to drive SCLK Low (Leading Edge)
                    if (clk_cnt == CLK_DIV - 1) begin
                        sclk    <= 1'b0; // Drive Low (Leading Edge)
                        mosi    <= shift_reg[7]; // Shift out MSB (CPHA=1: Shift on leading)
                        clk_cnt <= 0;
                        state   <= WAIT_RISE;
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                WAIT_RISE: begin // Waiting to drive SCLK High (Trailing Edge)
                    if (clk_cnt == CLK_DIV - 1) begin
                        sclk      <= 1'b1; // Drive High (Trailing Edge)
                        shift_reg <= {shift_reg[6:0], miso_sync_1}; // Sample MISO (CPHA=1: Sample on trailing)
                        clk_cnt   <= 0;

                        if (bit_cnt == 7) begin
                            state    <= IDLE;
                            done     <= 1'b1;
                            busy     <= 1'b0;
                            data_out <= {shift_reg[6:0], miso_sync_1};
                        end else begin
                            bit_cnt <= bit_cnt + 1;
                            state   <= WAIT_FALL;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end
            endcase
        end
    end

endmodule
