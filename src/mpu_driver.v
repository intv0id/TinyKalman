`default_nettype none

module mpu_driver #(
    parameter CLK_DIV = 5,
    parameter SAMPLE_RATE_HZ = 100, // 100Hz sample rate
    parameter SYS_CLK_FREQ = 10000000,
    // Timer for sample rate
`ifdef FAST_SIM
    parameter TIMER_LIMIT = 20,
    parameter INIT_WAIT_CYCLES = 10
`else
    parameter TIMER_LIMIT = SYS_CLK_FREQ / SAMPLE_RATE_HZ,
    // Timer for initialization wait (e.g., 10ms)
    parameter INIT_WAIT_CYCLES = SYS_CLK_FREQ / 100
`endif
)(
    input  wire        clk,
    input  wire        rst_n,

    // SPI Interface
    input  wire        spi_miso, // from MPU
    output wire        spi_mosi, // to MPU
    output wire        spi_sclk,
    output reg         spi_cs_n,

    // Sensor Data Output
    output reg  signed [15:0] accel_x,
    output reg  signed [15:0] accel_y,
    output reg  signed [15:0] accel_z,
    output reg  signed [15:0] gyro_x,
    output reg  signed [15:0] gyro_y,
    output reg  signed [15:0] gyro_z,
    output reg         valid
);

    // MPU-6500 Registers
    localparam MPU_PWR_MGMT_1   = 8'h6B;
    localparam MPU_ACCEL_XOUT_H = 8'h3B;

    // States
    localparam S_INIT          = 0;
    localparam S_WAKE_1        = 1;
    localparam S_WAKE_1_WAIT   = 2;
    localparam S_WAKE_2        = 3;
    localparam S_WAKE_2_WAIT   = 4;
    localparam S_IDLE          = 5;
    localparam S_READ_CMD      = 6;
    localparam S_READ_CMD_WAIT = 7;
    localparam S_READ_BYTES    = 8;
    localparam S_READ_BYTES_WAIT = 9;
    localparam S_UPDATE        = 10;

    reg [3:0]  state;
    reg [3:0]  byte_cnt;
    reg [31:0] timer;
    reg        spi_start;
    reg [7:0]  spi_data_in;
    wire [7:0] spi_data_out;
    wire       spi_busy;
    wire       spi_done;

    // SPI Master Instance
    spi_master #(
        .CLK_DIV(CLK_DIV)
    ) spi_inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(spi_start),
        .data_in(spi_data_in),
        .miso(spi_miso),
        .mosi(spi_mosi),
        .sclk(spi_sclk),
        .busy(spi_busy),
        .done(spi_done),
        .data_out(spi_data_out)
    );

    // Optimized: Only store 14 bytes if necessary, but we can decode on the fly
    // Actually we need 12 bytes (Accel X,Y,Z, Gyro X,Y,Z). Temp is skipped.
    // However, to save registers, we can shift into output registers directly?
    // Not easily because output registers are 16-bit and we get 8-bit chunks.
    // BUT we can remove `data_buffer` and write directly to `accel_x` etc.

    // Let's remove data_buffer and use a case statement on byte_cnt to direct data.

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_INIT;
            timer       <= 0;
            spi_cs_n    <= 1;
            spi_start   <= 0;
            byte_cnt    <= 0;
            valid       <= 0;
            accel_x     <= 0;
            accel_y     <= 0;
            accel_z     <= 0;
            gyro_x      <= 0;
            gyro_y      <= 0;
            gyro_z      <= 0;
            spi_data_in <= 0;
        end else begin
            // Default signals
            valid <= 0;
            spi_start <= 0;

            case (state)
                S_INIT: begin
                    // Wait a bit after reset
                    if (timer >= INIT_WAIT_CYCLES) begin
                        state <= S_WAKE_1;
                        timer <= 0;
                    end else begin
                        timer <= timer + 1;
                    end
                end

                S_WAKE_1: begin
                    // Assert CS, Send PWR_MGMT_1 Address (Write)
                    spi_cs_n    <= 0;
                    spi_data_in <= MPU_PWR_MGMT_1; // Write (MSB 0)
                    spi_start   <= 1;
                    state       <= S_WAKE_1_WAIT;
                end

                S_WAKE_1_WAIT: begin
                    if (spi_done) begin
                        state <= S_WAKE_2;
                    end
                end

                S_WAKE_2: begin
                    // Send 0x00 to wake up
                    spi_data_in <= 8'h00;
                    spi_start   <= 1;
                    state       <= S_WAKE_2_WAIT;
                end

                S_WAKE_2_WAIT: begin
                    if (spi_done) begin
                        spi_cs_n <= 1; // Deassert CS
                        state    <= S_IDLE;
                    end
                end

                S_IDLE: begin
                    if (timer >= TIMER_LIMIT) begin
                        state <= S_READ_CMD;
                        timer <= 0;
                    end else begin
                        timer <= timer + 1;
                    end
                end

                S_READ_CMD: begin
                    // Assert CS, Send ACCEL_XOUT_H Address (Read)
                    spi_cs_n    <= 0;
                    spi_data_in <= MPU_ACCEL_XOUT_H | 8'h80; // Read bit (MSB 1)
                    spi_start   <= 1;
                    state       <= S_READ_CMD_WAIT;
                end

                S_READ_CMD_WAIT: begin
                    if (spi_done) begin
                        byte_cnt <= 0;
                        state    <= S_READ_BYTES;
                    end
                end

                S_READ_BYTES: begin
                    // Send Dummy Byte to clock out data
                    spi_data_in <= 8'h00;
                    spi_start   <= 1;
                    state       <= S_READ_BYTES_WAIT;
                end

                S_READ_BYTES_WAIT: begin
                    if (spi_done) begin
                        // Optimized: Write directly to output registers
                        case (byte_cnt)
                            0: accel_x[15:8] <= spi_data_out;
                            1: accel_x[7:0]  <= spi_data_out;
                            2: accel_y[15:8] <= spi_data_out;
                            3: accel_y[7:0]  <= spi_data_out;
                            4: accel_z[15:8] <= spi_data_out;
                            5: accel_z[7:0]  <= spi_data_out;
                            // 6, 7: Temp - Ignore
                            8: gyro_x[15:8] <= spi_data_out;
                            9: gyro_x[7:0]  <= spi_data_out;
                            10: gyro_y[15:8] <= spi_data_out;
                            11: gyro_y[7:0]  <= spi_data_out;
                            12: gyro_z[15:8] <= spi_data_out;
                            13: gyro_z[7:0]  <= spi_data_out;
                        endcase

                        if (byte_cnt == 13) begin
                            state <= S_UPDATE;
                        end else begin
                            byte_cnt <= byte_cnt + 1;
                            state    <= S_READ_BYTES;
                        end
                    end
                end

                S_UPDATE: begin
                    spi_cs_n <= 1; // Deassert CS
                    valid <= 1;
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule
